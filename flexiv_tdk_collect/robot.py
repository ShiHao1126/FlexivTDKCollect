from __future__ import annotations

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any, Literal, Protocol

import numpy as np


@dataclass(frozen=True)
class FlexivState:
    timestamp_sdk_s: float
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    tau_ext: np.ndarray
    tau_interact: np.ndarray
    tcp_pose: np.ndarray
    tcp_vel: np.ndarray
    ext_wrench_in_tcp: np.ndarray
    ext_wrench_in_world: np.ndarray


class FlexivStateSource(Protocol):
    def read_state(self) -> FlexivState:
        ...


class TdkFlexivStateSource:
    """Read one robot from a TDK-owned teleoperation connection."""

    def __init__(
        self,
        teleop: Any,
        pair_index: int = 0,
        robot: Literal["leader", "follower"] = "follower",
    ):
        if pair_index < 0:
            raise ValueError("pair_index must be non-negative")
        if robot not in {"leader", "follower"}:
            raise ValueError("robot must be 'leader' or 'follower'")
        self._teleop = teleop
        self._pair_index = pair_index
        self._state_index = 0 if robot == "leader" else 1

    def read_state(self) -> FlexivState:
        pair = self._teleop.robot_states(self._pair_index)
        if not isinstance(pair, tuple) or len(pair) != 2:
            raise RuntimeError(
                f"TDK robot_states({self._pair_index}) must return (leader, follower)"
            )
        return _normalize_state(pair[self._state_index])


class RdkFlexivRobot:
    """Own one standalone RDK connection and expose state and joint control."""

    def __init__(
        self,
        robot_sn: str,
        *,
        rdk_module: Any = None,
        robot: Any = None,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ):
        if rdk_module is None:
            import flexivrdk as rdk_module
        self._rdk = rdk_module
        self._robot = robot if robot is not None else rdk_module.Robot(robot_sn)
        self._sleep = sleep
        self._clock = clock
        self._closed = False

    def read_state(self) -> FlexivState:
        return _normalize_state(self._robot.states())

    def prepare(
        self,
        *,
        clear_fault: bool,
        tool_name: str | None = None,
        operational_timeout_s: float,
        poll_interval_s: float,
        zero_force_timeout_s: float = 30.0,
    ) -> None:
        _require_positive("operational_timeout_s", operational_timeout_s)
        _require_positive("poll_interval_s", poll_interval_s)
        _require_positive("zero_force_timeout_s", zero_force_timeout_s)
        if self._robot.fault():
            if not clear_fault:
                raise RuntimeError("robot has a fault and fault clearing is disabled")
            if not self._robot.ClearFault():
                raise RuntimeError("robot fault cannot be cleared")
        if not self._robot.estop_released():
            raise RuntimeError("robot E-stop is not released")

        self._robot.Enable()
        self._wait_until(
            condition=self._operational,
            timeout_s=operational_timeout_s,
            poll_interval_s=poll_interval_s,
            timeout_message=lambda: (
                "timed out waiting for robot operational state; current status: "
                f"{self._robot.operational_status()}"
            ),
        )
        if tool_name is not None:
            self.switch_tool(tool_name)
        self.zero_force_torque(
            timeout_s=zero_force_timeout_s, poll_interval_s=poll_interval_s
        )

    def move_joints(
        self,
        target_q_rad: Sequence[float],
        *,
        max_velocity_rad_s: float,
        max_acceleration_rad_s2: float,
        tolerance_rad: float,
        timeout_s: float,
        command_period_s: float,
    ) -> None:
        target = _joint_vector("target_q_rad", target_q_rad)
        for name, value in (
            ("max_velocity_rad_s", max_velocity_rad_s),
            ("max_acceleration_rad_s2", max_acceleration_rad_s2),
            ("tolerance_rad", tolerance_rad),
            ("timeout_s", timeout_s),
            ("command_period_s", command_period_s),
        ):
            _require_positive(name, value)
        if not self._robot.operational():
            raise RuntimeError("robot must be operational before joint motion")
        self._check_joint_limits(target)

        zeros = [0.0] * 7
        max_velocity = [max_velocity_rad_s] * 7
        max_acceleration = [max_acceleration_rad_s2] * 7
        deadline = self._clock() + timeout_s
        self._robot.SwitchMode(self._rdk.Mode.NRT_JOINT_IMPEDANCE)
        while True:
            if self._robot.fault():
                raise RuntimeError("robot fault occurred during joint motion")
            current_q = np.asarray(self._robot.states().q, dtype=np.float64)
            if current_q.shape != (7,):
                raise RuntimeError(
                    f"RDK joint state must have shape (7,), got {current_q.shape}"
                )
            if np.max(np.abs(current_q - target)) <= tolerance_rad:
                return
            if self._clock() >= deadline:
                error = np.max(np.abs(current_q - target))
                raise TimeoutError(
                    f"timed out waiting for joint target; max error is {error:.6f} rad"
                )
            self._robot.SendJointPosition(
                target.tolist(), zeros, max_velocity, max_acceleration
            )
            self._sleep(command_period_s)

    def switch_tool(self, tool_name: str) -> None:
        if not tool_name:
            raise ValueError("tool_name must not be empty")
        if not self._robot.operational():
            raise RuntimeError("robot must be operational before switching Tool")
        self._robot.SwitchMode(self._rdk.Mode.IDLE)
        tool = self._rdk.Tool(self._robot)
        if not tool.exist(tool_name):
            raise RuntimeError(f"Tool {tool_name!r} does not exist on the robot")
        tool.Switch(tool_name)

    def zero_force_torque(
        self,
        *,
        timeout_s: float,
        poll_interval_s: float,
    ) -> None:
        _require_positive("timeout_s", timeout_s)
        _require_positive("poll_interval_s", poll_interval_s)
        if not self._robot.operational():
            raise RuntimeError(
                "robot must be operational before force/torque sensor zeroing"
            )

        self._robot.SwitchMode(self._rdk.Mode.NRT_PRIMITIVE_EXECUTION)
        self._robot.ExecutePrimitive("ZeroFTSensor", {})
        self._wait_until(
            condition=self._force_torque_zeroing_terminated,
            timeout_s=timeout_s,
            poll_interval_s=poll_interval_s,
            timeout_message=lambda: (
                "timed out waiting for force/torque sensor zeroing to finish"
            ),
        )
        self._robot.SwitchMode(self._rdk.Mode.IDLE)

    def stop(self) -> None:
        self._robot.Stop()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.stop()
        finally:
            self._robot = None

    def _operational(self) -> bool:
        if self._robot.fault():
            raise RuntimeError("robot fault occurred while enabling")
        return bool(self._robot.operational())

    def _force_torque_zeroing_terminated(self) -> bool:
        if self._robot.fault():
            raise RuntimeError(
                "robot fault occurred during force/torque sensor zeroing"
            )
        states = self._robot.primitive_states()
        if "terminated" not in states:
            raise RuntimeError(
                "ZeroFTSensor primitive state does not contain 'terminated'"
            )
        return bool(states["terminated"])

    def _check_joint_limits(self, target_q_rad: np.ndarray) -> None:
        info = self._robot.info()
        lower = np.asarray(info.q_min, dtype=np.float64)
        upper = np.asarray(info.q_max, dtype=np.float64)
        if lower.shape != (7,) or upper.shape != (7,):
            raise RuntimeError("RDK joint limits must each have shape (7,)")
        invalid = np.flatnonzero((target_q_rad < lower) | (target_q_rad > upper))
        if invalid.size:
            details = ", ".join(
                f"q{index + 1}={target_q_rad[index]:.6f} not in "
                f"[{lower[index]:.6f}, {upper[index]:.6f}]"
                for index in invalid
            )
            raise ValueError(f"target joint position exceeds robot limits: {details}")

    def _wait_until(
        self,
        *,
        condition: Callable[[], bool],
        timeout_s: float,
        poll_interval_s: float,
        timeout_message: Callable[[], str],
    ) -> None:
        deadline = self._clock() + timeout_s
        while not condition():
            if self._clock() >= deadline:
                raise TimeoutError(timeout_message())
            self._sleep(poll_interval_s)


def _normalize_state(state: Any) -> FlexivState:
    return FlexivState(
        timestamp_sdk_s=_sdk_timestamp_s(state.timestamp),
        q=_array(state, "q", 7),
        dq=_array(state, "dq", 7),
        tau=_array(state, "tau", 7),
        tau_ext=_array(state, "tau_ext", 7),
        tau_interact=_array(state, "tau_interact", 7),
        tcp_pose=_array(state, "tcp_pose", 7),
        tcp_vel=_array(state, "tcp_vel", 6),
        ext_wrench_in_tcp=_array(state, "ext_wrench_in_tcp", 6),
        ext_wrench_in_world=_array(state, "ext_wrench_in_world", 6),
    )


def _array(state: Any, name: str, length: int) -> np.ndarray:
    value = np.asarray(getattr(state, name), dtype=np.float64).copy()
    if value.shape != (length,):
        raise RuntimeError(
            f"Flexiv state {name} must have shape ({length},), got {value.shape}"
        )
    return value


def _joint_vector(name: str, values: Sequence[float]) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (7,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain exactly 7 finite values")
    return result


def _require_positive(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")


def _sdk_timestamp_s(value: Any) -> float:
    if isinstance(value, (tuple, list)) and len(value) == 2:
        return float(value[0]) + float(value[1]) * 1e-9
    return float(value)
