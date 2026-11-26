"""TDK lifecycle, state sampling, and paired reset through its existing robot connections."""

from __future__ import annotations

import gc
import time
from importlib.metadata import version
from typing import Any, Callable

import numpy as np
from scipy.spatial.transform import Rotation

from .robot_control import CARTESIAN_AXIS_NAMES, robot_state_sample


FOLLOWER_DIRECT_HOME_DISTANCE_M = 0.10


class TDKRobotController:
    def __init__(self, robot_config: dict, config: dict, *, rdk=None, tdk=None) -> None:
        # instances() exposes native RDK objects, so the Python bindings must
        # match the RDK build embedded in this TDK release.
        if rdk is None or tdk is None:
            for package, expected in (("flexivrdk", "1.9.2"), ("flexivtdk", "1.6.2")):
                installed = version(package)
                if installed != expected:
                    raise RuntimeError(
                        f"TDK capture requires {package}=={expected}; found {installed}. "
                        "Run conda activate flexiv, then python -m pip install -r requirements.txt."
                    )
        if rdk is None:
            import flexivrdk as rdk
        if tdk is None:
            import flexivtdk as tdk
        self.rdk, self.tdk = rdk, tdk
        self.config = config
        self._robot_config = robot_config
        self._requires_reconnect = False
        self.initialized = False
        self._needs_tdk_stop = False
        self.zeroed = False
        self.started = False
        self.engaged = False
        self.teleop = self._create_teleop()
        # Reuse TDK's connections for both state sampling and home commands.
        # Creating another full RDK connection here can fail after TDK connects.
        self.robot_sns = (config["leader_sn"], robot_config["robot_sn"])
        self.leader, self.follower = self.teleop.instances(0)
        self.robots = (self.leader, self.follower)
        self.dofs = tuple(int(robot.info().DoF) for robot in self.robots)
        self.joint_dof = self.dofs[1]
        self.metadata = self._metadata(self.follower)
        self.metadata.update(
            control_mode="tdk", leader=self._metadata(self.leader),
            teleop=config, state_source="tdk_instances",
        )
        self.leader_metadata = {
            **self._metadata(self.leader), "max_age_ms": config["max_age_ms"],
            "role": "leader", "quaternion_order": "wxyz",
        }
        self._last_stamp: dict[int, tuple[int, float]] = {}

    def _create_teleop(self):
        try:
            return self.tdk.TransparentCartesianTeleopLAN(
                [(self.config["leader_sn"], self._robot_config["robot_sn"])],
                list(self._robot_config.get("network_interfaces", [])),
            )
        except RuntimeError as exc:
            if "Failed to initialize scheduler" in str(exc):
                raise RuntimeError(
                    f"{exc}. Check Linux real-time scheduling privileges. "
                    "If the SDK reports 'Operation not permitted', run in the same terminal: "
                    "sudo prlimit --pid $$ --rtprio=99:99 ; "
                    "then retry with the flexiv Conda environment. "
                    "Alternatively, ./record.sh configures this before starting capture. "
                    "The setting applies only to this terminal and its child processes."
                ) from exc
            raise

    def _reconnect(self) -> None:
        """Replace TDK-owned RDK handles after TransparentCartesianTeleopLAN.Stop()."""
        # The native TDK Stop invalidates instances(). Release every Python
        # reference before constructing a replacement connection in-process.
        self.robots = ()
        self.leader = self.follower = None
        self.teleop = None
        gc.collect()
        teleop = self._create_teleop()
        leader, follower = teleop.instances(0)
        dofs = (int(leader.info().DoF), int(follower.info().DoF))
        if dofs != self.dofs:
            try:
                teleop.Stop()
            finally:
                raise RuntimeError(
                    f"Robot DoF changed while reconnecting TDK: expected {self.dofs}, found {dofs}"
                )
        self.teleop = teleop
        self.leader, self.follower = leader, follower
        self.robots = (leader, follower)
        self._requires_reconnect = False
        self._last_stamp.clear()

    def _ensure_connected(self) -> None:
        if self._requires_reconnect or not all(robot.connected() for robot in self.robots):
            self._reconnect()

    @staticmethod
    def _metadata(robot) -> dict:
        info = robot.info()
        return {
            "serial_number": str(info.serial_num), "model_name": str(info.model_name),
            "software_version": str(info.software_ver), "joint_dof": int(info.DoF),
            "has_ft_sensor": bool(info.has_FT_sensor),
        }

    def check_health(self) -> None:
        # A deliberate TDK Stop invalidates instances; replace only those handles.
        # Unexpected connection loss below remains a fault, not an automatic retry.
        if self._requires_reconnect:
            self._ensure_connected()
        failures = []
        for role, sn, robot in zip(("leader", "follower"), self.robot_sns, self.robots):
            if not robot.connected():
                failures.append(f"{role} [{sn}]: RDK connection lost")
                continue
            problems = []
            if robot.fault():
                problems.append("fault=true (check Flexiv Elements alarms)")
            if robot.recovery():
                problems.append("recovery=true")
            if not robot.estop_released():
                problems.append("estop_released=false (E-stop pressed)")
            if self.started and not robot.operational():
                problems.append("operational=false")
            if problems:
                failures.append(f"{role} [{sn}]: {', '.join(problems)}")
        if failures:
            raise RuntimeError("Robot health check failed: " + "; ".join(failures))
        if self.started and (self.teleop.any_fault() or self.teleop.stopped(0)):
            raise RuntimeError("TDK control loop stopped or faulted")

    def _init(self) -> None:
        self._ensure_connected()
        self.check_health()
        zero = self.tdk.ZeroFTSensor.Disable if self.zeroed else self.tdk.ZeroFTSensor.Enable
        # A partially failed Init can still have enabled the robots: Stop must run.
        self.initialized = True
        self._needs_tdk_stop = True
        self.teleop.Init(self.config["limit_wrist_singular"], zero)
        self.zeroed = True

    def prepare(self) -> None:
        if self.started:
            self.check_health()
            return
        try:
            self._init()
            self.teleop.Start()
            self.started = True
            lock = self.tdk.AxisLock()
            lock.coord = {
                "world": self.tdk.CoordType.WORLD,
                "tcp": self.tdk.CoordType.TCP,
            }[self.config.get("lock_coordinate", "world")]
            axes = self.config["locked_axes"]
            lock.lock_trans_axis = [axis in axes for axis in CARTESIAN_AXIS_NAMES[:3]]
            lock.lock_ori_axis = [axis in axes for axis in CARTESIAN_AXIS_NAMES[3:]]
            self.teleop.SetAxisLockCmd(0, lock)
            self.teleop.SetWrenchFeedbackScalingFactor(0, self.config["feedback_scale"])
            stiffness = self.config["follower_stiffness"]
            if isinstance(stiffness, list):
                # TDK takes physical WORLD-axis stiffness, not TCP-axis scale ratios.
                nominal = self.follower.info().K_x_nom
                stiffness = [float(ratio) * value for ratio, value in zip(stiffness, nominal)]
            else:
                stiffness = float(stiffness)
            self.teleop.SetFollowerCartStiff(0, stiffness)
            self.teleop.SetFollowerMaxContactWrench(0, self.config["max_contact_wrench"])
            # Preserve each arm's own posture, including distinct configured homes.
            self.teleop.SetLeaderNullSpacePosture(0, list(self.leader.states().q))
            self.teleop.SetFollowerNullSpacePosture(0, list(self.follower.states().q))
            self._last_stamp.clear()
        except BaseException:
            self.stop()
            raise

    def engage(self, active: bool) -> None:
        if active:
            self.prepare()
        if self.started:
            self.teleop.Engage(0, active)
        self.engaged = active

    def read_pair(self) -> tuple[dict, dict] | None:
        self.check_health()
        samples = []
        fresh = True
        for index, robot in enumerate(self.robots):
            sample = robot_state_sample(robot.states())
            now = time.monotonic()
            stamp = sample["robot_time_ns"]
            previous, changed = self._last_stamp.get(index, (None, now))
            if previous is not None and stamp < previous:
                raise RuntimeError("TDK robot timestamp moved backwards")
            if stamp == previous:
                fresh = False
            if stamp != previous:
                changed = now
            if now - changed > self.config["state_stale_timeout_s"]:
                raise RuntimeError("TDK robot state stopped updating")
            self._last_stamp[index] = (stamp, changed)
            for name, shape in (
                ("tcp_pose_wxyz", (7,)), ("q", (self.dofs[index],)),
                ("dq", (self.dofs[index],)), ("tcp_vel", (6,)),
                ("ext_wrench_in_tcp", (6,)), ("ext_wrench_in_tcp_raw", (6,)),
                ("ft_sensor_raw", (6,)),
            ):
                if sample[name].shape != shape or not np.isfinite(sample[name]).all():
                    raise RuntimeError(f"Invalid TDK robot state: {name}")
            samples.append(sample)
        # Keep network gaps visible to the downstream interpolation gap checks.
        return (samples[0], samples[1]) if fresh else None

    def stop(self) -> None:
        failure = None
        stopped_tdk = self._needs_tdk_stop
        try:
            if self._needs_tdk_stop:
                self.teleop.Stop()
        except BaseException as exc:
            failure = exc
        finally:
            self.initialized = False
            self._needs_tdk_stop = False
            self.started = self.engaged = False
            if stopped_tdk:
                # TDK owns these RDK instances; Stop invalidates their handles.
                self._requires_reconnect = True
            else:
                # A faulted robot already rejects Stop; do not mask the primary
                # fault with a secondary cleanup exception.
                for robot in self.robots:
                    try:
                        if robot.connected() and not robot.fault():
                            robot.Stop()
                    except BaseException as exc:
                        failure = failure or exc
        if failure is not None:
            raise RuntimeError("Failed to stop both TDK robots") from failure

    def reset_home(
        self, config: dict, *, prepare_after: bool = True,
        check_peripherals: Callable[[], None] | None = None,
    ) -> dict[str, Any]:
        """Reset to shared TCP Home, using a follower waypoint only beyond 10 cm."""
        def check_reset_health() -> None:
            self.check_health()
            if check_peripherals is not None:
                check_peripherals()

        # Preserve the original TDK prerequisite: Init performs the first
        # ZeroFTSensor calibration before any Cartesian force-control command.
        # Stop invalidates instances(), so reconnect before direct RDK homing.
        if self.initialized or self._needs_tdk_stop:
            self.stop()
        try:
            self._init()
            self.stop()
            self._ensure_connected()
        except BaseException:
            self.stop()
            raise
        cartesian_home = config.get("follower_cartesian_home")
        targets = [config[f"{role}_home_q_deg"] for role in ("leader", "follower")]
        if cartesian_home is not None:
            # Both Cartesian moves use the measured Home's joint reference.
            targets[0] = list(targets[1])
        for robot, target in zip(self.robots, targets):
            info = robot.info()
            radians = np.deg2rad(target)
            if len(target) != int(info.DoF) or np.any(radians < info.q_min) or np.any(radians > info.q_max):
                raise ValueError(f"Home target outside joint limits for {info.serial_num}")
        result: dict[str, Any] = {"mode": "paired_home", "velocity_scale": config["movej_velocity_scale"]}
        stage_indices = [0, 0]
        if cartesian_home is not None:
            home_pose = np.asarray(cartesian_home["tcp_pose_wxyz"], dtype=float)
            above_pose = home_pose.copy()
            above_pose[2] += cartesian_home["approach_height_m"]
            stage_poses = ((home_pose,), (above_pose, home_pose))
            stage_names = (("home",), ("above_home", "home"))
            home_rotation = Rotation.from_quat(home_pose[[4, 5, 6, 3]])
            for index, role in enumerate(("leader", "follower")):
                result[role] = {
                    "mode": "cartesian_direct" if index == 0 else "cartesian_via_above",
                    "reference_q_deg": list(targets[index]),
                    "control_api": "SendCartesianMotionForce",
                    "target_tcp_pose_wxyz": home_pose.tolist(), "stages": [],
                }

        def dispatch_cartesian_stage(index: int) -> None:
            check_reset_health()
            role = ("leader", "follower")[index]
            pose = stage_poses[index][stage_indices[index]]
            name = stage_names[index][stage_indices[index]]
            print(f"{role.capitalize()} reset: {name}, WORLD XYZ (m) = {pose[:3].tolist()}", flush=True)
            self.robots[index].SendCartesianMotionForce(
                pose.tolist(),
                max_linear_vel=float(cartesian_home["linear_velocity_m_s"]),
                max_angular_vel=float(np.deg2rad(cartesian_home["angular_velocity_deg_s"])),
                max_linear_acc=0.1,
                max_angular_acc=float(np.deg2rad(20.0)),
            )
            result[role]["stages"].append({
                "name": name, "target_tcp_pose_wxyz": pose.tolist(),
                "dispatch_time_ns": time.monotonic_ns(),
            })

        try:
            # Homing uses the native RDK instances directly. Do not initialize
            # then stop TDK first: TDK Stop invalidates these same instances.
            for robot in self.robots:
                robot.Stop()
            check_reset_health()
            deadline = time.monotonic() + config["enable_timeout_s"]
            for robot in self.robots:
                if not robot.operational():
                    robot.Enable()
            while not all(robot.operational() for robot in self.robots):
                check_reset_health()
                if time.monotonic() >= deadline:
                    raise TimeoutError("Timed out enabling both robots")
                time.sleep(0.02)
            if cartesian_home is not None:
                # Decide from the stopped arm's current WORLD XYZ, before any move.
                pose = np.asarray(self.follower.states().tcp_pose, dtype=float)
                if (pose.shape != (7,) or not np.isfinite(pose).all()
                        or abs(np.linalg.norm(pose[3:]) - 1.0) > 0.001):
                    raise RuntimeError("Invalid follower TCP pose before reset")
                distance = float(np.linalg.norm(pose[:3] - home_pose[:3]))
                result["follower"].update(
                    initial_home_distance_m=distance,
                    direct_home_distance_m=FOLLOWER_DIRECT_HOME_DISTANCE_M,
                )
                # Allow floating-point subtraction roundoff at exactly 10 cm.
                if distance <= FOLLOWER_DIRECT_HOME_DISTANCE_M + 1e-12:
                    stage_poses = ((home_pose,), (home_pose,))
                    stage_names = (("home",), ("home",))
                    result["follower"]["mode"] = "cartesian_direct"
            modes = [self.rdk.Mode.NRT_JOINT_POSITION] * 2
            if cartesian_home is not None:
                modes = [self.rdk.Mode.NRT_CARTESIAN_MOTION_FORCE] * 2
            for index, robot in enumerate(self.robots):
                robot.SwitchMode(modes[index])
                if cartesian_home is not None:
                    robot.SetForceControlAxis([False] * 6)
                    robot.SetCartesianImpedance(list(robot.info().K_x_nom))
                    robot.SetNullSpacePosture(np.deg2rad(targets[index]).tolist())
            deadline = time.monotonic() + config["motion_timeout_s"]
            self._last_stamp.clear()
            for index, (role, robot, target) in enumerate(zip(("leader", "follower"), self.robots, targets)):
                check_reset_health()
                if cartesian_home is not None:
                    dispatch_cartesian_stage(index)
                    continue
                print(f"{role.capitalize()} reset: direct joint Home (deg) = {target}", flush=True)
                # Preserve the old MoveJ default (20%) times its configured overall scale.
                max_vel = np.asarray(robot.info().dq_max) * 0.2 * config["movej_velocity_scale"] / 100.0
                # Direct joint limits use rad/s and rad/s².
                max_acc = [float(np.deg2rad(20.0))] * len(target)
                robot.SendJointPosition(np.deg2rad(target).tolist(), [0.0] * len(target),
                                        max_vel.tolist(), max_acc)
                result[role] = {"target_q_deg": list(target), "dispatch_time_ns": time.monotonic_ns(),
                                "control_api": "SendJointPosition", "max_vel_rad_s": max_vel.tolist()}
            done = [False, False]
            settled_since: list[float | None] = [None, None]
            while not all(done):
                check_reset_health()
                if time.monotonic() >= deadline:
                    raise TimeoutError("Timed out during paired home reset")
                samples = self.read_pair()
                if samples is None:
                    time.sleep(0.02)
                    continue
                now = time.monotonic()
                for index, (role, robot, target) in enumerate(zip(("leader", "follower"), self.robots, targets)):
                    if not robot.operational() or robot.mode() != modes[index]:
                        raise RuntimeError(f"{role} left home control mode")
                    if done[index]:
                        continue
                    measured = samples[index]
                    q = measured["q"]
                    stopped = np.max(np.abs(measured["dq"])) <= 0.005
                    if cartesian_home is not None:
                        pose = measured["tcp_pose_wxyz"]
                        if (pose.shape != (7,) or not np.isfinite(pose).all()
                                or abs(np.linalg.norm(pose[3:]) - 1.0) > 0.001):
                            raise RuntimeError(f"Invalid {role} TCP pose during reset")
                        target_pose = stage_poses[index][stage_indices[index]]
                        position_error = float(np.linalg.norm(pose[:3] - target_pose[:3]))
                        rotation_error = float(np.rad2deg((
                            Rotation.from_quat(pose[[4, 5, 6, 3]]) * home_rotation.inv()
                        ).magnitude()))
                        near = (position_error <= cartesian_home["position_tolerance_m"]
                                and rotation_error <= cartesian_home["rotation_tolerance_deg"])
                        stopped = (stopped and np.linalg.norm(measured["tcp_vel"][:3]) <= 0.005
                                   and np.linalg.norm(measured["tcp_vel"][3:]) <= 0.02)
                        if near and stopped:
                            if settled_since[index] is None:
                                settled_since[index] = now
                        else:
                            settled_since[index] = None
                        if settled_since[index] is not None and now - settled_since[index] >= 0.1:
                            stage = result[role]["stages"][-1]
                            stage.update(actual_tcp_pose_wxyz=pose.tolist(),
                                         reached_time_ns=time.monotonic_ns(),
                                         position_error_m=position_error,
                                         rotation_error_deg=rotation_error)
                            print(f"{role.capitalize()} reset: reached {stage['name']}", flush=True)
                            if stage_indices[index] + 1 < len(stage_poses[index]):
                                stage_indices[index] += 1
                                settled_since[index] = None
                                dispatch_cartesian_stage(index)
                            else:
                                done[index] = True
                                result[role].update(actual_q_rad=q.tolist(),
                                                    actual_tcp_pose_wxyz=pose.tolist())
                        continue
                    near = np.max(np.abs(np.rad2deg(q) - target)) <= config["position_tolerance_deg"]
                    if near and stopped:
                        if settled_since[index] is None:
                            settled_since[index] = now
                    else:
                        settled_since[index] = None
                    done[index] = settled_since[index] is not None and now - settled_since[index] >= 0.1
                    if done[index]:
                        result[role]["actual_q_rad"] = q.tolist()
                        print(f"{role.capitalize()} reset: reached Home", flush=True)
                time.sleep(0.02)
        finally:
            self.stop()
        if prepare_after:
            self.prepare()
        return result

    def close(self) -> None:
        self.stop()
