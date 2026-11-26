"""Reconstructed state adapter. No SDK connection is created here."""
from __future__ import annotations

import time
from numbers import Integral
from typing import Any

import numpy as np

CARTESIAN_AXIS_NAMES = ("x", "y", "z", "rx", "ry", "rz")


def robot_state_sample(state: Any) -> dict:
    """Copy RDK fields unchanged: wxyz pose, TCP wrench, separate robot/host clocks."""
    try:
        sec, nsec = state.timestamp
        if (isinstance(sec, bool) or isinstance(nsec, bool)
                or not isinstance(sec, Integral) or not isinstance(nsec, Integral)
                or sec < 0 or not 0 <= nsec < 1_000_000_000):
            raise ValueError("timestamp must contain nonnegative integer seconds and nanoseconds")
        sample = {
            "host_time_ns": time.monotonic_ns(),
            "robot_time_ns": int(sec) * 1_000_000_000 + int(nsec),
            "tcp_pose_wxyz": np.array(state.tcp_pose, dtype=np.float64, copy=True),
        }
        for name in ("tcp_vel", "q", "dq", "ext_wrench_in_tcp", "ext_wrench_in_tcp_raw", "ft_sensor_raw"):
            sample[name] = np.array(getattr(state, name), dtype=np.float64, copy=True)
        pose = sample["tcp_pose_wxyz"]
        if pose.shape != (7,) or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:]) - 1) > 1e-3:
            raise ValueError("tcp_pose must contain XYZ and a unit wxyz quaternion")
        for name in ("tcp_vel", "ext_wrench_in_tcp", "ext_wrench_in_tcp_raw", "ft_sensor_raw"):
            if sample[name].shape != (6,) or not np.isfinite(sample[name]).all():
                raise ValueError(f"invalid {name}: expected six finite values")
        for name in ("q", "dq"):
            if sample[name].shape != (7,) or not np.isfinite(sample[name]).all():
                raise ValueError(f"invalid {name}: expected seven finite values")
        return sample
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise RuntimeError(f"Invalid TDK robot state: {exc}") from exc
