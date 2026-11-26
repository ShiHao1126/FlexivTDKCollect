"""Offline validation of the recovered bilateral HDF5 data contract."""
from __future__ import annotations

import json
from pathlib import Path

import h5py
import numpy as np

from . import RAW_FORMAT

FIELDS = {"host_time_ns": (), "robot_time_ns": (), "tcp_pose_wxyz": (7,), "tcp_vel": (6,),
          "q": (7,), "dq": (7,), "ext_wrench_in_tcp": (6,), "ext_wrench_in_tcp_raw": (6,), "ft_sensor_raw": (6,)}


def validate_episode(path: Path) -> dict:
    with h5py.File(path, "r") as source:
        if source.attrs.get("format") != RAW_FORMAT or "failure_json" in source.attrs:
            raise ValueError("Wrong raw format or failed partial recording")
        if set(source) != {"leader", "robot"}:
            raise ValueError("Force-only record must contain exactly leader and robot groups")
        counts, times = {}, {}
        for role in ("leader", "robot"):
            group = source[role]
            if set(group) != set(FIELDS):
                raise ValueError(f"{role}: missing or unexpected fields")
            count = len(group["host_time_ns"])
            if count < 1:
                raise ValueError(f"{role}: empty stream")
            for name, shape in FIELDS.items():
                data = group[name][:]
                if data.shape != (count, *shape) or not np.isfinite(data).all():
                    raise ValueError(f"{role}/{name}: invalid shape or non-finite values")
                if name.endswith("time_ns"):
                    if data.dtype.kind not in "iu" or np.any(data < 0) or np.any(np.diff(data.astype(np.int64)) <= 0):
                        raise ValueError(f"{role}/{name}: invalid or non-increasing timestamps")
            quaternions = group["tcp_pose_wxyz"][:, 3:]
            if np.any(np.abs(np.linalg.norm(quaternions, axis=1) - 1) > 1e-3):
                raise ValueError(f"{role}: invalid wxyz quaternion")
            counts[role] = count
            times[role] = group["host_time_ns"][:]
        if counts["leader"] != counts["robot"]:
            raise ValueError("Bilateral sample counts differ")
        metadata = json.loads(source.attrs["metadata_json"])
        if metadata.get("capture_mode") != "tdk_teleop":
            raise ValueError("Unexpected capture mode")
        stats = json.loads(source.attrs["recording_stats_json"])
        for role in counts:
            if stats[role]["samples"] != counts[role] or stats[role]["queue_drops"] or stats[role]["source_errors"]:
                raise ValueError(f"{role}: recorded counts or error statistics do not match")
        skew = np.abs(times["leader"] - times["robot"]) / 1e6
        if np.max(skew) > metadata["config"]["teleop"]["max_age_ms"]:
            raise ValueError("Bilateral host-time skew exceeds max_age_ms")
        return {"samples_per_arm": counts["leader"], "max_pair_skew_ms": float(np.max(skew)),
                "duration_s": float((times["robot"][-1] - times["robot"][0]) / 1e9)}
