from __future__ import annotations

import queue
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any

from .config import CollectConfig
from .storage import write_json
from .workers import CameraWorker, RobotWorker, Worker, XenseWorker


class CollectionSession:
    def __init__(self, teleop: Any, config: CollectConfig):
        self.teleop = teleop
        self.config = config
        self.record_dir: Path | None = None
        self.workers: list[Worker] = []
        self.errors: queue.Queue = queue.Queue()
        self.started_at: str | None = None

    @property
    def recording(self) -> bool:
        return self.record_dir is not None

    def start(self) -> Path:
        if self.recording:
            raise RuntimeError("a record is already active")
        if not any((self.config.enable_robot, self.config.enable_camera, self.config.enable_xense)):
            raise RuntimeError("at least one data source must be enabled")
        if self.config.enable_robot and not self.config.robot_fields:
            raise RuntimeError("enable_robot requires at least one robot field group")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.record_dir = self.config.output_dir / ".inprogress" / f"record_{stamp}"
        self.record_dir.mkdir(parents=True)
        self.started_at = datetime.now().astimezone().isoformat()
        start_ns = time.monotonic_ns()
        args = (self.record_dir, start_ns, self.errors)
        self.workers = []
        if self.config.enable_robot:
            self.workers.append(RobotWorker(self.teleop, self.config, *args))
        if self.config.enable_camera:
            serials = self.config.camera_serials or (None,)
            for index, serial in enumerate(serials):
                self.workers.append(CameraWorker(self.config, *args, camera_index=index, camera_serial=serial))
        if self.config.enable_xense:
            self.workers.append(XenseWorker(self.config, *args))
        started_workers = []
        try:
            for worker in self.workers:
                worker.start()
                started_workers.append(worker)
        except Exception as exc:
            for worker in started_workers:
                worker.stop()
            for worker in reversed(started_workers):
                worker.join(timeout=60.0)
            self.workers = started_workers
            target = self._finalize("startup_error", [
                ("session", str(exc), traceback.format_exc()),
            ])
            raise RuntimeError(f"collection startup failed; invalid record saved to {target}") from exc
        return self.record_dir

    def finish(self, reason: str = "completed") -> Path:
        if self.record_dir is None:
            raise RuntimeError("no active record")
        for worker in self.workers:
            worker.stop()
        for worker in reversed(self.workers):
            worker.join(timeout=60.0)
        return self._finalize(reason)

    def _finalize(self, reason: str, failures: list[tuple[str, str, str]] | None = None) -> Path:
        if self.record_dir is None:
            raise RuntimeError("no active record")
        failures = list(failures or [])
        while not self.errors.empty():
            failures.append(self.errors.get_nowait())
        for worker in self.workers:
            if worker.is_alive():
                failures.append((worker.name, "shutdown timeout", ""))
        healthy = all(
            worker.stats["samples"] > 0
            and worker.stats["write_errors"] == 0
            and not worker.stats["cleanup_errors"]
            for worker in self.workers
        )
        valid = reason == "completed" and not failures and healthy
        metadata = {"started_at": self.started_at, "finished_at": datetime.now().astimezone().isoformat(),
                    "termination_reason": reason, "valid": valid, "leader_sn": self.config.leader_sn,
                    "follower_sn": self.config.follower_sn, "sources": {w.name: w.stats for w in self.workers},
                    "acquisition": self._acquisition_metadata(),
                    "user_metadata": self.config.metadata}
        validation = {"valid": valid, "errors": [{"source": n, "message": m, "traceback": t} for n, m, t in failures]}
        write_json(self.record_dir / "metadata.json", metadata)
        write_json(self.record_dir / "validation.json", validation)
        target_name = self.record_dir.name if valid else f"invalid_{self.record_dir.name}"
        target = self.config.output_dir / target_name
        self.config.output_dir.mkdir(parents=True, exist_ok=True)
        self.record_dir.replace(target)
        self.record_dir = None
        self.workers = []
        return target

    def _acquisition_metadata(self) -> dict[str, Any]:
        return {
            "robot": {
                "enabled": self.config.enable_robot,
                "rate_hz": self.config.robot_rate_hz,
                "fields": list(self.config.robot_fields) if self.config.enable_robot else [],
            },
            "camera": {
                "requested_serials": list(self.config.camera_serials),
                "count": self.config.camera_count if self.config.enable_camera else 0,
                "color": {
                    "enabled": self.config.enable_camera_color,
                    "width": self.config.camera_color_width,
                    "height": self.config.camera_color_height,
                    "fps": self.config.camera_color_fps,
                },
                "depth": {
                    "enabled": self.config.enable_camera_depth,
                    "width": self.config.camera_depth_width,
                    "height": self.config.camera_depth_height,
                    "fps": self.config.camera_depth_fps,
                },
                "align_depth_to_color": self.config.align_depth_to_color,
            },
            "xense": {
                "rate_hz": self.config.xense_rate_hz,
                "gripper_width": self.config.enable_gripper_width,
                "tactile": {
                    "marker": self.config.enable_tactile_marker,
                    "force": self.config.enable_tactile_force,
                    "rectify": self.config.enable_tactile_rectify,
                    "difference": self.config.enable_tactile_difference,
                    "depth": self.config.enable_tactile_depth,
                },
            },
        }
