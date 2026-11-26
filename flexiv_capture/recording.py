"""Force-only recorder, adapted from the recovered October 26 acquisition flow."""
from __future__ import annotations

import importlib.metadata
import json
import math
import queue
import threading
import time
import h5py
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import validate_force_only_config
from .hdf5_io import RawEpisodeWriter
from .teleop_control import TDKRobotController

def _version(package: str) -> str:
    try:
        return importlib.metadata.version(package)
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


@dataclass
class WorkerStats:
    samples: int = 0
    queue_drops: int = 0
    source_errors: int = 0
    first_time_ns: int | None = None
    last_time_ns: int | None = None
    last_frame_number: int | None = None
    missed_frames: int = 0
    max_gap_ns: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def record(self, host_time_ns: int, frame_number: int | None = None) -> None:
        with self.lock:
            if self.last_time_ns is not None:
                self.max_gap_ns = max(self.max_gap_ns, host_time_ns - self.last_time_ns)
            if frame_number is not None and self.last_frame_number is not None:
                step = frame_number - self.last_frame_number
                if step != 1:
                    self.missed_frames += max(0, step - 1)
                    raise RuntimeError(
                        f"RealSense frame number jumped from "
                        f"{self.last_frame_number} to {frame_number}"
                    )
            self.samples += 1
            if self.first_time_ns is None:
                self.first_time_ns = host_time_ns
            self.last_time_ns = host_time_ns
            self.last_frame_number = frame_number

    def record_queue_drop(self) -> None:
        with self.lock:
            self.queue_drops += 1

    def as_dict(self) -> dict[str, Any]:
        with self.lock:
            duration_s = 0.0
            rate_hz = 0.0
            if self.first_time_ns is not None and self.last_time_ns is not None:
                duration_s = max(0.0, (self.last_time_ns - self.first_time_ns) * 1e-9)
                if duration_s > 0 and self.samples > 1:
                    rate_hz = (self.samples - 1) / duration_s
            return {
                "samples": self.samples,
                "queue_drops": self.queue_drops,
                "source_errors": self.source_errors,
                "missed_frames": self.missed_frames,
                "duration_s": duration_s,
                "rate_hz": rate_hz,
                "max_gap_ms": self.max_gap_ns * 1e-6,
            }


class Recorder:
    def __init__(self, config: dict, output: Path, *, controller=None) -> None:
        validate_force_only_config(config)
        self.config = config
        self.output = output.expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.robot = controller if controller is not None else TDKRobotController(config["robot"], config["teleop"])
        self._closed = False
        self._faulted = False

    def check_teleop_health(self) -> None:
        if self._closed or self._faulted:
            raise RuntimeError("Recorder is closed or faulted; create a new session")
        self.robot.check_health()

    def start_teleop(self) -> None:
        self.check_teleop_health()
        self.robot.engage(True)

    def stop_teleop(self) -> None:
        self.robot.stop()

    def close(self) -> None:
        if not self._closed:
            try:
                self.robot.close()
            finally:
                self._closed = True

    def reset_paired_home(self) -> dict:
        self.check_teleop_health()
        try:
            self.stop_teleop()
            result = self.robot.reset_home(self.config["episode_reset"], prepare_after=False)
            self.check_teleop_health()
            result["post_reset_state"] = "hold"
            return result
        except BaseException:
            self._faulted = True
            self.stop_teleop()
            raise

    def _next_path(self) -> tuple[Path, Path]:
        existing = list(self.output.glob("episode_*.hdf5")) + list(self.output.glob("episode_*.partial.hdf5"))
        indices = []
        for path in existing:
            try:
                indices.append(int(path.name.split("_")[1].split(".")[0]))
            except (IndexError, ValueError):
                continue
        index = max(indices, default=-1) + 1
        final = self.output / f"episode_{index:04d}.hdf5"
        return self.output / f"episode_{index:04d}.partial.hdf5", final

    def record_episode(self, duration_s: float | None, *, stop_event: threading.Event | None = None,
                       start_reset: dict | None = None) -> tuple[Path, Path, dict]:
        """Return a closed partial file; finalize() validates it before renaming."""
        self.check_teleop_health()
        if duration_s is not None and (not math.isfinite(duration_s) or duration_s <= 0):
            raise ValueError("duration_s must be finite and positive")
        if duration_s is None and stop_event is None:
            raise ValueError("An unbounded recording requires a stop_event")
        partial, final = self._next_path()
        metadata = {
            "capture_mode": "tdk_teleop", "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "config": self.config, "robot": self.robot.metadata,
            "implementation": "force_only_rebuilt_20261126",
            "versions": {name: _version(name) for name in ("flexivrdk", "flexivtdk", "h5py")},
        }
        if start_reset is not None:
            metadata["start_reset"] = start_reset
        recording = self.config["recording"]
        compression = recording.get("compression")
        if str(compression).lower() in ("none", "null", ""):
            compression = None
        writer = RawEpisodeWriter(
            partial, joint_dof=self.robot.joint_dof, image_shape=None, compression=compression,
            metadata=metadata, camera_metadata=None, stream_metadata={"leader": self.robot.leader_metadata},
            compression_level=recording.get("compression_level", 4),
            compression_workers=recording.get("compression_workers", 4),
        )
        # Enqueue an entire pair atomically: saturation cannot silently separate the arms.
        pairs: queue.Queue = queue.Queue(maxsize=recording["robot_queue_size"])
        errors: queue.Queue = queue.Queue()
        stop = threading.Event()
        leader_stats, robot_stats = WorkerStats(), WorkerStats()
        wall_started = time.monotonic()
        thread = None

        def robot_worker() -> None:
            period = 1 / float(self.config["robot"]["rate_hz"])
            deadline = time.monotonic()
            try:
                while not stop.is_set():
                    pair = self.robot.read_pair()
                    if pair is not None:
                        leader, follower = pair
                        skew = abs(leader["host_time_ns"] - follower["host_time_ns"]) / 1e6
                        if skew > self.config["teleop"]["max_age_ms"]:
                            raise RuntimeError(f"Pair host-time skew {skew:.3f} ms exceeds max_age_ms")
                        try:
                            pairs.put_nowait(pair)
                        except queue.Full:
                            leader_stats.record_queue_drop()
                            robot_stats.record_queue_drop()
                            raise RuntimeError("paired robot write queue is full")
                        leader_stats.record(leader["host_time_ns"])
                        robot_stats.record(follower["host_time_ns"])
                    deadline += period
                    wait = deadline - time.monotonic()
                    if wait > 0:
                        stop.wait(wait)
                    elif wait < -period:
                        deadline = time.monotonic()
            except BaseException as exc:
                robot_stats.source_errors += 1
                errors.put(exc)
                stop.set()

        def drain() -> None:
            batch = []
            # Bounded batch even when a producer remains active.
            for _ in range(pairs.qsize()):
                batch.append(pairs.get_nowait())
            if batch:
                writer.append_robot([follower for _, follower in batch])
                writer.append_stream("leader", [leader for leader, _ in batch])

        try:
            self.start_teleop()
            wall_started = time.monotonic()
            thread = threading.Thread(target=robot_worker, name="tdk-paired-state-reader", daemon=True)
            thread.start()
            while not stop.is_set():
                if stop_event is not None and stop_event.is_set():
                    break
                if duration_s is not None and time.monotonic() - wall_started >= duration_s:
                    break
                if robot_stats.first_time_ns is None and time.monotonic() - wall_started > 2.0:
                    raise TimeoutError("No paired robot state arrived within 2 seconds")
                drain()
                writer.flush()
                stop.wait(recording["flush_interval_s"])
            stop.set()
            thread.join(timeout=2.0)
            if thread.is_alive():
                raise RuntimeError("Paired state reader did not stop; session cannot be reused")
            # Stop invalidates native handles: never call it while the reader is active.
            self.stop_teleop()
            if not errors.empty():
                exc = errors.get()
                raise RuntimeError(f"robot acquisition failed: {exc}") from exc
            drain()
            if not robot_stats.samples or leader_stats.samples != robot_stats.samples:
                raise RuntimeError("Recording contains no complete bilateral samples")
            stats = {"leader": leader_stats.as_dict(), "robot": robot_stats.as_dict(),
                     "wall_duration_s": time.monotonic() - wall_started}
            writer.close(stats)
            return partial, final, stats
        except BaseException as exc:
            self._faulted = True
            stop.set()
            if thread is not None and thread.ident is not None:
                thread.join(timeout=2.0)
            try:
                self.stop_teleop()
            except BaseException as cleanup:
                exc.add_note(f"Stop also failed: {cleanup}") if hasattr(exc, "add_note") else None
            try:
                if writer.file:
                    writer.file.attrs["failure_json"] = json.dumps({"type": type(exc).__name__, "message": str(exc)})
                else:
                    # close()/fsync can fail after the HDF5 handle has closed.
                    with h5py.File(partial, "r+") as failed:
                        failed.attrs["failure_json"] = json.dumps({"type": type(exc).__name__, "message": str(exc)})
                writer.abort()
            finally:
                # Keep the diagnostic partial; never promote failed data or delete it.
                raise


def finalize(partial: Path, final: Path) -> dict:
    from .validation import validate_episode
    if final.exists():
        raise FileExistsError(final)
    result = validate_episode(partial)
    # Hard-link publication fails atomically on an existing target; remove only this partial.
    # Both paths are in one session directory on the same filesystem.
    if partial.parent.resolve() != final.parent.resolve():
        raise ValueError("Partial and final must share a session directory")
    final.hardlink_to(partial)
    partial.unlink()
    return result
