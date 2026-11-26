from __future__ import annotations

import json
import os
import time
import zlib
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import Any, Iterable

import h5py
import numpy as np

from . import RAW_FORMAT


def _create_resizable(
    group: h5py.Group,
    name: str,
    shape_tail: tuple[int, ...],
    dtype: Any,
    *,
    compression: str | None = None,
    compression_level: int = 4,
) -> h5py.Dataset:
    chunks = (1, *shape_tail) if shape_tail else (1024,)
    if not shape_tail:
        chunks = (1024,)
    return group.create_dataset(
        name,
        shape=(0, *shape_tail),
        maxshape=(None, *shape_tail),
        dtype=dtype,
        chunks=chunks,
        compression=compression,
        compression_opts=compression_level if compression == "gzip" else None,
        shuffle=compression is not None,
    )


def _append(dataset: h5py.Dataset, values: np.ndarray) -> None:
    values = np.asarray(values, dtype=dataset.dtype)
    if values.shape[0] == 0:
        return
    start = dataset.shape[0]
    dataset.resize(start + values.shape[0], axis=0)
    dataset[start:] = values


def _gzip_chunks(values: np.ndarray, level: int) -> list[bytes]:
    """Apply HDF5's byte shuffle and deflate filters outside its global lock."""
    chunks = []
    for frame in values:
        if frame.dtype.itemsize > 1:
            raw = frame.view(np.uint8).reshape(-1, frame.dtype.itemsize).T.copy().tobytes()
        else:
            raw = frame.tobytes()
        chunks.append(zlib.compress(raw, level=level))
    return chunks


class RawEpisodeWriter:

    def __init__(
        self,
        path: Path,
        *,
        joint_dof: int,
        image_shape: tuple[int, int] | None,
        compression: str | None,
        metadata: dict[str, Any],
        camera_metadata: dict[str, Any] | None,
        stream_metadata: dict[str, dict[str, Any]] | None = None,
        compression_level: int = 4,
        compression_workers: int = 4,
    ) -> None:
        if type(compression_level) is not int or not 0 <= compression_level <= 9:
            raise ValueError("compression_level must be an integer from 0 to 9")
        if type(compression_workers) is not int or compression_workers < 1:
            raise ValueError("compression_workers must be a positive integer")
        self.path = path
        if (image_shape is None) != (camera_metadata is None):
            raise ValueError("image_shape and camera_metadata must both be set or both be None")
        self.file = h5py.File(path, "x", libver="latest")
        self.file.attrs["format"] = RAW_FORMAT
        self.file.attrs["metadata_json"] = json.dumps(metadata, sort_keys=True)
        self.compression = compression
        self.compression_level = compression_level
        self.compression_workers = compression_workers
        self._pool: ThreadPoolExecutor | None = None
        self._pending: deque[
            tuple[h5py.Dataset, int, np.ndarray, Future[list[bytes]] | None]
        ] = deque()
        self._pending_bytes = 0
        self._max_pending_bytes = 256 * 1024**2
        self._peak_pending_bytes = 0
        self._peak_pending_batches = 0
        self._next_rows: dict[str, int] = {}
        self.stream_datasets: dict[str, dict[str, h5py.Dataset]] = {}
        for path, info in (stream_metadata or {}).items():
            group = self.file.require_group(path)
            group.attrs["metadata_json"] = json.dumps(info, sort_keys=True)
            self.stream_datasets[path] = {}

        robot = self.file.create_group("robot")

        self.robot_datasets = {
            "host_time_ns": _create_resizable(robot, "host_time_ns", (), np.int64),
            "robot_time_ns": _create_resizable(robot, "robot_time_ns", (), np.int64),
            "tcp_pose_wxyz": _create_resizable(robot, "tcp_pose_wxyz", (7,), np.float64),
            "tcp_vel": _create_resizable(robot, "tcp_vel", (6,), np.float64),
            "q": _create_resizable(robot, "q", (joint_dof,), np.float64),
            "dq": _create_resizable(robot, "dq", (joint_dof,), np.float64),
            "ext_wrench_in_tcp": _create_resizable(
                robot, "ext_wrench_in_tcp", (6,), np.float64
            ),
            "ext_wrench_in_tcp_raw": _create_resizable(
                robot, "ext_wrench_in_tcp_raw", (6,), np.float64
            ),
            "ft_sensor_raw": _create_resizable(robot, "ft_sensor_raw", (6,), np.float64),
        }

        self.color_datasets: dict[str, h5py.Dataset] = {}
        self.depth_datasets: dict[str, h5py.Dataset] = {}
        if image_shape is not None and camera_metadata is not None:
            camera = self.file.create_group("camera")
            camera.attrs["metadata_json"] = json.dumps(camera_metadata, sort_keys=True)
            height, width = image_shape
            color = camera.create_group("color")
            depth = camera.create_group("depth")
            self.color_datasets = {
                "host_time_ns": _create_resizable(color, "host_time_ns", (), np.int64),
                "device_time_ms": _create_resizable(
                    color, "device_time_ms", (), np.float64
                ),
                "frame_number": _create_resizable(color, "frame_number", (), np.int64),
                "image": _create_resizable(
                    color, "image", (height, width, 3), np.uint8,
                    compression=compression, compression_level=compression_level,
                ),
            }
            self.depth_datasets = {
                "host_time_ns": _create_resizable(depth, "host_time_ns", (), np.int64),
                "device_time_ms": _create_resizable(
                    depth, "device_time_ms", (), np.float64
                ),
                "frame_number": _create_resizable(depth, "frame_number", (), np.int64),
                "image": _create_resizable(
                    depth, "image", (height, width), np.uint16,
                    compression=compression, compression_level=compression_level,
                ),
            }
        if compression == "gzip":
            self._pool = ThreadPoolExecutor(
                max_workers=compression_workers, thread_name_prefix="capture-gzip"
            )

    def append_robot(self, samples: Iterable[dict[str, Any]]) -> None:
        self._append_samples(self.robot_datasets, samples)

    def _append_samples(
        self, datasets: dict[str, h5py.Dataset], samples: Iterable[dict[str, Any]]
    ) -> None:
        self._drain_pending()
        batch = list(samples)
        if not batch:
            return
        if self._pool is None:
            for name, dataset in datasets.items():
                _append(dataset, np.asarray([sample[name] for sample in batch]))
            return

        # Own the arrays: sensor SDKs may reuse their sample buffers immediately.
        entries = [
            (dataset, np.asarray([sample[name] for sample in batch], dtype=dataset.dtype))
            for name, dataset in datasets.items()
        ]
        needed = sum(values.nbytes for _, values in entries)
        if self._pending_bytes + needed > self._max_pending_bytes:
            raise RuntimeError("Parallel gzip write queue exceeded 256 MiB")
        for dataset, values in entries:
            values.flags.writeable = False
            start = self._next_rows.get(dataset.name, 0)
            future = None
            if dataset.compression == "gzip":
                future = self._pool.submit(_gzip_chunks, values, self.compression_level)
            self._pending.append((dataset, start, values, future))
            self._next_rows[dataset.name] = start + len(values)
            self._pending_bytes += values.nbytes
        self._peak_pending_bytes = max(self._peak_pending_bytes, self._pending_bytes)
        self._peak_pending_batches = max(self._peak_pending_batches, len(self._pending))
        self._drain_pending()

    def _drain_pending(self, *, wait: bool = False) -> None:
        # Only the recording thread accesses HDF5; workers return compressed bytes.
        while self._pending:
            dataset, start, values, future = self._pending[0]
            if future is not None and not wait and not future.done():
                break
            chunks = future.result() if future is not None else None
            if len(dataset) != start:
                raise RuntimeError(f"Out-of-order write to {dataset.name}")
            dataset.resize(start + len(values), axis=0)
            if chunks is None:
                dataset[start:] = values
            else:
                for index, chunk in enumerate(chunks, start):
                    dataset.id.write_direct_chunk(
                        (index, *([0] * (dataset.ndim - 1))), chunk, filter_mask=0
                    )
            self._pending_bytes -= values.nbytes
            self._pending.popleft()

    def append_color(self, samples: Iterable[dict[str, Any]]) -> None:
        self._append_samples(self.color_datasets, samples)

    def append_depth(self, samples: Iterable[dict[str, Any]]) -> None:
        self._append_samples(self.depth_datasets, samples)

    def flush(self) -> None:
        # Flush completed work without blocking sensor queues on all gzip jobs.
        self._drain_pending()
        self.file.flush()

    def append_stream(self, path: str, samples: Iterable[dict[str, Any]]) -> None:
        batch = list(samples)
        if not batch:
            return
        datasets = self.stream_datasets[path]
        if not datasets:
            for name, value in batch[0].items():
                array = np.asarray(value)
                datasets[name] = _create_resizable(
                    self.file[path],
                    name,
                    array.shape,
                    array.dtype,
                    compression=self.compression if array.ndim else None,
                    compression_level=self.compression_level,
                )
        self._append_samples(datasets, batch)

    def close(self, stats: dict[str, Any] | None = None) -> None:
        if not self.file:
            return
        parallel = self._pool is not None
        try:
            started = time.monotonic()
            self._drain_pending(wait=True)
            if parallel:
                self.file.attrs["compression_stats_json"] = json.dumps({
                    "compression": "gzip", "level": self.compression_level,
                    "workers": self.compression_workers,
                    "max_pending_bytes": self._max_pending_bytes,
                    "peak_pending_bytes": self._peak_pending_bytes,
                    "peak_pending_batches": self._peak_pending_batches,
                    "close_drain_s": time.monotonic() - started,
                }, sort_keys=True)
            if stats is not None:
                self.file.attrs["recording_stats_json"] = json.dumps(stats, sort_keys=True)
            self.file.flush()
        finally:
            try:
                self._stop_workers()
            finally:
                self.file.close()
        if parallel:
            with self.path.open("r+b") as completed:
                os.fsync(completed.fileno())

    def _stop_workers(self) -> None:
        if self._pool is not None:
            self._pool.shutdown(wait=True, cancel_futures=True)
            self._pool = None
        self._pending.clear()
        self._pending_bytes = 0

    def abort(self) -> None:
        try:
            self._stop_workers()
        finally:
            if self.file:
                try:
                    self.file.flush()
                finally:
                    self.file.close()
