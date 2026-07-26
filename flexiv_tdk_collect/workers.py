from __future__ import annotations

import queue
import threading
import time
import traceback
from pathlib import Path
from typing import Any

import numpy as np

from .config import CollectConfig
from .storage import save_arrays, save_image


class Worker(threading.Thread):
    def __init__(self, name: str, record_dir: Path, start_ns: int, errors: queue.Queue):
        super().__init__(name=name, daemon=True)
        self.record_dir = record_dir
        self.start_ns = start_ns
        self.errors = errors
        self.stop_event = threading.Event()
        self.stats: dict[str, Any] = {
            "samples": 0,
            "write_errors": 0,
            "dropped_frames": 0,
            "cleanup_errors": [],
        }

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        try:
            self.collect()
        except Exception as exc:
            self.errors.put((self.name, str(exc), traceback.format_exc()))
            self.stop_event.set()

    def collect(self) -> None:
        raise NotImplementedError

    def timestamp_s(self) -> float:
        return (time.monotonic_ns() - self.start_ns) * 1e-9


class ImageWriter(threading.Thread):
    def __init__(self, maxsize: int, stats: dict[str, Any]):
        super().__init__(name="image-writer", daemon=True)
        self.items: queue.Queue = queue.Queue(maxsize=maxsize)
        self.stats = stats
        self.closing = threading.Event()

    def submit(self, images: list[tuple[Path, int, Any]]) -> bool:
        try:
            self.items.put_nowait(images)
            return True
        except queue.Full:
            self.stats["dropped_frames"] += 1
            return False

    def close(self) -> None:
        self.closing.set()
        self.join(timeout=60.0)
        if self.is_alive():
            self.stats["cleanup_errors"].append("image writer shutdown timeout")

    def run(self) -> None:
        while not self.closing.is_set() or not self.items.empty():
            try:
                images = self.items.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                for directory, index, image in images:
                    save_image(directory, index, image)
            except Exception:
                self.stats["write_errors"] += 1
            finally:
                self.items.task_done()


class RobotWorker(Worker):
    def __init__(self, teleop: Any, config: CollectConfig, *args: Any):
        super().__init__("robot", *args)
        self.teleop = teleop
        self.config = config

    def collect(self) -> None:
        fields = self.config.robot_fields
        values = {name: [] for name in ("timestamp_s", "timestamp_sdk_s", *fields)}
        period = 1.0 / self.config.robot_rate_hz
        deadline = time.monotonic()
        try:
            while not self.stop_event.is_set():
                pair = self.teleop.robot_states(0)
                if not isinstance(pair, tuple) or len(pair) != 2:
                    raise RuntimeError("TDK robot_states(0) must return (leader, follower)")
                state = pair[1]
                values["timestamp_s"].append(self.timestamp_s())
                values["timestamp_sdk_s"].append(_sdk_timestamp_s(state.timestamp))
                for name in fields:
                    values[name].append(np.asarray(getattr(state, name)).copy())
                deadline += period
                self.stop_event.wait(max(0.0, deadline - time.monotonic()))
        finally:
            self.stats["samples"] = len(values["timestamp_s"])
            save_arrays(self.record_dir / "robot", values)


class CameraWorker(Worker):
    def __init__(
        self,
        config: CollectConfig,
        *args: Any,
        camera_index: int = 0,
        camera_serial: str | None = None,
        rs_module: Any = None,
    ):
        label = camera_serial or f"auto_{camera_index}"
        super().__init__(f"camera:{label}", *args)
        self.config = config
        self.camera_index = camera_index
        self.camera_serial = camera_serial
        self.camera_dir = self.record_dir / "camera" / label
        self._rs = rs_module

    def collect(self) -> None:
        if self._rs is None:
            import pyrealsense2 as rs
        else:
            rs = self._rs

        pipeline = rs.pipeline()
        cfg = rs.config()
        if self.camera_serial:
            cfg.enable_device(self.camera_serial)
        if self.config.enable_camera_color:
            cfg.enable_stream(
                rs.stream.color,
                self.config.camera_color_width,
                self.config.camera_color_height,
                rs.format.bgr8,
                self.config.camera_color_fps,
            )
        if self.config.enable_camera_depth:
            cfg.enable_stream(
                rs.stream.depth,
                self.config.camera_depth_width,
                self.config.camera_depth_height,
                rs.format.z16,
                self.config.camera_depth_fps,
            )
        timestamps, device_timestamps, frame_indices = [], [], []
        pipeline_started = False
        writer = None
        index = 0
        try:
            profile = pipeline.start(cfg)
            pipeline_started = True
            if profile is not None and hasattr(profile, "get_device"):
                device = profile.get_device()
                self.stats["device_name"] = device.get_info(rs.camera_info.name)
                self.stats["device_serial"] = device.get_info(rs.camera_info.serial_number)
            align = None
            if (self.config.enable_camera_color and self.config.enable_camera_depth
                    and self.config.align_depth_to_color):
                align = rs.align(rs.stream.color)
            writer = ImageWriter(self.config.image_queue_size, self.stats)
            writer.start()
            while not self.stop_event.is_set():
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                if align is not None:
                    frames = align.process(frames)
                color = frames.get_color_frame() if self.config.enable_camera_color else None
                depth = frames.get_depth_frame() if self.config.enable_camera_depth else None
                if (self.config.enable_camera_color and not color) or (self.config.enable_camera_depth and not depth):
                    continue
                images = []
                if color:
                    images.append((self.camera_dir / "color", index, np.asanyarray(color.get_data()).copy()))
                if depth:
                    images.append((self.camera_dir / "depth", index, np.asanyarray(depth.get_data()).copy()))
                accepted = writer.submit(images)
                if accepted:
                    timestamps.append(self.timestamp_s())
                    timestamp_frame = color if color else depth
                    device_timestamps.append(float(timestamp_frame.get_timestamp()))
                    frame_indices.append(index)
                index += 1
        finally:
            if pipeline_started:
                pipeline.stop()
            if writer is not None:
                writer.close()
            self.stats["samples"] = index
            save_arrays(self.camera_dir, {
                "timestamp_s": timestamps,
                "timestamp_device_ms": device_timestamps,
                "frame_index": frame_indices,
            })


class XenseWorker(Worker):
    def __init__(
        self,
        config: CollectConfig,
        *args: Any,
        gripper_factory: Any = None,
        sensor_factory: Any = None,
        output_types: Any = None,
    ):
        super().__init__("xense", *args)
        self.config = config
        self._gripper_factory = gripper_factory
        self._sensor_factory = sensor_factory
        self._output_types = output_types

    def collect(self) -> None:
        gripper_factory = self._gripper_factory
        sensor_factory = self._sensor_factory
        output_types = self._output_types
        if self.config.enable_gripper_width and gripper_factory is None:
            from xensegripper import XenseGripper
            gripper_factory = XenseGripper.create
        if self.config.enable_tactile and (sensor_factory is None or output_types is None):
            from xensesdk import Sensor
            sensor_factory = Sensor.create
            output_types = Sensor.OutputType
        if self.config.enable_tactile and output_types is None:
            raise RuntimeError("Xense output types are required")

        output_specs = []
        if self.config.enable_tactile_marker:
            output_specs.append(("marker2d", output_types.Marker2D, False))
        if self.config.enable_tactile_force:
            output_specs.append(("force_torque", output_types.ForceResultant, False))
        if self.config.enable_tactile_rectify:
            output_specs.append(("rectify", output_types.Rectify, True))
        if self.config.enable_tactile_difference:
            output_specs.append(("difference", output_types.Difference, True))
        if self.config.enable_tactile_depth:
            output_specs.append(("depth", output_types.Depth, True))

        gripper = None
        left = None
        right = None
        gripper_values = {name: [] for name in ("timestamp_s", "width_m")}
        array_names = [name for name, _, is_image in output_specs if not is_image]
        tactile = {
            side: {name: [] for name in ("timestamp_s", "timestamp_sdk_s", *array_names)}
            for side in ("left", "right")
        }
        period, deadline, index = 1.0 / self.config.xense_rate_hz, time.monotonic(), 0
        writer = None
        try:
            if self.config.enable_gripper_width:
                gripper = gripper_factory(mac_addr=self.config.gripper_mac)
            if self.config.enable_tactile:
                left = sensor_factory(self.config.left_tactile_sn, mac_addr=self.config.gripper_mac)
                right = sensor_factory(self.config.right_tactile_sn, mac_addr=self.config.gripper_mac)
            if any(is_image for _, _, is_image in output_specs):
                writer = ImageWriter(self.config.image_queue_size, self.stats)
                writer.start()
            while not self.stop_event.is_set():
                if gripper is not None:
                    status = gripper.get_gripper_status()
                    gripper_values["timestamp_s"].append(self.timestamp_s())
                    gripper_values["width_m"].append(float(status["position"]) / 1000.0)
                image_batch = []
                for side, sensor in (("left", left), ("right", right)):
                    if sensor is None:
                        continue
                    outputs = [output for _, output, _ in output_specs]
                    values = sensor.selectSensorInfo(*outputs, output_types.TimeStamp)
                    if not isinstance(values, (tuple, list)) or len(values) != len(output_specs) + 1:
                        raise RuntimeError(f"{side} Xense tactile output count mismatch")
                    tactile[side]["timestamp_s"].append(self.timestamp_s())
                    tactile[side]["timestamp_sdk_s"].append(float(values[-1]))
                    for (name, _, is_image), value in zip(output_specs, values[:-1]):
                        if is_image:
                            image_batch.append((
                                self.record_dir / f"gripper/tactile/{side}/{name}",
                                index,
                                np.asarray(value).copy(),
                            ))
                        else:
                            dtype = np.float32 if name == "marker2d" else np.float64
                            array = np.asarray(value, dtype=dtype)
                            tactile[side][name].append(array.reshape(6) if name == "force_torque" else array)
                if writer is not None:
                    writer.submit(image_batch)
                index += 1
                deadline += period
                self.stop_event.wait(max(0.0, deadline - time.monotonic()))
        finally:
            if writer is not None:
                writer.close()
            for sensor in (right, left):
                if sensor is None:
                    continue
                try:
                    sensor.release()
                except Exception as exc:
                    self.stats["cleanup_errors"].append(str(exc))
            release = getattr(gripper, "release", None) if gripper is not None else None
            if callable(release):
                try:
                    release()
                except Exception as exc:
                    self.stats["cleanup_errors"].append(str(exc))
            self.stats["samples"] = index
            if self.config.enable_gripper_width:
                save_arrays(self.record_dir / "gripper", gripper_values)
            if self.config.enable_tactile:
                for side in ("left", "right"):
                    save_arrays(self.record_dir / f"gripper/tactile/{side}", tactile[side])


def _sdk_timestamp_s(value: Any) -> float:
    if isinstance(value, (tuple, list)) and len(value) == 2:
        return float(value[0]) + float(value[1]) * 1e-9
    return float(value)
