from __future__ import annotations

import queue
import threading
import time
import traceback
from pathlib import Path
from typing import Any

import numpy as np

from .config import ROBOT_FIELDS, CollectConfig
from .robot import FlexivStateSource
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
    def __init__(self, state_source: FlexivStateSource, config: CollectConfig, *args: Any):
        super().__init__("robot", *args)
        self.state_source = state_source
        self.config = config

    def collect(self) -> None:
        values = {
            name: [] for name in ("timestamp_s", "timestamp_sdk_s", *ROBOT_FIELDS)
        }
        period = 1.0 / self.config.robot_rate_hz
        deadline = time.monotonic()
        try:
            while not self.stop_event.is_set():
                state = self.state_source.read_state()
                values["timestamp_s"].append(self.timestamp_s())
                values["timestamp_sdk_s"].append(state.timestamp_sdk_s)
                for name in ROBOT_FIELDS:
                    values[name].append(getattr(state, name))
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
        cfg.enable_stream(
            rs.stream.color,
            self.config.camera_width,
            self.config.camera_height,
            rs.format.bgr8,
            self.config.camera_fps,
        )
        if self.config.collect_depth:
            cfg.enable_stream(
                rs.stream.depth,
                self.config.camera_width,
                self.config.camera_height,
                rs.format.z16,
                self.config.camera_fps,
            )

        timestamps, device_timestamps, frame_indices = [], [], []
        pipeline_started = False
        writer = None
        index = 0
        try:
            try:
                profile = pipeline.start(cfg)
            except RuntimeError as exc:
                serial = self.camera_serial or "auto-selected device"
                streams = "color and depth" if self.config.collect_depth else "color"
                raise RuntimeError(
                    f"RealSense {serial} does not support the requested {streams} "
                    f"profile: {self.config.camera_width}x{self.config.camera_height} "
                    f"at {self.config.camera_fps} FPS ({exc})"
                ) from exc
            pipeline_started = True
            if profile is not None and hasattr(profile, "get_device"):
                device = profile.get_device()
                self.stats["device_name"] = device.get_info(rs.camera_info.name)
                self.stats["device_serial"] = device.get_info(rs.camera_info.serial_number)
            writer = ImageWriter(self.config.image_queue_size, self.stats)
            writer.start()
            while not self.stop_event.is_set():
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                color = frames.get_color_frame()
                depth = frames.get_depth_frame() if self.config.collect_depth else None
                if not color or (self.config.collect_depth and not depth):
                    continue
                images = [(
                    self.camera_dir / "color",
                    index,
                    np.asanyarray(color.get_data()).copy(),
                )]
                if depth:
                    images.append((
                        self.camera_dir / "depth",
                        index,
                        np.asanyarray(depth.get_data()).copy(),
                    ))
                if writer.submit(images):
                    timestamps.append(self.timestamp_s())
                    device_timestamps.append(float(color.get_timestamp()))
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
        if self.config.enable_gripper and gripper_factory is None:
            from xensegripper import XenseGripper
            gripper_factory = XenseGripper.create
        if self.config.enable_tactile and (sensor_factory is None or output_types is None):
            from xensesdk import Sensor
            sensor_factory = Sensor.create
            output_types = Sensor.OutputType

        output_specs = (
            ("marker2d", output_types.Marker2D, False),
            ("force_torque", output_types.ForceResultant, False),
            ("rectify", output_types.Rectify, True),
            ("difference", output_types.Difference, True),
            ("depth", output_types.Depth, True),
        ) if self.config.enable_tactile else ()
        grippers: dict[str, Any] = {}
        sensors: dict[str, Any] = {}
        gripper_values = {
            mac: {"timestamp_s": [], "width_m": []}
            for mac in self.config.gripper_macs
        }
        tactile_values = {
            serial: {
                "timestamp_s": [],
                "timestamp_sdk_s": [],
                "marker2d": [],
                "force_torque": [],
            }
            for serial in self.config.tactile_sns
        }
        period = 1.0 / self.config.xense_rate_hz
        deadline = time.monotonic()
        index = 0
        writer = None
        try:
            if self.config.enable_gripper:
                for mac in self.config.gripper_macs:
                    grippers[mac] = gripper_factory(mac_addr=mac)
            if self.config.enable_tactile:
                mac = self.config.gripper_macs[0]
                for serial in self.config.tactile_sns:
                    sensors[serial] = sensor_factory(serial, mac_addr=mac)
                writer = ImageWriter(self.config.image_queue_size, self.stats)
                writer.start()

            outputs = [output for _, output, _ in output_specs]
            while not self.stop_event.is_set():
                for mac, gripper in grippers.items():
                    status = gripper.get_gripper_status()
                    gripper_values[mac]["timestamp_s"].append(self.timestamp_s())
                    gripper_values[mac]["width_m"].append(
                        float(status["position"]) / 1000.0
                    )
                image_batch = []
                for serial, sensor in sensors.items():
                    values = sensor.selectSensorInfo(*outputs, output_types.TimeStamp)
                    if not isinstance(values, (tuple, list)) or len(values) != len(output_specs) + 1:
                        raise RuntimeError(f"Xense tactile {serial} output count mismatch")
                    tactile_values[serial]["timestamp_s"].append(self.timestamp_s())
                    tactile_values[serial]["timestamp_sdk_s"].append(float(values[-1]))
                    for (name, _, is_image), value in zip(output_specs, values[:-1]):
                        if is_image:
                            image_batch.append((
                                self.record_dir / "tactile" / serial / name,
                                index,
                                np.asarray(value).copy(),
                            ))
                        else:
                            dtype = np.float32 if name == "marker2d" else np.float64
                            array = np.asarray(value, dtype=dtype)
                            tactile_values[serial][name].append(
                                array.reshape(6) if name == "force_torque" else array
                            )
                if writer is not None:
                    writer.submit(image_batch)
                index += 1
                deadline += period
                self.stop_event.wait(max(0.0, deadline - time.monotonic()))
        finally:
            if writer is not None:
                writer.close()
            for sensor in reversed(tuple(sensors.values())):
                try:
                    sensor.release()
                except Exception as exc:
                    self.stats["cleanup_errors"].append(str(exc))
            for gripper in reversed(tuple(grippers.values())):
                release = getattr(gripper, "release", None)
                if callable(release):
                    try:
                        release()
                    except Exception as exc:
                        self.stats["cleanup_errors"].append(str(exc))
            self.stats["samples"] = index
            for mac, values in gripper_values.items():
                if self.config.enable_gripper:
                    save_arrays(self.record_dir / "gripper" / mac, values)
            for serial, values in tactile_values.items():
                if self.config.enable_tactile:
                    save_arrays(self.record_dir / "tactile" / serial, values)
