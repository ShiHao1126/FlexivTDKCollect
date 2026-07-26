import queue
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from flexiv_tdk_collect.config import CollectConfig
from flexiv_tdk_collect.workers import CameraWorker, XenseWorker


class FakePipeline:
    def __init__(self):
        self.stopped = False

    def start(self, config):
        return None

    def wait_for_frames(self, timeout_ms):
        image = np.zeros((4, 6, 3), dtype=np.uint8)
        depth = np.zeros((4, 6), dtype=np.uint16)
        return FakeFrames(FakeFrame(image), FakeFrame(depth))

    def stop(self):
        self.stopped = True


class FakeFrame:
    def __init__(self, data):
        self.data = data

    def __bool__(self):
        return True

    def get_data(self):
        return self.data

    def get_timestamp(self):
        return 123.0


class FakeFrames:
    def __init__(self, color, depth):
        self.color = color
        self.depth = depth

    def get_color_frame(self):
        return self.color

    def get_depth_frame(self):
        return self.depth


class FakeConfig:
    instances = []

    def __init__(self):
        self.streams = []
        self.serial = None
        self.__class__.instances.append(self)

    def enable_device(self, serial):
        self.serial = serial

    def enable_stream(self, *args):
        self.streams.append(args)


class FakeSensor:
    def __init__(self):
        self.released = False
        self.requested_outputs = []

    def selectSensorInfo(self, *outputs):
        self.requested_outputs.append(outputs)
        values = {
            1: np.zeros((2, 2, 2), dtype=np.float32),
            2: np.arange(6, dtype=np.float64),
            3: np.zeros((4, 6), dtype=np.uint8),
            4: np.zeros((4, 6), dtype=np.uint8),
            5: np.zeros((4, 6), dtype=np.uint16),
            6: 42.0,
        }
        return tuple(values[output] for output in outputs)

    def release(self):
        self.released = True


class FakeGripper:
    def __init__(self):
        self.released = False

    def get_gripper_status(self):
        return {"position": 50.0, "velocity": 2.0, "force": 3.0, "temperature": 30.0}

    def release(self):
        self.released = True


class WorkerTests(unittest.TestCase):
    def test_camera_stops_pipeline_and_saves_aligned_frames(self):
        pipeline = FakePipeline()
        rs = SimpleNamespace(
            pipeline=lambda: pipeline,
            config=FakeConfig,
            align=lambda stream: SimpleNamespace(process=lambda frames: frames),
            stream=SimpleNamespace(color=1, depth=2),
            format=SimpleNamespace(bgr8=1, z16=2),
        )
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = CameraWorker(CollectConfig(Path(tmp), "leader"), Path(tmp), time.monotonic_ns(), queue.Queue(), rs_module=rs)
            worker.start()
            time.sleep(0.03)
            worker.stop()
            worker.join(2.0)
            self.assertTrue(pipeline.stopped)
            self.assertGreater(worker.stats["samples"], 0)
            saved = len(list((Path(tmp) / "camera/auto_0/color").glob("*.png")))
            self.assertEqual(saved + worker.stats["dropped_frames"], worker.stats["samples"])
            self.assertEqual(np.load(Path(tmp) / "camera/auto_0/frame_index.npy").shape, (saved,))

    def test_camera_color_only_enables_only_color_profile(self):
        pipeline = FakePipeline()
        rs = SimpleNamespace(
            pipeline=lambda: pipeline,
            config=FakeConfig,
            align=lambda stream: self.fail("alignment must not be created for color-only capture"),
            stream=SimpleNamespace(color=1, depth=2),
            format=SimpleNamespace(bgr8=1, z16=2),
        )
        config = CollectConfig(Path("unused"), "leader", camera_serials=("abc",), enable_camera_depth=False)
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = CameraWorker(
                config, Path(tmp), time.monotonic_ns(), queue.Queue(),
                camera_serial="abc", rs_module=rs,
            )
            worker.start()
            time.sleep(0.02)
            worker.stop()
            worker.join(2.0)
            self.assertEqual(FakeConfig.instances[-1].serial, "abc")
            self.assertEqual(len(FakeConfig.instances[-1].streams), 1)
            self.assertEqual(FakeConfig.instances[-1].streams[0][0], rs.stream.color)
            self.assertFalse((Path(tmp) / "camera/abc/depth").exists())

    def test_xense_releases_all_devices_and_saves_both_sides(self):
        gripper = FakeGripper()
        sensors = [FakeSensor(), FakeSensor()]
        sensor_iter = iter(sensors)
        outputs = SimpleNamespace(Marker2D=1, ForceResultant=2, Rectify=3, Difference=4, Depth=5, TimeStamp=6)
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = XenseWorker(
                CollectConfig(Path(tmp), "leader", xense_rate_hz=100.0),
                Path(tmp), time.monotonic_ns(), queue.Queue(),
                gripper_factory=lambda **kwargs: gripper,
                sensor_factory=lambda *args, **kwargs: next(sensor_iter),
                output_types=outputs,
            )
            worker.start()
            time.sleep(0.03)
            worker.stop()
            worker.join(2.0)
            self.assertTrue(gripper.released)
            self.assertTrue(all(sensor.released for sensor in sensors))
            for side in ("left", "right"):
                self.assertGreater(np.load(Path(tmp) / f"gripper/tactile/{side}/marker2d.npy").shape[0], 0)

    def test_xense_partial_startup_releases_acquired_resources(self):
        gripper = FakeGripper()
        left = FakeSensor()
        calls = 0

        def sensor_factory(*args, **kwargs):
            nonlocal calls
            calls += 1
            if calls == 1:
                return left
            raise RuntimeError("right sensor unavailable")

        outputs = SimpleNamespace(Marker2D=1, ForceResultant=2, Rectify=3, Difference=4, Depth=5, TimeStamp=6)
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            errors = queue.Queue()
            worker = XenseWorker(
                CollectConfig(Path(tmp), "leader"), Path(tmp), time.monotonic_ns(), errors,
                gripper_factory=lambda **kwargs: gripper,
                sensor_factory=sensor_factory,
                output_types=outputs,
            )
            worker.start()
            worker.join(2.0)
            self.assertTrue(left.released)
            self.assertTrue(gripper.released)
            self.assertFalse(errors.empty())

    def test_xense_width_only_does_not_open_tactile_sensors(self):
        gripper = FakeGripper()
        config = CollectConfig(
            Path("unused"), "leader", xense_rate_hz=100.0,
            enable_tactile_marker=False, enable_tactile_force=False,
            enable_tactile_rectify=False, enable_tactile_difference=False,
            enable_tactile_depth=False,
        )
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = XenseWorker(
                config, Path(tmp), time.monotonic_ns(), queue.Queue(),
                gripper_factory=lambda **kwargs: gripper,
                sensor_factory=lambda *args, **kwargs: self.fail("tactile sensor must stay closed"),
            )
            worker.start()
            time.sleep(0.02)
            worker.stop()
            worker.join(2.0)
            self.assertTrue(gripper.released)
            self.assertTrue((Path(tmp) / "gripper/width_m.npy").exists())
            self.assertFalse((Path(tmp) / "gripper/tactile").exists())

    def test_xense_requests_only_enabled_tactile_outputs(self):
        sensors = [FakeSensor(), FakeSensor()]
        sensor_iter = iter(sensors)
        outputs = SimpleNamespace(Marker2D=1, ForceResultant=2, Rectify=3, Difference=4, Depth=5, TimeStamp=6)
        config = CollectConfig(
            Path("unused"), "leader", xense_rate_hz=100.0, enable_gripper_width=False,
            enable_tactile_marker=False, enable_tactile_rectify=False,
            enable_tactile_difference=False, enable_tactile_depth=False,
        )
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = XenseWorker(
                config, Path(tmp), time.monotonic_ns(), queue.Queue(),
                gripper_factory=lambda **kwargs: self.fail("gripper must stay closed"),
                sensor_factory=lambda *args, **kwargs: next(sensor_iter),
                output_types=outputs,
            )
            worker.start()
            time.sleep(0.02)
            worker.stop()
            worker.join(2.0)
            self.assertTrue(all(sensor.requested_outputs[0] == (outputs.ForceResultant, outputs.TimeStamp)
                                for sensor in sensors))
            self.assertTrue((Path(tmp) / "gripper/tactile/left/force_torque.npy").exists())
            self.assertFalse((Path(tmp) / "gripper/tactile/left/marker2d.npy").exists())


if __name__ == "__main__":
    unittest.main()
