import queue
import threading
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


class RejectingPipeline(FakePipeline):
    def start(self, config):
        raise RuntimeError("Couldn't resolve requests")


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
    def __init__(self, serial):
        self.serial = serial
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
    def __init__(self, mac):
        self.mac = mac
        self.released = False

    def get_gripper_status(self):
        return {"position": 50.0}

    def release(self):
        self.released = True


OUTPUTS = SimpleNamespace(
    Marker2D=1,
    ForceResultant=2,
    Rectify=3,
    Difference=4,
    Depth=5,
    TimeStamp=6,
)


class WorkerTests(unittest.TestCase):
    def test_camera_saves_raw_color_and_depth_without_alignment(self):
        pipeline = FakePipeline()
        rs = SimpleNamespace(
            pipeline=lambda: pipeline,
            config=FakeConfig,
            stream=SimpleNamespace(color=1, depth=2),
            format=SimpleNamespace(bgr8=1, z16=2),
        )
        config = CollectConfig(
            Path("unused"),
            enable_robot=False,
            enable_gripper=False,
            enable_tactile=False,
            camera_width=848,
            camera_height=480,
            camera_fps=30,
        )
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = CameraWorker(
                config,
                Path(tmp),
                time.monotonic_ns(),
                queue.Queue(),
                rs_module=rs,
            )
            worker.start()
            time.sleep(0.03)
            worker.stop()
            worker.join(2.0)
            self.assertTrue(pipeline.stopped)
            streams = FakeConfig.instances[-1].streams
            self.assertEqual(len(streams), 2)
            self.assertEqual(streams[0][1:4], (848, 480, rs.format.bgr8))
            self.assertEqual(streams[0][4], 30)
            self.assertEqual(streams[1][1:4], (848, 480, rs.format.z16))
            self.assertEqual(streams[1][4], 30)
            saved = len(list((Path(tmp) / "camera/auto_0/color").glob("*.png")))
            self.assertEqual(saved + worker.stats["dropped_frames"], worker.stats["samples"])
            self.assertTrue((Path(tmp) / "camera/auto_0/depth").is_dir())

    def test_camera_color_only_does_not_request_depth(self):
        pipeline = FakePipeline()
        rs = SimpleNamespace(
            pipeline=lambda: pipeline,
            config=FakeConfig,
            stream=SimpleNamespace(color=1, depth=2),
            format=SimpleNamespace(bgr8=1, z16=2),
        )
        config = CollectConfig(
            Path("unused"),
            enable_robot=False,
            enable_gripper=False,
            enable_tactile=False,
            camera_serials=("D405_SN",),
            collect_depth=False,
        )
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = CameraWorker(
                config,
                Path(tmp),
                time.monotonic_ns(),
                queue.Queue(),
                camera_serial="D405_SN",
                rs_module=rs,
            )
            worker.start()
            time.sleep(0.02)
            worker.stop()
            worker.join(2.0)
            active = FakeConfig.instances[-1]
            self.assertEqual(active.serial, "D405_SN")
            self.assertEqual(len(active.streams), 1)
            self.assertEqual(active.streams[0][0], rs.stream.color)
            self.assertFalse((Path(tmp) / "camera/D405_SN/depth").exists())

    def test_camera_reports_serial_and_requested_profile_when_start_fails(self):
        pipeline = RejectingPipeline()
        rs = SimpleNamespace(
            pipeline=lambda: pipeline,
            config=FakeConfig,
            stream=SimpleNamespace(color=1, depth=2),
            format=SimpleNamespace(bgr8=1, z16=2),
        )
        config = CollectConfig(
            Path("unused"),
            enable_robot=False,
            enable_gripper=False,
            enable_tactile=False,
            camera_serials=("camera_sn",),
            camera_width=848,
            camera_height=480,
            camera_fps=30,
        )
        errors = queue.Queue()
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = CameraWorker(
                config,
                Path(tmp),
                time.monotonic_ns(),
                errors,
                camera_serial="camera_sn",
                rs_module=rs,
            )
            worker.start()
            worker.join(2.0)

        name, message, traceback_text = errors.get_nowait()
        self.assertEqual(name, "camera:camera_sn")
        self.assertIn("RealSense camera_sn", message)
        self.assertIn("color and depth", message)
        self.assertIn("848x480 at 30 FPS", message)
        self.assertIn("Couldn't resolve requests", message)
        self.assertIn("The above exception was the direct cause", traceback_text)
        self.assertFalse(pipeline.stopped)

    def test_xense_uses_mac_and_serial_lists_and_saves_full_tactile_contract(self):
        grippers = []
        sensors = []

        def gripper_factory(*, mac_addr):
            gripper = FakeGripper(mac_addr)
            grippers.append(gripper)
            return gripper

        def sensor_factory(serial, *, mac_addr):
            self.assertEqual(mac_addr, "route_mac")
            sensor = FakeSensor(serial)
            sensors.append(sensor)
            return sensor

        config = CollectConfig(
            Path("unused"),
            enable_robot=False,
            enable_camera=False,
            gripper_macs=("route_mac",),
            tactile_sns=("sensor_a", "sensor_b"),
            xense_rate_hz=100.0,
        )
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = XenseWorker(
                config,
                Path(tmp),
                time.monotonic_ns(),
                queue.Queue(),
                gripper_factory=gripper_factory,
                sensor_factory=sensor_factory,
                output_types=OUTPUTS,
            )
            worker.prepare()
            worker.start()
            time.sleep(0.03)
            worker.stop()
            worker.join(2.0)
            worker.close()
            self.assertEqual([gripper.mac for gripper in grippers], ["route_mac"])
            self.assertEqual([sensor.serial for sensor in sensors], ["sensor_a", "sensor_b"])
            self.assertTrue(all(gripper.released for gripper in grippers))
            self.assertTrue(all(sensor.released for sensor in sensors))
            expected = (
                OUTPUTS.Marker2D,
                OUTPUTS.ForceResultant,
                OUTPUTS.Rectify,
                OUTPUTS.Difference,
                OUTPUTS.Depth,
                OUTPUTS.TimeStamp,
            )
            self.assertTrue(all(sensor.requested_outputs[0] == expected for sensor in sensors))
            self.assertTrue((Path(tmp) / "gripper/route_mac/width_m.npy").is_file())
            for serial in ("sensor_a", "sensor_b"):
                self.assertTrue((Path(tmp) / f"tactile/{serial}/marker2d.npy").is_file())
                self.assertTrue((Path(tmp) / f"tactile/{serial}/force_torque.npy").is_file())
                for image_name in ("rectify", "difference", "depth"):
                    self.assertTrue((Path(tmp) / f"tactile/{serial}/{image_name}").is_dir())

    def test_xense_supports_multiple_grippers_when_tactile_is_disabled(self):
        grippers = []

        def gripper_factory(*, mac_addr):
            gripper = FakeGripper(mac_addr)
            grippers.append(gripper)
            return gripper

        config = CollectConfig(
            Path("unused"),
            enable_robot=False,
            enable_camera=False,
            gripper_macs=("mac_a", "mac_b"),
            enable_tactile=False,
            xense_rate_hz=100.0,
        )
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = XenseWorker(
                config,
                Path(tmp),
                time.monotonic_ns(),
                queue.Queue(),
                gripper_factory=gripper_factory,
                sensor_factory=lambda *_args, **_kwargs: self.fail("no tactile sensor"),
            )
            worker.prepare()
            worker.start()
            time.sleep(0.02)
            worker.stop()
            worker.join(2.0)
            worker.close()
            self.assertEqual([gripper.mac for gripper in grippers], ["mac_a", "mac_b"])
            self.assertTrue(all(gripper.released for gripper in grippers))
            self.assertTrue((Path(tmp) / "gripper/mac_a/width_m.npy").is_file())
            self.assertTrue((Path(tmp) / "gripper/mac_b/width_m.npy").is_file())
            self.assertFalse((Path(tmp) / "tactile").exists())

    def test_xense_devices_are_prepared_on_calling_thread(self):
        factory_threads = []
        caller_thread = threading.get_ident()

        def gripper_factory(*, mac_addr):
            factory_threads.append(threading.get_ident())
            return FakeGripper(mac_addr)

        def sensor_factory(serial, *, mac_addr):
            factory_threads.append(threading.get_ident())
            return FakeSensor(serial)

        config = CollectConfig(
            Path("unused"),
            enable_robot=False,
            enable_camera=False,
            gripper_macs=("route_mac",),
            tactile_sns=("sensor_a",),
        )
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            worker = XenseWorker(
                config,
                Path(tmp),
                time.monotonic_ns(),
                queue.Queue(),
                gripper_factory=gripper_factory,
                sensor_factory=sensor_factory,
                output_types=OUTPUTS,
            )
            worker.prepare()
            worker.start()
            time.sleep(0.02)
            worker.stop()
            worker.join(2.0)
            worker.close()

        self.assertEqual(factory_threads, [caller_thread, caller_thread])

    def test_xense_partial_startup_releases_acquired_resources(self):
        gripper = FakeGripper("route_mac")
        first_sensor = FakeSensor("sensor_a")
        calls = 0

        def sensor_factory(serial, *, mac_addr):
            nonlocal calls
            calls += 1
            if calls == 1:
                return first_sensor
            raise RuntimeError("second sensor unavailable")

        config = CollectConfig(
            Path("unused"),
            enable_robot=False,
            enable_camera=False,
            gripper_macs=("route_mac",),
            tactile_sns=("sensor_a", "sensor_b"),
        )
        with tempfile.TemporaryDirectory(dir="/tmp") as tmp:
            errors = queue.Queue()
            worker = XenseWorker(
                config,
                Path(tmp),
                time.monotonic_ns(),
                errors,
                gripper_factory=lambda **_kwargs: gripper,
                sensor_factory=sensor_factory,
                output_types=OUTPUTS,
            )
            with self.assertRaisesRegex(RuntimeError, "second sensor unavailable"):
                worker.prepare()
            self.assertTrue(first_sensor.released)
            self.assertTrue(gripper.released)


if __name__ == "__main__":
    unittest.main()
