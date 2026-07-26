import time
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace
from uuid import uuid4

import numpy as np

from flexiv_tdk_collect.config import CollectConfig
from flexiv_tdk_collect.session import CollectionSession


class FakeTeleop:
    def robot_states(self, index):
        state = SimpleNamespace(timestamp=(12, 500_000_000), **{
            name: np.zeros(7 if name in {"q", "dq", "tau", "tau_ext", "tau_interact", "tcp_pose"} else 6)
            for name in ("q", "dq", "tau", "tau_ext", "tau_interact", "tcp_pose", "tcp_vel", "ext_wrench_in_tcp", "ext_wrench_in_world")
        })
        return object(), state


class FailingTeleop:
    def robot_states(self, index):
        raise RuntimeError("robot state unavailable")


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.output = Path("/tmp/flexiv_tdk_collect_tests") / uuid4().hex

    def test_robot_only_record_is_finalized(self):
        config = CollectConfig(self.output, "leader", enable_camera_color=False, enable_camera_depth=False,
                               enable_gripper_width=False, enable_tactile_marker=False,
                               enable_tactile_force=False, enable_tactile_rectify=False,
                               enable_tactile_difference=False, enable_tactile_depth=False)
        session = CollectionSession(FakeTeleop(), config)
        session.start()
        time.sleep(0.03)
        result = session.finish()
        self.assertTrue(result.name.startswith("record_"))
        self.assertGreater(np.load(result / "robot/q.npy").shape[0], 0)
        self.assertEqual(np.load(result / "robot/timestamp_sdk_s.npy")[0], 12.5)
        self.assertFalse((result / "robot/temperature.npy").exists())
        self.assertFalse((result / "robot/ext_wrench_in_tcp_raw.npy").exists())
        self.assertTrue((result / "validation.json").is_file())

    def test_interrupted_record_is_invalid(self):
        session = CollectionSession(FakeTeleop(), self._robot_only_config())
        session.start()
        time.sleep(0.02)
        result = session.finish("interrupted")
        self.assertTrue(result.name.startswith("invalid_record_"))

    def test_worker_failure_is_finalized_as_invalid(self):
        session = CollectionSession(FailingTeleop(), self._robot_only_config())
        session.start()
        time.sleep(0.02)
        result = session.finish()
        self.assertTrue(result.name.startswith("invalid_record_"))
        validation = (result / "validation.json").read_text(encoding="utf-8")
        self.assertIn("robot state unavailable", validation)

    def test_partial_thread_start_is_finalized_as_invalid(self):
        class FailingCameraWorker:
            def __init__(self, *args, **kwargs):
                self.name = "camera"

            def start(self):
                raise RuntimeError("camera thread could not start")

        config = self._robot_only_config()
        object.__setattr__(config, "enable_camera_color", True)
        session = CollectionSession(FakeTeleop(), config)
        with patch("flexiv_tdk_collect.session.CameraWorker", FailingCameraWorker):
            with self.assertRaisesRegex(RuntimeError, "invalid record saved"):
                session.start()
        self.assertFalse(session.recording)
        invalid_records = list(self.output.glob("invalid_record_*"))
        self.assertEqual(len(invalid_records), 1)
        self.assertIn("camera thread could not start", (invalid_records[0] / "validation.json").read_text())

    def test_each_camera_serial_creates_an_independent_worker(self):
        created = []

        class FakeCameraWorker:
            def __init__(self, config, *args, camera_index, camera_serial):
                self.name = f"camera:{camera_serial}"
                self.camera_index = camera_index
                self.camera_serial = camera_serial
                self.stats = {"samples": 1, "write_errors": 0, "cleanup_errors": []}
                created.append(self)

            def start(self):
                pass

            def stop(self):
                pass

            def join(self, timeout=None):
                pass

            def is_alive(self):
                return False

        config = CollectConfig(
            self.output, "leader", enable_robot=False, camera_serials=("123", "456"),
            enable_gripper_width=False, enable_tactile_marker=False,
            enable_tactile_force=False, enable_tactile_rectify=False,
            enable_tactile_difference=False, enable_tactile_depth=False,
        )
        with patch("flexiv_tdk_collect.session.CameraWorker", FakeCameraWorker):
            session = CollectionSession(FakeTeleop(), config)
            session.start()
            self.assertEqual(
                [(worker.camera_index, worker.camera_serial) for worker in created],
                [(0, "123"), (1, "456")],
            )
            result = session.finish()
        self.assertTrue(result.name.startswith("record_"))

    def test_duplicate_camera_serials_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "camera serials must be unique"):
            CollectConfig(self.output, "leader", camera_serials=("123", "123"))

    def _robot_only_config(self):
        return CollectConfig(
            self.output, "leader", enable_camera_color=False, enable_camera_depth=False,
            enable_gripper_width=False, enable_tactile_marker=False,
            enable_tactile_force=False, enable_tactile_rectify=False,
            enable_tactile_difference=False, enable_tactile_depth=False,
        )


if __name__ == "__main__":
    unittest.main()
