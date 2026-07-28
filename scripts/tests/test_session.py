import json
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from uuid import uuid4

import numpy as np

from flexiv_tdk_collect.config import ROBOT_FIELDS, CollectConfig
from flexiv_tdk_collect.robot import TdkFlexivStateSource
from flexiv_tdk_collect.session import CollectionSession


class FakeTeleop:
    def robot_states(self, index):
        state = SimpleNamespace(timestamp=(12, 500_000_000), **{
            name: np.zeros(7 if name in {"q", "dq", "tau", "tau_ext", "tau_interact", "tcp_pose"} else 6)
            for name in ROBOT_FIELDS
        })
        return object(), state


class FailingTeleop:
    def robot_states(self, index):
        raise RuntimeError("robot state unavailable")


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.output = Path("/tmp/flexiv_tdk_collect_tests") / uuid4().hex

    def test_robot_only_record_saves_fixed_state_contract(self):
        session = CollectionSession(
            TdkFlexivStateSource(FakeTeleop()), self._robot_only_config()
        )
        session.start()
        time.sleep(0.03)
        result = session.finish()
        self.assertTrue(result.name.startswith("record_"))
        for field in ROBOT_FIELDS:
            self.assertGreater(np.load(result / f"robot/{field}.npy").shape[0], 0)
        self.assertEqual(np.load(result / "robot/timestamp_sdk_s.npy")[0], 12.5)
        self.assertFalse((result / "robot/temperature.npy").exists())
        metadata = json.loads((result / "metadata.json").read_text())
        self.assertEqual(metadata["acquisition"]["robot"]["fields"], list(ROBOT_FIELDS))
        self.assertEqual(metadata["acquisition"]["robot"]["leader_tool"], "gripper")
        self.assertEqual(metadata["acquisition"]["robot"]["follower_tool"], "xense")

    def test_interrupted_record_is_invalid(self):
        session = CollectionSession(TdkFlexivStateSource(FakeTeleop()), self._robot_only_config())
        session.start()
        time.sleep(0.02)
        result = session.finish("interrupted")
        self.assertTrue(result.name.startswith("invalid_record_"))

    def test_worker_failure_is_finalized_as_invalid(self):
        session = CollectionSession(
            TdkFlexivStateSource(FailingTeleop()), self._robot_only_config()
        )
        session.start()
        time.sleep(0.02)
        result = session.finish()
        self.assertTrue(result.name.startswith("invalid_record_"))
        self.assertIn("robot state unavailable", (result / "validation.json").read_text())

    def test_each_camera_serial_creates_an_independent_worker(self):
        created = []

        class FakeCameraWorker:
            def __init__(self, config, *args, camera_index, camera_serial):
                self.name = f"camera:{camera_serial}"
                self.camera_index = camera_index
                self.camera_serial = camera_serial
                self.stats = {"samples": 1, "write_errors": 0, "cleanup_errors": []}
                created.append(self)

            def prepare(self):
                pass

            def close(self):
                pass

            def start(self):
                pass

            def stop(self):
                pass

            def join(self, timeout=None):
                pass

            def is_alive(self):
                return False

        config = CollectConfig(
            self.output,
            enable_robot=False,
            enable_gripper=False,
            enable_tactile=False,
            camera_serials=("D405_SN", "D415_SN"),
        )
        with patch("flexiv_tdk_collect.session.CameraWorker", FakeCameraWorker):
            session = CollectionSession(None, config)
            session.start()
            self.assertEqual(
                [(worker.camera_index, worker.camera_serial) for worker in created],
                [(0, "D405_SN"), (1, "D415_SN")],
            )
            result = session.finish()
        self.assertTrue(result.name.startswith("record_"))

    def test_duplicate_device_identifiers_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "camera serials must be unique"):
            CollectConfig(self.output, camera_serials=("123", "123"))
        with self.assertRaisesRegex(ValueError, "gripper MACs must be unique"):
            CollectConfig(self.output, gripper_macs=("abc", "abc"))
        with self.assertRaisesRegex(ValueError, "tactile serials must be unique"):
            CollectConfig(self.output, tactile_sns=("one", "one"))

    def test_tactile_requires_unambiguous_gripper_route(self):
        with self.assertRaisesRegex(ValueError, "exactly one gripper MAC"):
            CollectConfig(
                self.output,
                enable_gripper=False,
                gripper_macs=("first", "second"),
                enable_tactile=True,
            )

    def _robot_only_config(self):
        return CollectConfig(
            self.output,
            enable_camera=False,
            enable_gripper=False,
            enable_tactile=False,
        )


if __name__ == "__main__":
    unittest.main()
