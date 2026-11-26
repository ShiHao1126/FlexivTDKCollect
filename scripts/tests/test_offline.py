from pathlib import Path
import copy
import json
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch
from uuid import uuid4

import h5py
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from flexiv_capture.config import load_config, validate_force_only_config
from flexiv_capture.robot_control import robot_state_sample
from flexiv_capture.teleop_control import TDKRobotController
from flexiv_capture.recording import Recorder, finalize
from flexiv_capture.hdf5_io import RawEpisodeWriter
from flexiv_capture.validation import validate_episode
from fakes import SDK, RDK

ARTIFACTS = ROOT / "offline_validation" / (time.strftime("%Y%m%d_%H%M%S") + "_" + uuid4().hex[:8])


class OfflineTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / "config.tdk_force_only.yaml")
        self.sdk = SDK()
        self.controller = TDKRobotController(self.config["robot"], self.config["teleop"], rdk=RDK, tdk=self.sdk)
        self.output = ARTIFACTS / self._testMethodName
        self.recorder = Recorder(self.config, self.output, controller=self.controller)

    def tearDown(self):
        self.recorder.close()

    def capture(self, seconds=0.16):
        return self.recorder.record_episode(seconds)

    def test_default_cli_never_imports_sdk(self):
        code = '''import runpy, sys
class Block:
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in {'flexivrdk', 'flexivtdk', 'pyrealsense2', 'xensesdk'}:
            raise AssertionError('Hardware import attempted: ' + fullname)
sys.meta_path.insert(0, Block())
sys.argv = ['collect.py', '--check_only']
runpy.run_path('scripts/collect.py', run_name='__main__')
'''
        result = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Offline configuration check passed", result.stdout)

    def test_scope_rejects_peripherals(self):
        for section in ("camera", "wrist_camera", "gripper", "gripper_teleop"):
            config = copy.deepcopy(self.config)
            config[section]["enabled"] = True
            with self.subTest(section=section), self.assertRaises(ValueError):
                validate_force_only_config(config)
        for side in ("left", "right"):
            config = copy.deepcopy(self.config)
            config["tactile"][side]["enabled"] = True
            with self.assertRaises(ValueError): validate_force_only_config(config)

    def test_scope_rejects_missing_camera_flag(self):
        del self.config["camera"]["enabled"]
        with self.assertRaises(ValueError): validate_force_only_config(self.config)

    def test_scope_rejects_cartesian_home(self):
        self.config["episode_reset"]["follower_cartesian_home"] = {}
        with self.assertRaises(ValueError): validate_force_only_config(self.config)

    def test_state_copy_and_units(self):
        state = self.controller.leader.states()
        sample = robot_state_sample(state)
        state.q[:] = 1
        np.testing.assert_array_equal(sample["q"], np.zeros(7))
        np.testing.assert_array_equal(sample["ext_wrench_in_tcp"], np.arange(6) + 1)
        self.assertEqual(sample["robot_time_ns"], 1101000000)

    def test_bad_state_rejected(self):
        for name, value in (("ft_sensor_raw", [1]*5), ("tcp_pose", [0]*7),
                            ("dq", [float('nan')]*7), ("timestamp", [1, -1])):
            state = self.controller.leader.states()
            setattr(state, name, value)
            with self.subTest(name=name), self.assertRaises(RuntimeError): robot_state_sample(state)

    def test_tdk_parameters_and_reconnect(self):
        self.controller.engage(True)
        self.assertIn(("feedback", .75), self.sdk.log)
        self.assertIn(("stiffness", [1000.]*6), self.sdk.log)
        self.assertIn(("wrench_limit", [25.]*3+[40.]*3), self.sdk.log)
        self.controller.stop()
        self.controller.engage(True)
        self.assertEqual([item for item in self.sdk.log if item[0] == 'init'], [("init", "zero"), ("init", "keep")])
        self.assertEqual(len(self.sdk.connections), 2)

    def test_failed_init_stops(self):
        self.sdk.fail_init = True
        with self.assertRaises(RuntimeError): self.controller.engage(True)
        self.assertIn(("tdk_stop",), self.sdk.log)

    def test_failed_home_init_stops_immediately(self):
        self.sdk.fail_init = True
        with self.assertRaises(RuntimeError): self.controller.reset_home(self.config["episode_reset"])
        self.assertIn(("tdk_stop",), self.sdk.log)

    def test_duplicate_backward_stale_states(self):
        self.controller.read_pair()
        for robot in self.controller.robots: robot.freeze = True
        self.assertIsNone(self.controller.read_pair())
        for index, (stamp, _) in self.controller._last_stamp.items():
            self.controller._last_stamp[index] = (stamp, time.monotonic() - 1)
        with self.assertRaisesRegex(RuntimeError, "stopped updating"): self.controller.read_pair()
        self.controller.leader.tick -= 2
        with self.assertRaisesRegex(RuntimeError, "backwards"): self.controller.read_pair()

    def test_fault_rejected(self):
        self.controller.follower.bad = True
        with self.assertRaisesRegex(RuntimeError, "fault=true"): self.controller.read_pair()

    def test_unexpected_disconnect_is_not_retried(self):
        self.controller.leader.valid = False
        with self.assertRaisesRegex(RuntimeError, "connection lost"): self.controller.read_pair()
        self.assertEqual(len(self.sdk.connections), 1)

    def test_home_outside_limits_never_moves(self):
        self.config["episode_reset"]["leader_home_q_deg"] = [300]*7
        with self.assertRaisesRegex(ValueError, "outside joint limits"):
            self.controller.reset_home(self.config["episode_reset"])
        self.assertFalse(any(len(entry)>1 and entry[1] == "move" for entry in self.sdk.log))

    def test_home_timeout_stops_both(self):
        self.config["episode_reset"]["motion_timeout_s"] = .001
        with self.assertRaisesRegex(TimeoutError, "paired home"):
            self.controller.reset_home(self.config["episode_reset"])
        self.assertEqual([item[1] for item in self.sdk.log[-2:]], ["robot_stop", "robot_stop"])

    def test_joint_home_and_record(self):
        reset = self.recorder.reset_paired_home()
        self.assertEqual(reset["post_reset_state"], "hold")
        moves = [entry for entry in self.sdk.log if len(entry)>1 and entry[1] == "move"]
        self.assertEqual(len(moves), 2)
        np.testing.assert_allclose(moves[0][3], [.01]*7)
        partial, final, _ = self.capture()
        finalize(partial, final)

    def test_bilateral_roundtrip_and_second_episode(self):
        for _ in range(2):
            partial, final, stats = self.capture()
            result = finalize(partial, final)
            self.assertGreater(result["samples_per_arm"], 1)
            self.assertFalse(partial.exists())
            with h5py.File(final, "r") as file:
                self.assertEqual(set(file), {"leader", "robot"})
                for role, offset in (("leader", 1), ("robot", 101)):
                    for key, extra in (("ext_wrench_in_tcp", 0), ("ext_wrench_in_tcp_raw", 10), ("ft_sensor_raw", 20)):
                        np.testing.assert_array_equal(file[role][key][0], np.arange(6) + offset + extra)
                    self.assertEqual(len(file[role]["q"]), stats[role]["samples"])

    def test_non_gzip_roundtrip(self):
        self.config["recording"]["compression"] = None
        partial, final, _ = self.capture()
        finalize(partial, final)

    def test_corrupt_force_rejected(self):
        partial, final, _ = self.capture()
        with h5py.File(partial, "r+") as file: file["leader/ft_sensor_raw"][0, 0] = float("nan")
        with self.assertRaises(ValueError): finalize(partial, final)
        self.assertFalse(final.exists())

    def test_corrupt_timestamp_rejected(self):
        partial, final, _ = self.capture()
        with h5py.File(partial, "r+") as file:
            file["robot/robot_time_ns"][1] = file["robot/robot_time_ns"][0]
        with self.assertRaisesRegex(ValueError, "timestamps"): finalize(partial, final)

    def test_unequal_stream_lengths_rejected(self):
        partial, final, _ = self.capture()
        with h5py.File(partial, "r+") as file:
            for dataset in file["leader"].values(): dataset.resize(len(dataset)-1, axis=0)
        with self.assertRaisesRegex(ValueError, "counts differ"): finalize(partial, final)

    def test_no_overwrite(self):
        partial, final, _ = self.capture()
        final.write_bytes(b"existing")
        with self.assertRaises(FileExistsError): finalize(partial, final)
        self.assertEqual(final.read_bytes(), b"existing")
        self.assertTrue(partial.exists())

    def test_queue_overflow_aborts(self):
        self.config["recording"].update(robot_queue_size=1, flush_interval_s=.1)
        with self.assertRaisesRegex(RuntimeError, "queue is full"): self.capture(.3)
        partial = next(self.output.glob("*.partial.hdf5"))
        with self.assertRaises(ValueError): validate_episode(partial)
        with self.assertRaisesRegex(RuntimeError, "faulted"): self.capture()

    def test_writer_failure_aborts(self):
        with patch.object(RawEpisodeWriter, "append_robot", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"): self.capture()
        self.assertIn(("tdk_stop",), self.sdk.log)
        partial = next(self.output.glob("*.partial.hdf5"))
        with self.assertRaises(ValueError): validate_episode(partial)

    def test_fsync_failure_marks_closed_file_failed(self):
        with patch("flexiv_capture.hdf5_io.os.fsync", side_effect=OSError("fsync failure")):
            with self.assertRaisesRegex(OSError, "fsync failure"): self.capture()
        partial = next(self.output.glob("*.partial.hdf5"))
        with self.assertRaises(ValueError): validate_episode(partial)

    def test_keyboard_interrupt_stops_and_retains_partial(self):
        with patch.object(RawEpisodeWriter, "flush", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt): self.capture()
        self.assertIn(("tdk_stop",), self.sdk.log)
        partial = next(self.output.glob("*.partial.hdf5"))
        with self.assertRaises(ValueError): validate_episode(partial)

    def test_empty_record_aborts(self):
        with patch.object(self.controller, "read_pair", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "no complete"): self.capture(.05)

    def test_external_stop(self):
        stop = threading.Event()
        timer = threading.Timer(.12, stop.set)
        timer.start()
        try:
            partial, final, _ = self.recorder.record_episode(None, stop_event=stop)
        finally: timer.join()
        finalize(partial, final)

    def test_pair_skew_aborts(self):
        original = self.controller.read_pair
        def skewed():
            pair = original()
            if pair: pair[1]["host_time_ns"] += 100_000_000
            return pair
        with patch.object(self.controller, "read_pair", side_effect=skewed):
            with self.assertRaisesRegex(RuntimeError, "skew"): self.capture()


if __name__ == "__main__":
    unittest.main(verbosity=2)
