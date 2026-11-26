from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from flexiv_capture.config import load_config
from flexiv_capture.desktop import ManualSession, CollectionWindow
from flexiv_capture.recording import Recorder
from flexiv_capture.teleop_control import TDKRobotController
from flexiv_capture.validation import validate_episode
from fakes import SDK, RDK


class ManualTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / "config.tdk_force_only.yaml")
        self.output = ROOT / "offline_validation" / ("manual_" + uuid4().hex)
        self.sdk = SDK()
        def factory(config, output):
            controller = TDKRobotController(config["robot"], config["teleop"], rdk=RDK, tdk=self.sdk)
            return Recorder(config, output, controller=controller)
        self.session = ManualSession(self.config, self.output, recorder_factory=factory)

    def tearDown(self):
        self.session.stop()
        if self.session.thread: self.session.thread.join(5)

    def wait_recording(self):
        end = time.monotonic() + 3
        while time.monotonic() < end:
            if self.session.snapshot()["progress"].get("robot", {}).get("samples", 0) >= 5: return
            time.sleep(.01)
        self.fail(str(self.session.snapshot()))

    def test_manual_stop_is_only_normal_end_and_can_repeat(self):
        for _ in range(2):
            self.assertTrue(self.session.start())
            self.wait_recording()
            before = self.session.snapshot()["progress"]["robot"]["samples"]
            time.sleep(.15)
            self.assertTrue(self.session.thread.is_alive())
            self.assertGreater(self.session.snapshot()["progress"]["robot"]["samples"], before)
            self.session.stop()
            self.session.thread.join(3)
            snapshot = self.session.snapshot()
            self.assertEqual(snapshot["state"], "saved", snapshot)
            self.assertGreater(validate_episode(Path(snapshot["path"]))["samples_per_arm"], 5)
        self.assertEqual(len(list(self.output.glob("episode_*.hdf5"))), 2)

    def test_duplicate_start_rejected(self):
        self.session.start()
        self.assertFalse(self.session.start())
        self.wait_recording()
        self.assertEqual(len(self.sdk.connections), 1)

    def test_start_cancel_does_not_engage(self):
        gate = threading.Event()
        original = self.session.recorder_factory
        def delayed(*args):
            gate.wait(2)
            return original(*args)
        self.session.recorder_factory = delayed
        self.session.start()
        self.session.stop()
        gate.set()
        self.session.thread.join(3)
        self.assertEqual(self.session.snapshot()["state"], "idle")
        self.assertNotIn(("engage", True), self.sdk.log)

    def test_recording_fault_latches_and_does_not_publish(self):
        self.session.start()
        self.wait_recording()
        self.sdk.connections[-1].robots[1].bad = True
        self.session.thread.join(3)
        self.assertEqual(self.session.snapshot()["state"], "fault")
        self.assertFalse(self.session.start())
        self.assertFalse(any(p for p in self.output.glob("episode_*.hdf5") if '.partial.' not in p.name))
        self.assertTrue((self.output / 'session_errors.log').exists())

    def test_gui_buttons_stop_and_close_save(self):
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        window = CollectionWindow(root, self.session, execute=True)
        try:
            with patch.object(window.messagebox, 'askokcancel', return_value=True):
                window.start_button.invoke()
            self.wait_recording()
            root.update()
            window.stop_button.invoke()
            self.session.thread.join(3)
            self.assertEqual(self.session.snapshot()['state'], 'saved')
            root.update()
            with patch.object(window.messagebox, 'askokcancel', return_value=True):
                window.start()
            self.wait_recording()
            window.close()
            self.session.thread.join(3)
            self.assertEqual(self.session.snapshot()['state'], 'saved')
        finally:
            root.destroy()

    def test_offline_gui_never_creates_recorder(self):
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        try:
            window = CollectionWindow(root, self.session, execute=False)
            window.start_button.invoke()
            window.start()
            root.update()
            self.assertIsNone(self.session.thread)
            self.assertEqual(self.sdk.connections, [])
            self.assertEqual(str(window.start_button['state']), 'disabled')
        finally: root.destroy()


if __name__ == '__main__': unittest.main(verbosity=2)
