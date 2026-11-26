from pathlib import Path
import sys
import os
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
        self.session.shutdown()
        if self.session.thread: self.session.thread.join(5)

    def wait_state(self, state):
        end = time.monotonic() + 3
        while time.monotonic() < end:
            if self.session.snapshot()['state'] == state: return
            time.sleep(.01)
        self.fail(str(self.session.snapshot()))

    def enable_teleop(self):
        self.assertTrue(self.session.engage())
        self.wait_state('teleop')

    def wait_recording(self):
        end = time.monotonic() + 3
        while time.monotonic() < end:
            if self.session.snapshot()["progress"].get("robot", {}).get("samples", 0) >= 5: return
            time.sleep(.01)
        self.fail(str(self.session.snapshot()))

    def test_home_moves_both_to_their_own_targets_and_holds(self):
        self.config['episode_reset']['leader_home_q_deg'] = [0, -30, 0, 90, 0, 40, 0]
        self.assertTrue(self.session.home())
        self.wait_state('ready')
        moves = [x for x in self.sdk.log if len(x)>1 and x[1] == 'move']
        self.assertEqual(len(moves), 2)
        import numpy as np
        for move, role in zip(moves, ('leader','follower')):
            np.testing.assert_allclose(move[2], np.deg2rad(self.config['episode_reset'][role+'_home_q_deg']))
        self.assertNotIn(('engage',True), self.sdk.log)
        self.assertFalse(list(self.output.glob('*.hdf5')))
        self.enable_teleop()

    def test_r_does_not_record_and_s_keeps_teleop_for_next_episode(self):
        self.enable_teleop()
        self.assertFalse(list(self.output.glob('*.hdf5')))
        for _ in range(2):
            self.assertTrue(self.session.start())
            self.wait_recording()
            before = self.session.snapshot()["progress"]["robot"]["samples"]
            time.sleep(.15)
            self.assertGreater(self.session.snapshot()["progress"]["robot"]["samples"], before)
            self.session.stop()
            self.wait_state('teleop')
            snapshot = self.session.snapshot()
            self.assertGreater(validate_episode(Path(snapshot["path"]))["samples_per_arm"], 5)
            self.assertNotIn(('tdk_stop',),self.sdk.log)
        self.assertEqual(self.sdk.log.count(('engage', True)),1)
        self.assertEqual(len(list(self.output.glob("episode_*.hdf5"))), 2)
        self.session.shutdown()
        self.session.thread.join(3)
        self.assertIn(('tdk_stop',),self.sdk.log)
        self.assertEqual(self.session.snapshot()['state'], 'closed')

    def test_invalid_and_duplicate_commands_rejected(self):
        self.assertFalse(self.session.start())
        self.assertIsNone(self.session.thread)
        self.enable_teleop()
        self.assertFalse(self.session.engage())
        self.session.start()
        self.assertFalse(self.session.start())
        self.assertFalse(self.session.home())
        self.wait_recording()
        self.assertEqual(len(self.sdk.connections), 1)

    def test_home_from_teleop_stops_both_before_moving(self):
        self.enable_teleop()
        self.session.home()
        self.wait_state('ready')
        first_stop=self.sdk.log.index(('tdk_stop',))
        moves=[i for i,x in enumerate(self.sdk.log) if len(x)>1 and x[1]=='move']
        self.assertEqual(len(moves),2)
        self.assertTrue(all(first_stop<i for i in moves))
        self.assertEqual(self.sdk.log.count(('engage',True)),1)

    def test_start_cancel_does_not_engage(self):
        gate = threading.Event()
        original = self.session.recorder_factory
        def delayed(*args):
            gate.wait(2)
            return original(*args)
        self.session.recorder_factory = delayed
        self.session.engage()
        self.session.shutdown()
        gate.set()
        self.session.thread.join(3)
        self.assertEqual(self.session.snapshot()["state"], "closed")
        self.assertNotIn(("engage", True), self.sdk.log)

    def test_recording_fault_latches_and_does_not_publish(self):
        self.enable_teleop()
        self.session.start()
        self.wait_recording()
        self.sdk.connections[-1].robots[1].bad = True
        self.session.thread.join(3)
        self.assertEqual(self.session.snapshot()["state"], "fault")
        self.assertFalse(self.session.start())
        self.assertFalse(self.session.engage())
        self.assertFalse(any(p for p in self.output.glob("episode_*.hdf5") if '.partial.' not in p.name))

    def test_stale_state_while_not_recording_stops_teleop(self):
        self.enable_teleop()
        for robot in self.sdk.connections[-1].robots: robot.freeze=True
        self.session.thread.join(3)
        self.assertEqual(self.session.snapshot()['state'],'fault')
        self.assertIn(('tdk_stop',),self.sdk.log)

    @unittest.skipIf(sys.platform == 'linux' and not os.environ.get('DISPLAY'), 'GUI requires a display; terminal tests still run')
    def test_gui_separate_home_teleop_record_and_close(self):
        import tkinter as tk
        root = tk.Tk(); root.withdraw()
        window = CollectionWindow(root, self.session, execute=True)
        try:
            with patch.object(window.messagebox,'askokcancel',side_effect=AssertionError('Unexpected confirmation')):
                window.home()
                self.wait_state('ready')
                window.engage()
                self.wait_state('teleop')
            window.start()
            self.wait_recording()
            deadline=time.monotonic()+2
            while str(window.stop_button['state'])!='normal' and time.monotonic()<deadline:
                root.update();time.sleep(.01)
            window.stop_button.invoke()
            self.wait_state('teleop')
            self.assertTrue(self.session.snapshot()['path'])
            self.assertEqual(self.sdk.log.count(('engage',True)),1)
            window.close()
            self.session.thread.join(3)
            self.assertEqual(self.session.snapshot()['state'],'closed')
        finally:
            for callback in root.tk.call('after','info'): root.after_cancel(callback)
            root.destroy()
            window = root = None
            import gc
            gc.collect()

    @unittest.skipIf(sys.platform == 'linux' and not os.environ.get('DISPLAY'), 'GUI requires a display; terminal tests still run')
    def test_offline_gui_never_creates_recorder(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw()
        try:
            window=CollectionWindow(root,self.session,execute=False)
            window.home();window.engage();window.start();root.update()
            self.assertIsNone(self.session.thread)
            for button in (window.home_button,window.engage_button,window.start_button):
                self.assertEqual(str(button['state']),'disabled')
        finally:
            for callback in root.tk.call('after','info'): root.after_cancel(callback)
            root.destroy()
            window = root = None
            import gc
            gc.collect()


if __name__ == '__main__': unittest.main(verbosity=2)
