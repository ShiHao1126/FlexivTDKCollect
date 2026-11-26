import io
import os
import signal
import subprocess
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import test_manual
ROOT = test_manual.ROOT
from flexiv_capture.terminal import run_terminal
from flexiv_capture.validation import validate_episode


class ScriptedInput:
    def __init__(self, lines): self.lines = iter(lines)
    def isatty(self): return True
    def readline(self):
        item = next(self.lines, '')
        return item() if callable(item) else item


class TerminalTests(unittest.TestCase):
    setUp = test_manual.ManualTests.setUp
    tearDown = test_manual.ManualTests.tearDown
    wait_recording = test_manual.ManualTests.wait_recording

    def wait_home_then_r(self):
        test_manual.ManualTests.wait_state(self,'ready')
        return 'r\n'

    def wait_teleop_then_c(self):
        test_manual.ManualTests.wait_state(self,'teleop')
        self.assertFalse(list(self.output.glob('*.hdf5')))
        return 'c\n'

    def wait_saved_then_quit(self):
        end = time.monotonic() + 3
        while time.monotonic() < end:
            if self.session.snapshot()['path'] and self.session.snapshot()['state'] == 'teleop': return 'q\n'
            time.sleep(.01)
        raise OSError(str(self.session.snapshot()))

    def test_commands_record_stop_save_and_quit(self):
        def stop_after_samples():
            self.wait_recording()
            return 's\n'
        stream = ScriptedInput(['h\n', self.wait_home_then_r, self.wait_teleop_then_c, stop_after_samples, self.wait_saved_then_quit])
        result = run_terminal(self.session, execute=True, stdin=stream, stdout=io.StringIO())
        self.assertEqual(result, 0)
        self.assertEqual(self.session.snapshot()['state'], 'closed')
        validate_episode(Path(self.session.snapshot()['path']))

    def test_quit_while_recording_saves(self):
        def quit_after_samples():
            self.wait_recording()
            return 'q\n'
        result = run_terminal(self.session, execute=True, stdin=ScriptedInput(['r\n', self.wait_teleop_then_c, quit_after_samples]), stdout=io.StringIO())
        self.assertEqual(result, 0)
        self.assertEqual(self.session.snapshot()['state'], 'closed')

    def test_eof_while_recording_does_not_publish(self):
        def disconnect():
            self.wait_recording()
            return ''
        result = run_terminal(self.session, execute=True, stdin=ScriptedInput(['r\n', self.wait_teleop_then_c, disconnect]), stdout=io.StringIO())
        self.assertEqual(result, 1)
        self.assertEqual(self.session.snapshot()['state'], 'fault')
        self.assertEqual(self.session.snapshot()['path'], '')
        self.assertTrue(list(self.output.glob('*.partial.hdf5')))
        with self.assertRaises(ValueError):
            validate_episode(next(self.output.glob('*.partial.hdf5')))

    def test_offline_start_is_disabled(self):
        result = run_terminal(self.session, stdin=ScriptedInput(['h\n', 'r\n', 'c\n', 'q\n']), stdout=io.StringIO())
        self.assertEqual(result, 0)
        self.assertIsNone(self.session.thread)

    def test_hardware_redirected_input_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'interactive terminal'):
            run_terminal(self.session, execute=True, stdin=io.StringIO('r\nRUN\n'))
        self.assertIsNone(self.session.thread)

    def test_old_confirmation_words_are_not_commands(self):
        stream = ScriptedInput(['HOME\n', 'RUN\n', 'q\n'])
        run_terminal(self.session, execute=True, stdin=stream, stdout=io.StringIO())
        self.assertIsNone(self.session.thread)

    @unittest.skipUnless(sys.platform == 'linux', 'POSIX signal subprocess test')
    def test_real_sigterm_and_sighup_stop_fake_recording(self):
        code = '''import sys,time
from pathlib import Path
sys.path[:0]=['.', 'scripts/tests']
from fakes import SDK,RDK
from flexiv_capture.config import load_config
from flexiv_capture.recording import Recorder
from flexiv_capture.teleop_control import TDKRobotController
from flexiv_capture.manual_session import ManualSession
from flexiv_capture.terminal import run_terminal
c=load_config('config.tdk_force_only.yaml'); sdk=SDK()
def factory(c,p): return Recorder(c,p,controller=TDKRobotController(c['robot'],c['teleop'],rdk=RDK,tdk=sdk))
s=ManualSession(c,Path(sys.argv[1]),recorder_factory=factory)
s.engage()
while s.snapshot()['state']!='teleop': time.sleep(.01)
s.start()
while s.snapshot()['progress'].get('robot',{}).get('samples',0)<5: time.sleep(.01)
raise SystemExit(run_terminal(s,execute=False))
'''
        for sig in (signal.SIGTERM, signal.SIGHUP):
            output = self.output / str(sig)
            process = subprocess.Popen([sys.executable, '-u', '-c', code, str(output)], cwd=ROOT,
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                self.assertIn('TDK force collection', process.stdout.readline())
                process.send_signal(sig)
                stdout, stderr = process.communicate(timeout=10)
                self.assertEqual(process.returncode, 1, stdout+stderr)
                self.assertTrue(list(output.glob('*.partial.hdf5')))
                self.assertFalse([p for p in output.glob('*.hdf5') if '.partial.' not in p.name])
            finally:
                if process.poll() is None: process.kill(); process.wait()


if __name__ == '__main__': unittest.main(verbosity=2)
