import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "zero_force_torque.py"
SPEC = importlib.util.spec_from_file_location("zero_force_torque", SCRIPT_PATH)
zero_force_torque = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(zero_force_torque)


def make_args(**overrides):
    values = {
        "robot_sn": "Rizon4s-123456",
        "tool_name": "test_tool",
        "operational_timeout": 1.0,
        "zero_force_timeout": 2.0,
        "poll_interval": 0.01,
        "clear_fault": True,
        "yes": True,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class FakeRobot:
    def __init__(self):
        self.calls = []

    def prepare(self, **kwargs):
        self.calls.append(("prepare", kwargs))

    def close(self):
        self.calls.append(("close",))


class ZeroForceTorqueTest(unittest.TestCase):
    def test_defaults_are_ready_for_direct_run(self):
        args = zero_force_torque.ZeroForceTorqueArgs().parse_args([])
        self.assertEqual(args.robot_sn, "Rizon4s-063586")
        self.assertEqual(args.tool_name, "hapticexoteleop")
        self.assertTrue(args.clear_fault)
        self.assertTrue(args.yes)

    def test_prepares_robot_with_tool_and_zeroing_timeout(self):
        robot = FakeRobot()
        result = zero_force_torque.run_zeroing(
            make_args(), robot_factory=lambda _serial: robot
        )
        self.assertEqual(result, 0)
        self.assertEqual(
            robot.calls,
            [
                ("prepare", {
                    "clear_fault": True,
                    "tool_name": "test_tool",
                    "operational_timeout_s": 1.0,
                    "poll_interval_s": 0.01,
                    "zero_force_timeout_s": 2.0,
                }),
                ("close",),
            ],
        )

    def test_operator_can_cancel_before_connecting(self):
        connected = False

        def factory(_serial):
            nonlocal connected
            connected = True

        result = zero_force_torque.run_zeroing(
            make_args(yes=False),
            input_fn=lambda _prompt: "n",
            robot_factory=factory,
        )
        self.assertEqual(result, 2)
        self.assertFalse(connected)


if __name__ == "__main__":
    unittest.main()
