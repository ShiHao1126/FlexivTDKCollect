import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "home_robot.py"
SPEC = importlib.util.spec_from_file_location("home_robot", SCRIPT_PATH)
home_robot = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(home_robot)


def make_args(**overrides):
    values = {
        "robot_sn": "Rizon4s-123456",
        "tool_name": "test_tool",
        "target_q": [0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "joint_max_velocity": 5.0,
        "joint_max_acceleration": 4.0,
        "joint_tolerance": 0.5,
        "operational_timeout": 1.0,
        "motion_timeout": 2.0,
        "zero_force_timeout": 1.0,
        "poll_interval": 0.01,
        "clear_fault": False,
        "yes": True,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class FakeRobot:
    def __init__(self, fail_motion=False):
        self.fail_motion = fail_motion
        self.calls = []

    def prepare(self, **kwargs):
        self.calls.append(("prepare", kwargs))

    def move_joints(self, target, **kwargs):
        self.calls.append(("move_joints", np.asarray(target), kwargs))
        if self.fail_motion:
            raise RuntimeError("motion failed")

    def close(self):
        self.calls.append(("close",))


class HomeRobotTest(unittest.TestCase):
    def test_defaults_are_ready_for_direct_run(self):
        args = home_robot.HomeArgs().parse_args([])
        self.assertEqual(args.robot_sn, "Rizon4s-063652")
        self.assertEqual(args.tool_name, "tool1")
        self.assertEqual(args.target_q, [0, -40, 0, 90, 0, 40, 45])
        self.assertEqual(args.joint_max_velocity, 5.0)
        self.assertEqual(args.joint_max_acceleration, 5.0)
        self.assertEqual(args.joint_tolerance, 0.5)
        self.assertTrue(args.clear_fault)
        self.assertTrue(args.yes)

    def test_rejects_invalid_target(self):
        with self.assertRaisesRegex(ValueError, "exactly 7"):
            home_robot.validate_args(make_args(target_q=[0.0] * 6))

    def test_delegates_homing_with_converted_joint_units(self):
        robot = FakeRobot()
        serials = []

        def factory(serial):
            serials.append(serial)
            return robot

        result = home_robot.run_homing(make_args(), robot_factory=factory)
        self.assertEqual(result, 0)
        self.assertEqual(serials, ["Rizon4s-123456"])
        self.assertEqual(robot.calls[0][0], "prepare")
        self.assertEqual(robot.calls[0][1]["tool_name"], "test_tool")
        self.assertEqual(robot.calls[0][1]["zero_force_timeout_s"], 1.0)
        move = robot.calls[1]
        np.testing.assert_allclose(move[1], np.deg2rad(make_args().target_q))
        self.assertAlmostEqual(move[2]["max_velocity_rad_s"], math.radians(5.0))
        self.assertAlmostEqual(move[2]["max_acceleration_rad_s2"], math.radians(4.0))
        self.assertAlmostEqual(move[2]["tolerance_rad"], math.radians(0.5))
        self.assertEqual(robot.calls[-1], ("close",))

    def test_motion_error_still_closes_robot(self):
        robot = FakeRobot(fail_motion=True)
        with self.assertRaisesRegex(RuntimeError, "motion failed"):
            home_robot.run_homing(make_args(), robot_factory=lambda _serial: robot)
        self.assertEqual(robot.calls[-1], ("close",))

    def test_operator_can_cancel_before_connecting(self):
        connected = False

        def factory(_serial):
            nonlocal connected
            connected = True

        result = home_robot.run_homing(
            make_args(yes=False), input_fn=lambda _prompt: "n", robot_factory=factory
        )
        self.assertEqual(result, 2)
        self.assertFalse(connected)


if __name__ == "__main__":
    unittest.main()
