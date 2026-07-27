from types import SimpleNamespace
import unittest

import numpy as np

from flexiv_tdk_collect.robot import RdkFlexivRobot, TdkFlexivStateSource


def make_state(q=None):
    return SimpleNamespace(
        timestamp=(12, 500_000_000),
        q=np.asarray(q if q is not None else np.zeros(7), dtype=np.float64),
        dq=np.ones(7),
        tau=np.ones(7) * 2,
        tau_ext=np.ones(7) * 3,
        tau_interact=np.ones(7) * 4,
        tcp_pose=np.arange(7),
        tcp_vel=np.arange(6),
        ext_wrench_in_tcp=np.arange(6) + 10,
        ext_wrench_in_world=np.arange(6) + 20,
    )


class FakeTeleop:
    def __init__(self):
        self.calls = []
        self.leader = make_state(np.ones(7))
        self.follower = make_state(np.ones(7) * 2)

    def robot_states(self, pair_index):
        self.calls.append(pair_index)
        return self.leader, self.follower


class FakeRdkRobot:
    def __init__(self, q_sequence=None, fault=False, operational=True):
        self.q_sequence = list(q_sequence or [np.zeros(7)])
        self.has_fault = fault
        self.is_operational = operational
        self.calls = []

    def states(self):
        q = self.q_sequence.pop(0) if len(self.q_sequence) > 1 else self.q_sequence[0]
        return make_state(q)

    def info(self):
        return SimpleNamespace(q_min=np.full(7, -2.0), q_max=np.full(7, 2.0))

    def fault(self):
        return self.has_fault

    def ClearFault(self):
        self.calls.append("ClearFault")
        self.has_fault = False
        return True

    def estop_released(self):
        return True

    def Enable(self):
        self.calls.append("Enable")

    def operational(self):
        return self.is_operational

    def operational_status(self):
        return "READY" if self.is_operational else "NOT_ENABLED"

    def SwitchMode(self, mode):
        self.calls.append(("SwitchMode", mode))

    def SendJointPosition(self, target, velocity, max_velocity, max_acceleration):
        self.calls.append((
            "SendJointPosition", target, velocity, max_velocity, max_acceleration,
        ))

    def Stop(self):
        self.calls.append("Stop")


class FakeTool:
    def __init__(self, robot):
        self.robot = robot

    def exist(self, name):
        return name != "missing"

    def Switch(self, name):
        self.robot.calls.append(("Tool.Switch", name))


def make_robot(fake, **kwargs):
    rdk = SimpleNamespace(
        Mode=SimpleNamespace(NRT_JOINT_IMPEDANCE="joint", IDLE="idle"),
        Tool=FakeTool,
    )
    return RdkFlexivRobot(
        "unused",
        rdk_module=rdk,
        robot=fake,
        sleep=kwargs.get("sleep", lambda _seconds: None),
        clock=kwargs.get("clock", lambda: 0.0),
    )


class TdkStateSourceTests(unittest.TestCase):
    def test_reads_and_copies_selected_follower_state(self):
        teleop = FakeTeleop()
        source = TdkFlexivStateSource(teleop, pair_index=2, robot="follower")
        state = source.read_state()
        self.assertEqual(teleop.calls, [2])
        self.assertEqual(state.timestamp_sdk_s, 12.5)
        np.testing.assert_array_equal(state.q, np.ones(7) * 2)
        self.assertFalse(np.shares_memory(state.q, teleop.follower.q))

    def test_rejects_malformed_tdk_pair(self):
        teleop = SimpleNamespace(robot_states=lambda _index: (make_state(),))
        with self.assertRaisesRegex(RuntimeError, "leader, follower"):
            TdkFlexivStateSource(teleop).read_state()

    def test_rejects_malformed_state_shape(self):
        teleop = FakeTeleop()
        teleop.follower.q = np.zeros(6)
        with self.assertRaisesRegex(RuntimeError, "q must have shape"):
            TdkFlexivStateSource(teleop).read_state()


class RdkFlexivRobotTests(unittest.TestCase):
    def test_prepare_requires_explicit_fault_clear(self):
        fake = FakeRdkRobot(fault=True)
        robot = make_robot(fake)
        with self.assertRaisesRegex(RuntimeError, "fault clearing is disabled"):
            robot.prepare(
                clear_fault=False, operational_timeout_s=1.0, poll_interval_s=0.1
            )
        self.assertNotIn("Enable", fake.calls)

    def test_prepare_clears_fault_and_enables(self):
        fake = FakeRdkRobot(fault=True)
        robot = make_robot(fake)
        robot.prepare(clear_fault=True, operational_timeout_s=1.0, poll_interval_s=0.1)
        self.assertEqual(fake.calls[:2], ["ClearFault", "Enable"])

    def test_move_joints_uses_joint_velocity_and_acceleration_limits(self):
        target = np.full(7, 0.5)
        fake = FakeRdkRobot(q_sequence=[np.zeros(7), target])
        robot = make_robot(fake)
        robot.move_joints(
            target,
            max_velocity_rad_s=0.1,
            max_acceleration_rad_s2=0.2,
            tolerance_rad=0.01,
            timeout_s=1.0,
            command_period_s=0.01,
        )
        self.assertEqual(fake.calls[0], ("SwitchMode", "joint"))
        command = fake.calls[1]
        self.assertEqual(command[0], "SendJointPosition")
        np.testing.assert_allclose(command[1], target)
        np.testing.assert_allclose(command[2], np.zeros(7))
        np.testing.assert_allclose(command[3], np.full(7, 0.1))
        np.testing.assert_allclose(command[4], np.full(7, 0.2))

    def test_move_joints_rejects_target_outside_reported_limits(self):
        fake = FakeRdkRobot()
        robot = make_robot(fake)
        with self.assertRaisesRegex(ValueError, "q1"):
            robot.move_joints(
                [3.0, 0, 0, 0, 0, 0, 0],
                max_velocity_rad_s=0.1,
                max_acceleration_rad_s2=0.2,
                tolerance_rad=0.01,
                timeout_s=1.0,
                command_period_s=0.01,
            )
        self.assertFalse(any(call[0] == "SwitchMode" for call in fake.calls))

    def test_switch_tool_uses_idle_mode(self):
        fake = FakeRdkRobot()
        robot = make_robot(fake)
        robot.switch_tool("xense")
        self.assertEqual(
            fake.calls,
            [("SwitchMode", "idle"), ("Tool.Switch", "xense")],
        )

    def test_switch_tool_rejects_unknown_tool(self):
        fake = FakeRdkRobot()
        robot = make_robot(fake)
        with self.assertRaisesRegex(RuntimeError, "does not exist"):
            robot.switch_tool("missing")
        self.assertEqual(fake.calls, [("SwitchMode", "idle")])

    def test_close_is_idempotent(self):
        fake = FakeRdkRobot()
        robot = make_robot(fake)
        robot.close()
        robot.close()
        self.assertEqual(fake.calls, ["Stop"])


if __name__ == "__main__":
    unittest.main()
