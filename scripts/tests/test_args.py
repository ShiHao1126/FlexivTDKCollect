import unittest
from pathlib import Path
from types import SimpleNamespace

from scripts.collect import (
    CollectArgs,
    build_config,
    set_robot_teleop_engaged,
    start_robot_teleop,
    stop_robot_teleop,
)


class FakeRobot:
    def __init__(self, serial, calls):
        self.serial = serial
        self.calls = calls

    def prepare(self, **kwargs):
        self.calls.append((self.serial, "prepare", kwargs))

    def close(self):
        self.calls.append((self.serial, "close"))


class FakeTeleop:
    def __init__(self, pairs, lan_ips):
        self.pairs = pairs
        self.lan_ips = lan_ips
        self.calls = []

    def Init(self):
        self.calls.append("Init")

    def Start(self):
        self.calls.append("Start")

    def Engage(self, index, engaged):
        self.calls.append(("Engage", index, engaged))

    def Stop(self):
        self.calls.append("Stop")


class CollectArgsTests(unittest.TestCase):
    def test_defaults_are_ready_for_direct_run(self):
        args = CollectArgs().parse_args([])
        self.assertEqual(args.leader_sn, "Rizon4s-063652")
        self.assertEqual(args.follower_sn, "Rizon4s-063586")
        self.assertEqual(args.leader_tool, "tool1")
        self.assertEqual(args.follower_tool, "hapticexoteleop")
        self.assertEqual(args.gripper_macs, ["1659f0e0dde0"])
        self.assertEqual(args.tactile_sns, ["OG001452"])
        self.assertEqual(args.serials, ["327322062498"])
        self.assertEqual((args.width, args.height, args.fps), (640, 480, 30))
        self.assertTrue(args.collect_depth)
        self.assertEqual(args.output, Path("records"))

    def test_build_config_preserves_device_lists_and_enable_semantics(self):
        args = CollectArgs().parse_args([])
        args.enable_robot = False
        args.enable_camera = False
        args.collect_depth = False
        config = build_config(args)
        self.assertFalse(config.enable_robot)
        self.assertFalse(config.enable_camera)
        self.assertFalse(config.collect_depth)
        self.assertEqual(config.gripper_macs, ("1659f0e0dde0",))
        self.assertEqual(config.tactile_sns, ("OG001452",))

    def test_disabled_robot_does_not_create_rdk_or_tdk(self):
        args = CollectArgs().parse_args([])
        args.enable_robot = False
        config = build_config(args)
        result = start_robot_teleop(
            config,
            robot_factory=lambda _serial: self.fail("RDK must not be created"),
            tdk_module=SimpleNamespace(
                TransparentCartesianTeleopLAN=lambda *_args: self.fail("TDK must not be created")
            ),
        )
        self.assertEqual(result, (None, None))

    def test_enabled_robot_configures_tools_before_starting_tdk(self):
        args = CollectArgs().parse_args([])
        config = build_config(args)
        calls = []
        teleops = []

        def create_teleop(pairs, lan_ips):
            teleop = FakeTeleop(pairs, lan_ips)
            teleops.append(teleop)
            return teleop

        teleop, source = start_robot_teleop(
            config,
            robot_factory=lambda serial: FakeRobot(serial, calls),
            tdk_module=SimpleNamespace(TransparentCartesianTeleopLAN=create_teleop),
        )
        self.assertEqual(
            calls,
            [
                (config.leader_sn, "prepare", {
                    "clear_fault": True,
                    "tool_name": config.leader_tool,
                    "operational_timeout_s": 30.0,
                    "poll_interval_s": 0.1,
                    "zero_force_timeout_s": 30.0,
                }),
                (config.leader_sn, "close"),
                (config.follower_sn, "prepare", {
                    "clear_fault": True,
                    "tool_name": config.follower_tool,
                    "operational_timeout_s": 30.0,
                    "poll_interval_s": 0.1,
                    "zero_force_timeout_s": 30.0,
                }),
                (config.follower_sn, "close"),
            ],
        )
        self.assertIs(teleop, teleops[0])
        self.assertEqual(
            teleop.calls,
            ["Init", "Start", ("Engage", 0, False)],
        )
        self.assertIsNotNone(source)


    def test_teleop_engage_and_stop_are_explicit(self):
        teleop = FakeTeleop([], [])

        set_robot_teleop_engaged(teleop, True)
        stop_robot_teleop(teleop)

        self.assertEqual(
            teleop.calls,
            [("Engage", 0, True), ("Engage", 0, False), "Stop"],
        )


if __name__ == "__main__":
    unittest.main()
