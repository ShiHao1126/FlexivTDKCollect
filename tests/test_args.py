import unittest
from pathlib import Path

from collect import CollectArgs


class CollectArgsTests(unittest.TestCase):
    def test_typed_arguments_and_enable_flags(self):
        args = CollectArgs().parse_args([
            "--leader-sn", "Rizon4-123456",
            "--lan-ip", "192.168.2.10",
            "--lan-ip", "192.168.3.10",
            "--output", "records_test",
            "--robot-rate", "250",
            "--xense-rate", "50",
            "--enable-camera-color", "false",
            "--enable-camera-depth", "false",
            "--enable-gripper-width", "false",
            "--enable-tactile-marker", "false",
        ])

        self.assertEqual(args.leader_sn, "Rizon4-123456")
        self.assertEqual(args.lan_ip, ["192.168.2.10", "192.168.3.10"])
        self.assertEqual(args.output, Path("records_test"))
        self.assertEqual(args.robot_rate, 250.0)
        self.assertEqual(args.xense_rate, 50.0)
        self.assertFalse(args.enable_camera_color)
        self.assertFalse(args.enable_camera_depth)
        self.assertFalse(args.enable_gripper_width)
        self.assertFalse(args.enable_tactile_marker)

    def test_camera_profiles_are_independent(self):
        args = CollectArgs().parse_args([
            "--leader-sn", "leader",
            "--camera-serial", "123456",
            "--camera-serial", "789012",
            "--camera-color-width", "1280",
            "--camera-color-height", "720",
            "--camera-color-fps", "15",
            "--camera-depth-width", "640",
            "--camera-depth-height", "480",
            "--camera-depth-fps", "30",
            "--align-depth-to-color", "false",
        ])
        self.assertEqual(args.camera_serial, ["123456", "789012"])
        self.assertEqual((args.camera_color_width, args.camera_color_height, args.camera_color_fps), (1280, 720, 15))
        self.assertEqual((args.camera_depth_width, args.camera_depth_height, args.camera_depth_fps), (640, 480, 30))
        self.assertFalse(args.align_depth_to_color)


if __name__ == "__main__":
    unittest.main()
