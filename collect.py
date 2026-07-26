#!/usr/bin/env python3
from pathlib import Path

from tap import Tap

from flexiv_tdk_collect import CollectConfig, CollectionSession


class CollectArgs(Tap):
    leader_sn: str
    follower_sn: str = "Rizon4s-063586"
    lan_ip: list[str] = []
    output: Path = Path("records")

    gripper_mac: str = "1659f0e0dde0"
    left_tactile_sn: str = "OG001452"
    right_tactile_sn: str = "OG001454"
    camera_serial: list[str] = []

    robot_rate: float = 200.0
    xense_rate: float = 60.0

    enable_robot: bool = True
    enable_robot_joint_state: bool = True
    enable_robot_torque: bool = True
    enable_robot_tcp: bool = True
    enable_robot_wrench_tcp: bool = True
    enable_robot_wrench_world: bool = True

    enable_camera_color: bool = True
    camera_color_width: int = 640
    camera_color_height: int = 480
    camera_color_fps: int = 30
    enable_camera_depth: bool = True
    camera_depth_width: int = 640
    camera_depth_height: int = 480
    camera_depth_fps: int = 30
    align_depth_to_color: bool = True

    enable_gripper_width: bool = True
    enable_tactile_marker: bool = True
    enable_tactile_force: bool = True
    enable_tactile_rectify: bool = True
    enable_tactile_difference: bool = True
    enable_tactile_depth: bool = True

    def configure(self) -> None:
        self.description = "Flexiv TDK synchronized data collection"
        self.add_argument("--leader-sn", dest="leader_sn", required=True)
        self.add_argument("--follower-sn", dest="follower_sn", default="Rizon4s-063586")
        self.add_argument("--lan-ip", action="append", default=[])
        self.add_argument("--gripper-mac", dest="gripper_mac", default="1659f0e0dde0")
        self.add_argument("--left-tactile-sn", dest="left_tactile_sn", default="OG001452")
        self.add_argument("--right-tactile-sn", dest="right_tactile_sn", default="OG001454")
        self.add_argument("--camera-serial", dest="camera_serial", action="append", default=[])
        self.add_argument("--robot-rate", dest="robot_rate", type=float, default=200.0)
        self.add_argument("--xense-rate", dest="xense_rate", type=float, default=60.0)
        for name in (
            "enable_robot", "enable_robot_joint_state", "enable_robot_torque",
            "enable_robot_tcp", "enable_robot_wrench_tcp", "enable_robot_wrench_world",
            "enable_camera_color", "enable_camera_depth", "align_depth_to_color",
            "enable_gripper_width", "enable_tactile_marker", "enable_tactile_force",
            "enable_tactile_rectify", "enable_tactile_difference", "enable_tactile_depth",
        ):
            self.add_argument(f"--{name.replace('_', '-')}", dest=name, type=_parse_bool, default=True)
        self.add_argument("--camera-color-width", dest="camera_color_width", type=int, default=640)
        self.add_argument("--camera-color-height", dest="camera_color_height", type=int, default=480)
        self.add_argument("--camera-color-fps", dest="camera_color_fps", type=int, default=30)
        self.add_argument("--camera-depth-width", dest="camera_depth_width", type=int, default=640)
        self.add_argument("--camera-depth-height", dest="camera_depth_height", type=int, default=480)
        self.add_argument("--camera-depth-fps", dest="camera_depth_fps", type=int, default=30)


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise ValueError(f"expected true or false, got {value!r}")


def main() -> int:
    import flexivtdk

    args = CollectArgs().parse_args()
    config = CollectConfig(output_dir=args.output.resolve(), leader_sn=args.leader_sn,
                           follower_sn=args.follower_sn, lan_ips=tuple(args.lan_ip),
                           gripper_mac=args.gripper_mac,
                           left_tactile_sn=args.left_tactile_sn,
                           right_tactile_sn=args.right_tactile_sn,
                           robot_rate_hz=args.robot_rate, xense_rate_hz=args.xense_rate,
                           enable_robot=args.enable_robot,
                           enable_robot_joint_state=args.enable_robot_joint_state,
                           enable_robot_torque=args.enable_robot_torque,
                           enable_robot_tcp=args.enable_robot_tcp,
                           enable_robot_wrench_tcp=args.enable_robot_wrench_tcp,
                           enable_robot_wrench_world=args.enable_robot_wrench_world,
                           camera_serials=tuple(args.camera_serial),
                           enable_camera_color=args.enable_camera_color,
                           camera_color_width=args.camera_color_width,
                           camera_color_height=args.camera_color_height,
                           camera_color_fps=args.camera_color_fps,
                           enable_camera_depth=args.enable_camera_depth,
                           camera_depth_width=args.camera_depth_width,
                           camera_depth_height=args.camera_depth_height,
                           camera_depth_fps=args.camera_depth_fps,
                           align_depth_to_color=args.align_depth_to_color,
                           enable_gripper_width=args.enable_gripper_width,
                           enable_tactile_marker=args.enable_tactile_marker,
                           enable_tactile_force=args.enable_tactile_force,
                           enable_tactile_rectify=args.enable_tactile_rectify,
                           enable_tactile_difference=args.enable_tactile_difference,
                           enable_tactile_depth=args.enable_tactile_depth)
    teleop = None
    session = None
    try:
        teleop = flexivtdk.TransparentCartesianTeleopLAN(
            [(config.leader_sn, config.follower_sn)], list(config.lan_ips)
        )
        session = CollectionSession(teleop, config)
        print("Initializing TDK (robot enable and FT zeroing may occur)...")
        teleop.Init()
        teleop.Start()
        print("Ready: c=start record, v=finish record, q=quit")
        while True:
            command = input("> ").strip().lower()
            if command == "c":
                if session.recording:
                    print("Already recording")
                else:
                    print(f"Recording: {session.start()}")
            elif command == "v":
                if session.recording:
                    print(f"Saved: {session.finish()}")
                else:
                    print("No active record")
            elif command == "q":
                if session.recording:
                    print(f"Interrupted record: {session.finish('operator_quit')}")
                return 0
            else:
                print("Command must be c, v, or q")
    except (KeyboardInterrupt, EOFError):
        if session is not None and session.recording:
            print(f"\nInterrupted record: {session.finish('interrupted')}")
        return 130
    except Exception as exc:
        if session is not None and session.recording:
            print(f"Invalid record: {session.finish('runtime_error')}")
        print(f"Collection failed: {exc}")
        return 1
    finally:
        if teleop is not None:
            try:
                teleop.Stop()
            except Exception as exc:
                print(f"TDK stop failed: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
