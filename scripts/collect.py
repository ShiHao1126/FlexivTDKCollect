#!/usr/bin/env python3
from pathlib import Path
import sys
from typing import Any

from tap import Tap

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from flexiv_tdk_collect import (
    CollectConfig,
    CollectionSession,
    RdkFlexivRobot,
    TdkFlexivStateSource,
)


class CollectArgs(Tap):
    # Robot
    enable_robot: bool = True
    leader_sn: str = "Rizon4s-063652"
    follower_sn: str = "Rizon4s-063586"
    leader_tool: str = "gripper"
    follower_tool: str = "xense"
    lan_ip: list[str] = []
    output: Path = Path("records")
    robot_rate: float = 200.0

    # Gripper
    enable_gripper: bool = True
    gripper_macs: list[str] = ["1659f0e0dde0"]
    enable_tactile: bool = True
    tactile_sns: list[str] = ["OG001452", "OG001454"]
    xense_rate: float = 60.0

    # Camera
    enable_camera: bool = True
    serials: list[str] = []
    width: int = 640
    height: int = 480
    fps: int = 30
    collect_depth: bool = True

    def configure(self) -> None:
        self.description = "Flexiv TDK synchronized data collection"
        self.add_argument("--lan-ip", action="append", default=self.lan_ip)
        self.add_argument("--gripper-macs", action="append", default=self.gripper_macs)
        self.add_argument("--tactile-sns", action="append", default=self.tactile_sns)
        self.add_argument("--serials", action="append", default=self.serials)
        for name in (
            "enable_robot",
            "enable_gripper",
            "enable_tactile",
            "enable_camera",
            "collect_depth",
        ):
            self.add_argument(
                f"--{name.replace('_', '-')}",
                dest=name,
                type=_parse_bool,
                default=getattr(self, name),
            )


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise ValueError(f"expected true or false, got {value!r}")


def build_config(args: CollectArgs) -> CollectConfig:
    return CollectConfig(
        output_dir=args.output.resolve(),
        enable_robot=args.enable_robot,
        leader_sn=args.leader_sn,
        follower_sn=args.follower_sn,
        leader_tool=args.leader_tool,
        follower_tool=args.follower_tool,
        lan_ips=tuple(args.lan_ip),
        robot_rate_hz=args.robot_rate,
        enable_gripper=args.enable_gripper,
        gripper_macs=tuple(args.gripper_macs),
        enable_tactile=args.enable_tactile,
        tactile_sns=tuple(args.tactile_sns),
        xense_rate_hz=args.xense_rate,
        enable_camera=args.enable_camera,
        camera_serials=tuple(args.serials),
        camera_width=args.width,
        camera_height=args.height,
        camera_fps=args.fps,
        collect_depth=args.collect_depth,
    )


def start_robot_teleop(
    config: CollectConfig,
    *,
    robot_factory: Any = RdkFlexivRobot,
    tdk_module: Any = None,
) -> tuple[Any, TdkFlexivStateSource] | tuple[None, None]:
    if not config.enable_robot:
        return None, None

    for robot_sn, tool_name in (
        (config.leader_sn, config.leader_tool),
        (config.follower_sn, config.follower_tool),
    ):
        robot = robot_factory(robot_sn)
        try:
            robot.prepare(
                clear_fault=True,
                operational_timeout_s=30.0,
                poll_interval_s=0.1,
            )
            robot.switch_tool(tool_name)
        finally:
            robot.close()

    if tdk_module is None:
        import flexivtdk as tdk_module
    teleop = tdk_module.TransparentCartesianTeleopLAN(
        [(config.leader_sn, config.follower_sn)], list(config.lan_ips)
    )
    try:
        print("Initializing TDK (robot enable and FT zeroing may occur)...")
        teleop.Init()
        teleop.Start()
    except Exception:
        try:
            teleop.Stop()
        except Exception:
            pass
        raise
    return teleop, TdkFlexivStateSource(teleop, robot="follower")


def main() -> int:
    args = CollectArgs().parse_args()
    config = build_config(args)
    teleop = None
    session = None
    try:
        teleop, robot_state_source = start_robot_teleop(config)
        session = CollectionSession(robot_state_source, config)
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
