#!/usr/bin/env python3
import math
from collections.abc import Callable
from pathlib import Path
import sys

import numpy as np
from tap import Tap

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from flexiv_tdk_collect import RdkFlexivRobot


class HomeArgs(Tap):
    # robot_sn: str = "Rizon4s-063652"   # Leader
    robot_sn: str = "Rizon4s-063586"   # Follower

    target_q: list[float] = [0, -40, 0, 90, 0, 40, 45]
    joint_max_velocity: float = 5.0
    joint_max_acceleration: float = 5.0
    joint_tolerance: float = 0.5
    operational_timeout: float = 30.0
    motion_timeout: float = 120.0
    poll_interval: float = 0.1
    clear_fault: bool = True
    yes: bool = True

    def configure(self) -> None:
        self.description = "Move one Flexiv robot to a configured joint posture"
        self.add_argument("--robot-sn", dest="robot_sn", default=self.robot_sn)
        self.add_argument(
            "--target-q", dest="target_q", type=float, nargs=7, default=self.target_q,
            metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
            help="target arm joint positions in degrees",
        )
        self.add_argument(
            "--operational-timeout", dest="operational_timeout", type=float,
            default=self.operational_timeout, help="seconds to wait after enabling",
        )
        self.add_argument(
            "--motion-timeout", dest="motion_timeout", type=float,
            default=self.motion_timeout, help="seconds to wait for joint motion",
        )
        self.add_argument(
            "--poll-interval", dest="poll_interval", type=float,
            default=self.poll_interval, help="control and status polling interval in seconds",
        )
        self.add_argument(
            "--clear-fault", dest="clear_fault", type=_parse_bool,
            default=self.clear_fault, help="explicitly allow clearing an existing robot fault",
        )
        self.add_argument(
            "--yes", dest="yes", type=_parse_bool, default=self.yes,
            help="skip the operator confirmation prompt",
        )


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise ValueError(f"expected true or false, got {value!r}")


def validate_args(args: HomeArgs) -> tuple[float, ...]:
    target_q = tuple(args.target_q)
    if len(target_q) != 7 or not all(math.isfinite(value) for value in target_q):
        raise ValueError("--target-q requires exactly 7 finite joint angles")
    for name in (
        "joint_max_velocity", "joint_max_acceleration", "joint_tolerance",
        "operational_timeout", "motion_timeout", "poll_interval",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(
                f"--{name.replace('_', '-')} must be finite and greater than zero"
            )
    return target_q


def run_homing(
    args: HomeArgs,
    input_fn: Callable[[str], str] = input,
    robot_factory: Callable[[str], RdkFlexivRobot] = RdkFlexivRobot,
) -> int:
    target_q = validate_args(args)
    print("Target joint angles [deg]: " + " ".join(f"{value:.3f}" for value in target_q))
    if not args.yes:
        answer = input_fn("Confirm workspace is clear and move robot? [y/N] ")
        if answer.strip().lower() not in {"y", "yes"}:
            print("Homing cancelled")
            return 2

    robot = robot_factory(args.robot_sn)
    try:
        print("Enabling robot...")
        robot.prepare(
            clear_fault=args.clear_fault,
            operational_timeout_s=args.operational_timeout,
            poll_interval_s=args.poll_interval,
        )
        robot.move_joints(
            np.deg2rad(target_q),
            max_velocity_rad_s=math.radians(args.joint_max_velocity),
            max_acceleration_rad_s2=math.radians(args.joint_max_acceleration),
            tolerance_rad=math.radians(args.joint_tolerance),
            timeout_s=args.motion_timeout,
            command_period_s=args.poll_interval,
        )
        print("Homing target reached")
    except BaseException:
        try:
            robot.close()
        except Exception as close_error:
            print(f"Robot stop also failed: {close_error}")
        raise
    else:
        robot.close()
    return 0


def main() -> int:
    args = HomeArgs().parse_args()
    try:
        return run_homing(args)
    except KeyboardInterrupt:
        print("\nHoming interrupted")
        return 130
    except Exception as exc:
        print(f"Homing failed: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
