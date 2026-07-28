#!/usr/bin/env python3
import math
from collections.abc import Callable
from pathlib import Path
import sys

from tap import Tap

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from flexiv_tdk_collect import RdkFlexivRobot


class ZeroForceTorqueArgs(Tap):
    # robot_sn: str = "Rizon4s-063652"   # Leader
    # tool_name: str = "tool1"
    robot_sn: str = "Rizon4s-063586"   # Follower
    tool_name: str = "hapticexoteleop"

    operational_timeout: float = 30.0
    zero_force_timeout: float = 30.0
    poll_interval: float = 0.1
    clear_fault: bool = True
    yes: bool = True

    def configure(self) -> None:
        self.description = "Zero one Flexiv robot force/torque sensor"
        self.add_argument("--robot-sn", dest="robot_sn", default=self.robot_sn)
        self.add_argument("--tool-name", dest="tool_name", default=self.tool_name)
        self.add_argument(
            "--operational-timeout", dest="operational_timeout", type=float,
            default=self.operational_timeout,
        )
        self.add_argument(
            "--zero-force-timeout", dest="zero_force_timeout", type=float,
            default=self.zero_force_timeout,
        )
        self.add_argument(
            "--poll-interval", dest="poll_interval", type=float,
            default=self.poll_interval,
        )
        self.add_argument(
            "--clear-fault", dest="clear_fault", type=_parse_bool,
            default=self.clear_fault,
        )
        self.add_argument(
            "--yes", dest="yes", type=_parse_bool, default=self.yes,
        )


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise ValueError(f"expected true or false, got {value!r}")


def validate_args(args: ZeroForceTorqueArgs) -> None:
    if not args.tool_name:
        raise ValueError("tool_name must not be empty")
    for name in ("operational_timeout", "zero_force_timeout", "poll_interval"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(
                f"--{name.replace('_', '-')} must be finite and greater than zero"
            )


def run_zeroing(
    args: ZeroForceTorqueArgs,
    input_fn: Callable[[str], str] = input,
    robot_factory: Callable[[str], RdkFlexivRobot] = RdkFlexivRobot,
) -> int:
    validate_args(args)
    if not args.yes:
        answer = input_fn(
            "Confirm the robot is unloaded and not in contact with anything? [y/N] "
        )
        if answer.strip().lower() not in {"y", "yes"}:
            print("Force/torque sensor zeroing cancelled")
            return 2

    robot = robot_factory(args.robot_sn)
    try:
        print("Keep the robot unloaded and free from external contact...")
        robot.prepare(
            clear_fault=args.clear_fault,
            tool_name=args.tool_name,
            operational_timeout_s=args.operational_timeout,
            poll_interval_s=args.poll_interval,
            zero_force_timeout_s=args.zero_force_timeout,
        )
        print("Force/torque sensor zeroing complete")
    finally:
        robot.close()
    return 0


def main() -> int:
    args = ZeroForceTorqueArgs().parse_args()
    try:
        return run_zeroing(args)
    except KeyboardInterrupt:
        print("\nForce/torque sensor zeroing interrupted")
        return 130
    except Exception as exc:
        print(f"Force/torque sensor zeroing failed: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
