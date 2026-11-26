"""Manual no-camera bilateral TDK collection with a desktop operator interface."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tap import Tap
from flexiv_capture.config import load_config, validate_force_only_config


class Args(Tap):
    config: Path = ROOT / "config.tdk_force_only.yaml"
    output: Path = ROOT / "recordings"
    execute: bool = False  # False: offline GUI preview; True: allow hardware start.
    check_only: bool = False  # Validate config and exit without opening a window.
    reset_home: bool = False  # Optional joint Home before each episode.


def main() -> int:
    args = Args().parse_args()
    config = load_config(args.config)
    validate_force_only_config(config)
    if args.check_only:
        print("Offline configuration check passed. No hardware SDK imported or connected.")
        return 0
    if args.execute and sys.platform != "linux":
        raise RuntimeError("Hardware execution is restricted to the original Linux SDK environment")
    from flexiv_capture.desktop import run_window
    run_window(config, args.output, execute=args.execute, reset_home=args.reset_home)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
