"""Manual no-camera bilateral TDK collection with terminal or desktop interfaces."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tap import Tap
from flexiv_capture.config import load_config, validate_force_only_config


class Args(Tap):
    config: Path = ROOT / "config.tdk_force_only.yaml"
    output: Path = ROOT / "recordings"
    interface: str = "terminal"  # terminal for SSH; gui for a graphical desktop.
    execute: bool = False  # False: offline preview; True: allow hardware start.
    check_only: bool = False  # Validate config and exit without opening a window.


def main() -> int:
    args = Args().parse_args()
    config = load_config(args.config)
    validate_force_only_config(config)
    if args.check_only:
        print("Offline configuration check passed. No hardware SDK imported or connected.")
        return 0
    if args.execute and sys.platform != "linux":
        raise RuntimeError("Hardware execution is restricted to the original Linux SDK environment")
    if args.interface == "terminal":
        from flexiv_capture.manual_session import ManualSession
        from flexiv_capture.terminal import run_terminal
        return run_terminal(ManualSession(config, args.output), execute=args.execute)
    if args.interface == "gui":
        from flexiv_capture.desktop import run_window
        run_window(config, args.output, execute=args.execute)
        return 0
    raise ValueError("interface must be terminal or gui")


if __name__ == "__main__":
    raise SystemExit(main())
