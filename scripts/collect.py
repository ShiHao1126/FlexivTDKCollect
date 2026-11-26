"""No-camera bilateral TDK collection. Default execution only validates config."""
from pathlib import Path
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tap import Tap
from flexiv_capture.config import load_config, validate_force_only_config


class Args(Tap):
    config: Path = ROOT / "config.tdk_force_only.yaml"
    output: Path = ROOT / "recordings"
    execute: bool = False  # False: offline configuration check; True: connects to hardware.
    duration_s: float = 30.0  # Recording duration per episode, seconds.
    reset_home: bool = False  # Explicit optional motion before recording.


def main() -> int:
    args = Args().parse_args()
    config = load_config(args.config)
    validate_force_only_config(config)
    if not math.isfinite(args.duration_s) or args.duration_s <= 0:
        raise ValueError("duration_s must be finite and positive")
    print(json.dumps({"mode": "tdk_force_only", "execute": args.execute,
                      "leader": config["teleop"]["leader_sn"], "follower": config["robot"]["robot_sn"],
                      "duration_s": args.duration_s, "reset_home": args.reset_home}, indent=2))
    if not args.execute:
        print("Offline configuration check passed. No hardware SDK imported or connected.")
        return 0
    if sys.platform != "linux":
        raise RuntimeError("Hardware execution is restricted to the original Linux SDK environment")
    print("Check serial numbers, workspace, unloaded FT sensors and reachable emergency stop.")
    if input("Type RUN to initialize/zero and engage both arms: ").strip() != "RUN":
        return 0
    from flexiv_capture.recording import Recorder, finalize
    recorder = Recorder(config, args.output)
    try:
        reset = recorder.reset_paired_home() if args.reset_home else None
        partial, final, stats = recorder.record_episode(args.duration_s, start_reset=reset)
        print(json.dumps(stats, indent=2))
        if input("Both arms stopped. Type SAVE to publish this episode: ").strip() == "SAVE":
            print(json.dumps(finalize(partial, final), indent=2))
            print(final)
        else:
            print(f"Retained unapproved partial: {partial}")
    finally:
        recorder.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
