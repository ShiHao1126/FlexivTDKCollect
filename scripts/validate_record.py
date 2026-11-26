"""Read-only validation of a rebuilt force-only episode."""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tap import Tap
from flexiv_capture.validation import validate_episode


class Args(Tap):
    path: Path = ROOT / "recordings" / "episode_0000.hdf5"


if __name__ == "__main__":
    print(json.dumps(validate_episode(Args().parse_args().path), indent=2))
