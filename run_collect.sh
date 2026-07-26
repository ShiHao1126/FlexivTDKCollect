#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="/home/xense/miniconda3/envs/haptic_exo_env/bin/python"
exec "$PYTHON" "$ROOT_DIR/collect.py" "$@"
