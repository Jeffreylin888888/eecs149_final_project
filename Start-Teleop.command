#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
  .venv/bin/python -m pip install -e '.[serial]'
fi
exec .venv/bin/python tools/robot_teleop.py "$@"
