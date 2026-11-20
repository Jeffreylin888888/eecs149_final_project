#!/bin/bash
# Double-click on macOS, or run from a terminal. Uses the locked project environment.
set -euo pipefail
cd -- "$(dirname -- "$0")"
if ! command -v uv >/dev/null 2>&1; then
  export PATH="$HOME/.local/bin:$PATH"
fi
if ! command -v uv >/dev/null 2>&1; then
  echo 'Install uv from https://docs.astral.sh/uv/getting-started/installation/ or use the pip instructions in README.md.'
  exit 1
fi
uv sync --locked
uv run satex demo --scenario patrol --output results/patrol
if [[ "$(uname -s)" == Darwin ]]; then
  open results/patrol/trajectory.html
else
  echo 'Open results/patrol/trajectory.html in your browser.'
fi
