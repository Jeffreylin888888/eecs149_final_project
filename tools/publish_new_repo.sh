#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NAME="${1:-eecs149_robot_complete}"
cd "$ROOT"
if ! command -v gh >/dev/null; then
  echo 'GitHub CLI is not installed. Run: brew install gh; gh auth login' >&2
  exit 1
fi
if ! gh auth status >/dev/null 2>&1; then
  echo 'Authenticate first with: gh auth login' >&2
  exit 1
fi
if [[ ! -d .git ]]; then
  git init -b main
  git add .
  git commit -m 'Initial EECS149 mobile manipulation project'
fi
if git remote get-url origin >/dev/null 2>&1; then
  echo 'A remote named origin already exists; refusing to overwrite it.' >&2
  exit 1
fi
gh repo create "$NAME" --public --source=. --remote=origin --push
