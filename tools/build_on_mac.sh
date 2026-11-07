#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUCKLER="${BUCKLER_ROOT:-$HOME/buckler}"
DEST="$BUCKLER/software/apps/eecs149_robot"
if [[ ! -f "$BUCKLER/software/apps/blink/Makefile" ]]; then
  echo "Buckler source not found at $BUCKLER; set BUCKLER_ROOT or clone lab11/buckler first." >&2
  exit 1
fi
if ! command -v arm-none-eabi-gcc >/dev/null; then
  echo "ARM GCC not found; install the compiler first." >&2
  exit 1
fi
mkdir -p "$DEST"
cp -R "$ROOT/firmware/buckler/." "$DEST/"
printf 'Firmware copied to %s\n' "$DEST"
printf 'Running build only. No firmware will be flashed.\n'
(cd "$DEST" && make clean && make)
echo 'Build completed. Motor/servo pins remain disabled by default.'
