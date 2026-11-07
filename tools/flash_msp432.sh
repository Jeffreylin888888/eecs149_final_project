#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${1:-}"
if [[ "$MODE" != led && "$MODE" != wheels && "$MODE" != control ]]; then
  echo 'Usage: flash_msp432.sh led|wheels|control --confirmed-target-and-overwrite [--confirmed-chassis-safe-wheels-lifted]' >&2; exit 2
fi
if [[ "${2:-}" != --confirmed-target-and-overwrite ]]; then
  echo 'Confirm MSP432P401R target and replacement of its existing application first.' >&2; exit 2
fi
if [[ "$MODE" != led && "${3:-}" != --confirmed-chassis-safe-wheels-lifted ]]; then
  echo 'Confirm removed 5V jumper, mounting, wiring, power, and lifted wheels before motor firmware.' >&2; exit 2
fi
cd "$ROOT/firmware/msp432"
make "build/$MODE.elf"
mkdir -p backups logs
STAMP="$(date +%Y%m%d-%H%M%S)"
# Complete main flash backup is mandatory; error aborts before programming.
# The probe log must be reviewed for the exact device before this script is used.
openocd -f openocd.cfg -f verify_target.tcl \
  -c init -c halt -c verify_msp432_target -c "dump_image backups/main-$STAMP.bin 0 0x40000" -c "dump_image backups/main-$STAMP-check.bin 0 0x40000" -c shutdown \
  2>&1 | tee "logs/backup-$STAMP.log"
cmp "backups/main-$STAMP.bin" "backups/main-$STAMP-check.bin"
[[ "$(wc -c < "backups/main-$STAMP.bin" | tr -d ' ')" == 262144 ]]
# Sector erase only for this image. No mass erase, unlock, BSL or information flash writes.
openocd -f openocd.cfg -f verify_target.tcl -c init -c halt -c verify_msp432_target -c "program build/$MODE.elf verify reset exit" 2>&1 | tee "logs/flash-$MODE-$STAMP.log"
