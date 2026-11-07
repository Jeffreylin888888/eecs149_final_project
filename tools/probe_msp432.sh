#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/firmware/msp432"
mkdir -p logs
# No reset, erase, programming, or resuming unknown firmware.
openocd -f openocd.cfg -f verify_target.tcl -c init -c targets -c halt \
  -c 'mdw 0xe000ed00 1' -c 'mdw 0x0020100c 2' -c 'mdw 0xe0043020 1' \
  -c 'flash info 0' -c verify_msp432_target -c shutdown 2>&1 | tee logs/probe.log
