#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/firmware/msp432"
echo 'Connect arm-none-eabi-gdb to localhost:3333. Motor power must be off while debugging: halted PWM can persist.'
exec openocd -f openocd.cfg -c 'init; halt'
