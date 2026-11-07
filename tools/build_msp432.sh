#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/firmware/msp432/logs"
make -C "$ROOT/firmware/msp432" all test build/control-fixture.elf 2>&1 | tee "$ROOT/firmware/msp432/logs/build-test.log"
