#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
command -v arm-none-eabi-gcc >/dev/null || { echo 'Install Arm GNU embedded toolchain for macOS first.' >&2; exit 1; }
command -v openocd >/dev/null || brew install open-ocd
BASE="$ROOT/firmware/msp432/vendor/simplelink_msp432p4_sdk_3_40_01_02"
SDK="$BASE/simplelink_msp432p4_sdk_3_40_01_02"
if [[ -f "$SDK/source/ti/devices/msp432p4xx/driverlib/gcc/msp432p4xx_driverlib.a" ]]; then
  echo "TI SDK already present: $SDK"; exit 0
fi
WORK="$ROOT/work/msp432-setup"
mkdir -p "$WORK" "$BASE"
curl -fL 'https://dr-download.ti.com/software-development/software-development-kit-sdk/MD-dSV82B3Lb6/3.40.01.02/simplelink_msp432p4_sdk_3_40_01_02.app.zip' -o "$WORK/sdk.zip"
# Digest of vendor download obtained and used on 2026-11-06, not an independent vendor signature.
ACTUAL="$(shasum -a 256 "$WORK/sdk.zip" | cut -d ' ' -f1)"
[[ "$ACTUAL" == 8a059e9031f722ee1aa465287fbe8e08148c57ba3e191b32c1043fe9df6da322 ]] || { echo 'SDK archive changed; inspect before installing.' >&2; exit 1; }
unzip -q -o "$WORK/sdk.zip" -d "$WORK"
"$WORK/simplelink_msp432p4_sdk_3_40_01_02.app/Contents/MacOS/osx-x86_64" --mode unattended --prefix "$BASE"
test -f "$SDK/source/ti/devices/msp432p4xx/driverlib/gcc/msp432p4xx_driverlib.a"
echo "SDK installed: $SDK"
