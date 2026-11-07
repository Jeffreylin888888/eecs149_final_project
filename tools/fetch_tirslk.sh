#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/work/upstream"
FILE="$ROOT/work/upstream/tirslk_max_1_00_00.zip"
curl -fL 'https://users.ece.utexas.edu/~valvano/arm/tirslk_max_1_00_00.zip' -o "$FILE"
ACTUAL="$(shasum -a 256 "$FILE" | cut -d ' ' -f1)"
[[ "$ACTUAL" == 2e2e50002ca6a6f5a31d0156d6ef6703061c0913b198e8d7b9c77f093a19b1fc ]] || { echo 'Archive changed; review before using.' >&2; exit 1; }
unzip -q -o "$FILE" -d "$ROOT/work/upstream"
echo "Official source downloaded and extracted: $ROOT/work/upstream"
