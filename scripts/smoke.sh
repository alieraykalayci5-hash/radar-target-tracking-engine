#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

cmake -S . -B build -G Ninja >/dev/null
cmake --build build -j >/dev/null

OUTDIR="out_smoke"
rm -rf "$OUTDIR"
mkdir -p "$OUTDIR"

# Golden scenario: ambiguous cross + higher noise + wide gate, clutter off.
EXPECTED="9c3d2602b240fd45"

GOT="$(./build/radar_tracker.exe \
  --scenario cross \
  --steps 400 \
  --seed 123 \
  --out "$OUTDIR" \
  --hungarian 1 \
  --clutter 0 \
  --p_detect 1.0 \
  --sigma_z 15 \
  --gate_maha2 50 \
  2>&1 | sed -n 's/^FNV1A64=//p' | head -n 1)"

echo "[SMOKE] expected=$EXPECTED"
echo "[SMOKE] got     =$GOT"

if [[ "$GOT" != "$EXPECTED" ]]; then
  echo "[SMOKE] FAIL"
  exit 1
fi

echo "[SMOKE] PASS"