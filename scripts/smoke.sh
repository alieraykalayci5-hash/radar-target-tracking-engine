#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# Golden run config (v1)
SEED=123
STEPS=300
EXPECTED="4541f796003b5a1e"

# Build
cmake -S . -B build -G Ninja
cmake --build build -j

# Run (stderr contains FNV line)
OUTDIR="out_smoke"
rm -rf "$OUTDIR"
mkdir -p "$OUTDIR"

FNV_LINE="$("./build/radar_tracker.exe" --steps "$STEPS" --seed "$SEED" --out "$OUTDIR" 2>&1 >/dev/null | grep -E "^FNV1A64=" || true)"
if [[ -z "${FNV_LINE}" ]]; then
  echo "[SMOKE] FAIL: FNV1A64 line not found"
  exit 1
fi

GOT="${FNV_LINE#FNV1A64=}"

echo "[SMOKE] expected=$EXPECTED"
echo "[SMOKE] got     =$GOT"

if [[ "$GOT" != "$EXPECTED" ]]; then
  echo "[SMOKE] FAIL: hash mismatch"
  exit 1
fi

echo "[SMOKE] PASS"