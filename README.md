# Radar Target Tracking Engine

Deterministic 2D multi-target radar tracking engine in C++17 using a constant-velocity Kalman Filter, Mahalanobis gating, track management, and reproducible simulation with golden hash verification.

This repository focuses on **state estimation and tracking logic** (not visualization). Output is written as **CSV logs** for offline analysis.

---

## Engineering Scope

This is a minimal tracking core intended as a systems-oriented demo:

- 2D constant-velocity (CV) target motion simulation
- Gaussian radar measurements (position-only) with configurable detection probability
- Kalman Filter predict-update with state vector `[x, y, vx, vy]`
- Multi-target tracking
  - Track IDs
  - Missed-detection handling
  - Track initiation and termination
  - Mahalanobis-distance gating
- Deterministic reproducibility via fixed seed and golden FNV-1a hash smoke test
- CSV logging: truth, measurements, track states, innovation and covariance statistics

---

## Outputs

The executable writes four CSV files:

- `truth.csv` — ground truth target states
- `meas.csv` — noisy radar measurements
- `tracks.csv` — estimated track states and management fields
- `residuals.csv` — innovation vector and innovation covariance (S)

---

## Build (Windows / MSYS2 UCRT64)

### Install dependencies


pacman -S --needed
mingw-w64-ucrt-x86_64-toolchain
mingw-w64-ucrt-x86_64-cmake
mingw-w64-ucrt-x86_64-ninja
mingw-w64-ucrt-x86_64-eigen3


### Configure and build


cmake -S . -B build -G Ninja
cmake --build build -j


---

## Run


./build/radar_tracker.exe --steps 300 --seed 123 --out out


Expected output:


FNV1A64=...
Wrote logs to: out
Files: truth.csv, meas.csv, tracks.csv, residuals.csv


---

## Determinism Smoke Test


chmod +x scripts/smoke.sh
./scripts/smoke.sh


Expected:


[SMOKE] PASS


---

## CLI Options


./build/radar_tracker.exe --help


Common parameters:

- `--seed`
- `--steps`
- `--dt`
- `--targets`
- `--sigma_z`
- `--p_detect`
- `--sigma_a`
- `--gate_maha2`
- `--confirm_hits`
- `--max_misses`
- `--out`

---

## License

MIT

---

## Author

Ali Eray Kalaycı  
Computer Engineering — Real-Time Systems & Tracking Focus