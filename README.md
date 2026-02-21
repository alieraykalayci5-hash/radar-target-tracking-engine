Radar Target Tracking Engine

Deterministic 2D multi-target radar tracking engine in C++17 implementing a constant-velocity Kalman Filter, Mahalanobis gating, multi-target data association (Greedy & Hungarian), and reproducible simulation with golden-hash verification.

This project focuses purely on state estimation and tracking logic.
Visualization is provided separately via Python plotting scripts.

1. System Overview

This repository implements a full tracking pipeline.

Motion & Measurement Model

2D constant-velocity (CV) motion model

State vector: [x, y, vx, vy]^T

Discrete white-noise acceleration process model

Gaussian radar measurements (position only)

Configurable detection probability

Optional uniform clutter injection

Estimation Core

Kalman Filter (predict-update)

Innovation computation

Innovation covariance (S)

NIS (Normalized Innovation Squared)

Mahalanobis-distance gating

Multi-Target Tracking

Track IDs

Missed detection handling

Track initiation from unassigned measurements

Track termination via miss threshold

M-of-N confirmation logic

Greedy nearest-neighbor association

Hungarian global assignment (optional)

Determinism & Reproducibility

Fixed random seed

FNV-1a 64-bit hash of simulation outputs

Smoke test for regression detection

2. Repository Structure
src/
  main.cpp
  sim.cpp / sim.h
  tracker.cpp / tracker.h
  kalman.cpp / kalman.h
  hungarian.cpp / hungarian.h
  math_types.h
  rng.h
  csv.h
  fnv1a.h

scripts/
  smoke.sh

tools/
  plot_tracks.py
  plot_nis.py
  requirements.txt
3. Build (Windows – MSYS2 UCRT64)
Install Dependencies
pacman -S --needed \
  mingw-w64-ucrt-x86_64-toolchain \
  mingw-w64-ucrt-x86_64-cmake \
  mingw-w64-ucrt-x86_64-ninja \
  mingw-w64-ucrt-x86_64-eigen3
Configure and Build
cd /c/Users/AliEray/Desktop/Staj-Proje/radar-target-tracking-engine

cmake -S . -B build -G Ninja
cmake --build build -j
4. Running the Simulation

Example:

./build/radar_tracker.exe \
  --steps 400 \
  --seed 123 \
  --scenario cross \
  --hungarian 1 \
  --clutter 0 \
  --p_detect 1.0 \
  --sigma_z 15 \
  --gate_maha2 50 \
  --out out_smoke

Expected output:

FNV1A64=9c3d2602b240fd45
Wrote logs to: out_smoke
Files: truth.csv, meas.csv, tracks.csv, residuals.csv
5. Output Files

Each run generates:

truth.csv — ground-truth states

meas.csv — noisy radar measurements (+ clutter if enabled)

tracks.csv — estimated track states

residuals.csv — innovation and covariance statistics

6. Association Comparison

Built-in demo:

./build/radar_tracker.exe --assoc_demo 1

Example output:

=== ASSOC DEMO (Greedy vs Hungarian) ===
cost matrix:
  row 0: 1, 2
  row 1: 2, 100

greedy assignment:    total_cost = 101
hungarian assignment: total_cost = 4

Demonstrates why global assignment can outperform greedy matching.

7. Visualization (Python Tools)

Install plotting dependencies:

python -m pip install -r tools/requirements.txt

Generate plots:

python tools/plot_tracks.py --in out_smoke
python tools/plot_nis.py --in out_smoke

Outputs:

plots/tracks_out_smoke.png
plots/nis_out_smoke.png

These plots validate:

Stable estimation

No ID swaps (cross scenario)

Proper statistical behavior

8. Deterministic Smoke Test
chmod +x scripts/smoke.sh
./scripts/smoke.sh

Expected:

[SMOKE] expected=9c3d2602b240fd45
[SMOKE] got     =9c3d2602b240fd45
[SMOKE] PASS

If the hash changes, simulation behavior or output formatting changed.

9. CLI Parameters
Parameter	Description
--steps	Number of simulation steps
--dt	Timestep
--targets	Number of targets
--sigma_z	Measurement noise std
--sigma_a	Process noise std
--p_detect	Detection probability
--clutter	Enable clutter (0/1)
--clutter_n	Clutter per step
--clutter_A	Clutter area half-size
--gate_maha2	Mahalanobis gate threshold
--confirm_M	Confirmation hits
--confirm_N	Confirmation window
--hungarian	Use global assignment
--scenario	Scenario type (default/cross)
--seed	Random seed
--out	Output directory
10. Performance Characteristics

Example benchmark (cross scenario, no clutter):

steps=400
targets=2
sigma_z=15
hungarian=1
steps_per_sec ≈ 13,000

Performance is deterministic and reproducible.

11. Engineering Highlights

Fully deterministic simulation core

Regression-detecting golden hash

Multi-target lifecycle management

Configurable clutter model

Greedy vs Hungarian comparison

Statistical validation via NIS

Clean CMake-based build

Eigen-based linear algebra

12. Technologies

C++17

Eigen (linear algebra)

CMake

Ninja

Python (matplotlib, numpy)

13. License

MIT

Author

Ali Eray Kalaycı
Computer Engineering
Focus: Real-Time Systems, Tracking & Estimation, Autonomous Systems