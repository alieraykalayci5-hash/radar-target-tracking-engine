#!/usr/bin/env python3
import argparse
import csv
import os
from collections import defaultdict

import numpy as np
import matplotlib.pyplot as plt


def nis_2d(innov_x, innov_y, S00, S01, S10, S11):
    # NIS = v^T S^{-1} v, with v = [innov_x, innov_y]
    # Inverse of 2x2:
    # inv(S) = (1/det) * [ S11, -S01; -S10, S00 ]
    det = S00 * S11 - S01 * S10
    if det == 0.0:
        return np.nan
    inv00 = S11 / det
    inv01 = -S01 / det
    inv10 = -S10 / det
    inv11 = S00 / det
    v0 = innov_x
    v1 = innov_y
    return v0 * (inv00 * v0 + inv01 * v1) + v1 * (inv10 * v0 + inv11 * v1)


def read_residuals(path):
    # residuals.csv: step,track_id,innov_x,innov_y,S00,S01,S10,S11
    by_track = defaultdict(list)
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            step = int(row["step"])
            tid = int(row["track_id"])
            ix = float(row["innov_x"])
            iy = float(row["innov_y"])
            S00 = float(row["S00"])
            S01 = float(row["S01"])
            S10 = float(row["S10"])
            S11 = float(row["S11"])
            nis = nis_2d(ix, iy, S00, S01, S10, S11)
            # ignore rows with zero S (uninitialized) or nan
            if not np.isfinite(nis):
                continue
            by_track[tid].append((step, nis))
    for tid in list(by_track.keys()):
        by_track[tid].sort(key=lambda t: t[0])
    return by_track


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="indir", required=True, help="Run output directory containing residuals.csv")
    ap.add_argument("--out", dest="outpath", default=None, help="Output PNG path (default: plots/nis_<indir>.png)")
    ap.add_argument("--title", dest="title", default=None, help="Plot title override")
    args = ap.parse_args()

    indir = args.indir
    resid_path = os.path.join(indir, "residuals.csv")
    if not os.path.isfile(resid_path):
        raise SystemExit(f"Missing: {resid_path}")

    by_track = read_residuals(resid_path)

    base = os.path.basename(os.path.normpath(indir))
    outpath = args.outpath or os.path.join("plots", f"nis_{base}.png")
    os.makedirs(os.path.dirname(outpath), exist_ok=True)

    plt.figure()
    for tid in sorted(by_track.keys()):
        data = by_track[tid]
        steps = [p[0] for p in data]
        nis = [p[1] for p in data]
        plt.plot(steps, nis, linewidth=1.2, label=f"track {tid}")

    plt.xlabel("step")
    plt.ylabel("NIS (v^T S^{-1} v)")
    title = args.title or f"NIS per Track ({base})"
    plt.title(title)
    plt.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(outpath, dpi=160)
    print(f"Wrote: {outpath}")


if __name__ == "__main__":
    main()