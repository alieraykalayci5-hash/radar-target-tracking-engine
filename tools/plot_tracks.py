#!/usr/bin/env python3
import argparse
import csv
import os
from collections import defaultdict

import numpy as np
import matplotlib.pyplot as plt


def read_truth(path):
    # truth.csv: step,true_id,x,y,vx,vy
    by_id = defaultdict(list)
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            tid = int(row["true_id"])
            step = int(row["step"])
            x = float(row["x"])
            y = float(row["y"])
            by_id[tid].append((step, x, y))
    # sort by step
    for tid in list(by_id.keys()):
        by_id[tid].sort(key=lambda t: t[0])
    return by_id


def read_tracks(path):
    # tracks.csv header:
    # step,track_id,confirmed,x,y,vx,vy,misses,maha2,hits_window
    by_id = defaultdict(list)
    with open(path, "r", newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            tid = int(row["track_id"])
            step = int(row["step"])
            x = float(row["x"])
            y = float(row["y"])
            confirmed = int(row["confirmed"])
            misses = int(row["misses"])
            maha2 = float(row["maha2"])
            by_id[tid].append((step, x, y, confirmed, misses, maha2))
    for tid in list(by_id.keys()):
        by_id[tid].sort(key=lambda t: t[0])
    return by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="indir", required=True, help="Run output directory containing truth.csv and tracks.csv")
    ap.add_argument("--out", dest="outpath", default=None, help="Output PNG path (default: plots/tracks_<indir>.png)")
    ap.add_argument("--title", dest="title", default=None, help="Plot title override")
    args = ap.parse_args()

    indir = args.indir
    truth_path = os.path.join(indir, "truth.csv")
    tracks_path = os.path.join(indir, "tracks.csv")

    if not os.path.isfile(truth_path):
        raise SystemExit(f"Missing: {truth_path}")
    if not os.path.isfile(tracks_path):
        raise SystemExit(f"Missing: {tracks_path}")

    truth = read_truth(truth_path)
    tracks = read_tracks(tracks_path)

    # Choose output name
    base = os.path.basename(os.path.normpath(indir))
    outpath = args.outpath or os.path.join("plots", f"tracks_{base}.png")
    os.makedirs(os.path.dirname(outpath), exist_ok=True)

    plt.figure()
    # Plot truth trajectories
    for true_id in sorted(truth.keys()):
        if true_id == 0:
            continue
        data = truth[true_id]
        xs = [p[1] for p in data]
        ys = [p[2] for p in data]
        plt.plot(xs, ys, linewidth=2.0, label=f"truth {true_id}")

    # Plot track estimates
    for track_id in sorted(tracks.keys()):
        data = tracks[track_id]
        xs = [p[1] for p in data]
        ys = [p[2] for p in data]
        # Slightly thinner than truth so it overlays nicely
        plt.plot(xs, ys, linewidth=1.2, alpha=0.9, label=f"track {track_id}")

    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")

    title = args.title or f"Truth vs Track Estimates ({base})"
    plt.title(title)
    plt.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(outpath, dpi=160)
    print(f"Wrote: {outpath}")


if __name__ == "__main__":
    main()