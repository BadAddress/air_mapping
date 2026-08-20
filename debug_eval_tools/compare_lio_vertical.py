#!/usr/bin/env python3
"""Compare baseline and experimental LIO vertical drift diagnostics."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def quaternion_euler_deg(row, prefix):
    qx = float(row[prefix + "qx"])
    qy = float(row[prefix + "qy"])
    qz = float(row[prefix + "qz"])
    qw = float(row[prefix + "qw"])
    roll = math.atan2(2.0 * (qw * qx + qy * qz),
                      1.0 - 2.0 * (qx * qx + qy * qy))
    pitch_term = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def load_trajectory(path):
    with path.open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    distance = [0.0]
    for previous, current in zip(rows, rows[1:]):
        dx = float(current["raw_x"]) - float(previous["raw_x"])
        dy = float(current["raw_y"]) - float(previous["raw_y"])
        distance.append(distance[-1] + math.hypot(dx, dy))
    return {
        "distance": distance,
        "raw_z": [float(row["raw_z"]) for row in rows],
        "opt_z": [float(row["z"]) for row in rows],
        "raw_roll": [quaternion_euler_deg(row, "raw_")[0] for row in rows],
        "raw_pitch": [quaternion_euler_deg(row, "raw_")[1] for row in rows],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    baseline = load_trajectory(args.baseline)
    experiment = load_trajectory(args.experiment)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    figure, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    axes[0].plot(baseline["distance"], baseline["raw_z"],
                 label="baseline raw LIO Z", color="#d62728", alpha=0.85)
    axes[0].plot(experiment["distance"], experiment["raw_z"],
                 label="2-45 m raw LIO Z", color="#1f77b4", alpha=0.9)
    axes[0].plot(baseline["distance"], baseline["opt_z"],
                 label="baseline loop-opt Z", color="#ff9896", alpha=0.7)
    axes[0].plot(experiment["distance"], experiment["opt_z"],
                 label="2-45 m loop-opt Z", color="#9ecae1", alpha=0.8)
    axes[0].set_ylabel("Z in local map (m)")
    axes[0].set_title("Vertical trajectory comparison (same first 995 s)")
    axes[0].legend(ncol=2)

    axes[1].plot(baseline["distance"], baseline["raw_pitch"],
                 label="baseline", color="#d62728")
    axes[1].plot(experiment["distance"], experiment["raw_pitch"],
                 label="2-45 m LIO work set", color="#1f77b4")
    axes[1].set_ylabel("Raw LIO pitch (deg)")
    axes[1].legend()

    axes[2].plot(baseline["distance"], baseline["raw_roll"],
                 label="baseline", color="#d62728")
    axes[2].plot(experiment["distance"], experiment["raw_roll"],
                 label="2-45 m LIO work set", color="#1f77b4")
    axes[2].set_ylabel("Raw LIO roll (deg)")
    axes[2].set_xlabel("Cumulative horizontal keyframe distance (m)")
    axes[2].legend()

    for axis in axes:
        axis.grid(True, alpha=0.25)
    figure.tight_layout()
    figure.savefig(args.output, dpi=160)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
