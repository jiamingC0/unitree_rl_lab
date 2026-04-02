#!/usr/bin/env python3
"""Plot left ankle PR/AB diagnostic traces."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot left ankle diagnostic CSV.")
    parser.add_argument("--csv", required=True, help="Path to left_ankle_diag.csv")
    parser.add_argument("--output-dir", required=True, help="Directory for plot outputs")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with csv_path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        return

    time = [float(r["time_s"]) for r in rows]
    target_pitch = [float(r["target_pitch"]) for r in rows]
    actual_pitch = [float(r["actual_pitch"]) for r in rows]
    target_roll = [float(r["target_roll"]) for r in rows]
    actual_roll = [float(r["actual_roll"]) for r in rows]
    dq_pitch = [float(r["dq_pitch"]) for r in rows]
    dq_roll = [float(r["dq_roll"]) for r in rows]

    plt.figure(figsize=(10, 4))
    plt.plot(time, target_pitch, label="target_pitch", linewidth=1.2)
    plt.plot(time, actual_pitch, label="actual_pitch", linewidth=1.0)
    plt.plot(time, target_roll, label="target_roll", linewidth=1.2)
    plt.plot(time, actual_roll, label="actual_roll", linewidth=1.0)
    plt.xlabel("time (s)")
    plt.ylabel("joint position (rad)")
    plt.title("Left ankle position diagnostic")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "left_ankle_position.png", dpi=150)
    plt.close()

    plt.figure(figsize=(10, 4))
    plt.plot(time, dq_pitch, label="dq_pitch", linewidth=1.0)
    plt.plot(time, dq_roll, label="dq_roll", linewidth=1.0)
    plt.xlabel("time (s)")
    plt.ylabel("joint velocity (rad/s)")
    plt.title("Left ankle velocity diagnostic")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "left_ankle_velocity.png", dpi=150)
    plt.close()


if __name__ == "__main__":
    main()
