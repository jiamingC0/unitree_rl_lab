#!/usr/bin/env python3
"""Generate one plot per joint from the recorded R1 tracking CSV."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot desired vs actual joint tracking.")
    parser.add_argument("--csv", required=True, help="Path to joint_tracking.csv")
    parser.add_argument("--output-dir", required=True, help="Directory for PNG outputs")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        return

    columns = reader.fieldnames or []
    time = [float(row["time_s"]) for row in rows]

    target_cols = [c for c in columns if c.startswith("target_")]
    for target_col in target_cols:
        joint_name = target_col[len("target_") :]
        actual_col = f"actual_{joint_name}"
        if actual_col not in columns:
            continue

        target = [float(row[target_col]) for row in rows]
        actual = [float(row[actual_col]) for row in rows]

        plt.figure(figsize=(10, 4))
        plt.plot(time, target, label="target_q", linewidth=1.2)
        plt.plot(time, actual, label="actual_q", linewidth=1.0)
        plt.xlabel("time (s)")
        plt.ylabel("joint position (rad)")
        plt.title(joint_name)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output_dir / f"{joint_name}.png", dpi=150)
        plt.close()

        obs_rel_col = f"obs_joint_pos_rel_{joint_name}"
        ref_rel_col = f"ref_joint_pos_rel_{joint_name}"
        if obs_rel_col in columns and ref_rel_col in columns:
            obs_rel = [float(row[obs_rel_col]) for row in rows]
            ref_rel = [float(row[ref_rel_col]) for row in rows]

            plt.figure(figsize=(10, 4))
            plt.plot(time, ref_rel, label="ref_joint_pos_rel", linewidth=1.2)
            plt.plot(time, obs_rel, label="obs_joint_pos_rel", linewidth=1.0)
            plt.xlabel("time (s)")
            plt.ylabel("relative joint position (rad)")
            plt.title(f"{joint_name} relative tracking")
            plt.legend()
            plt.tight_layout()
            plt.savefig(output_dir / f"rel_{joint_name}.png", dpi=150)
            plt.close()


if __name__ == "__main__":
    main()
