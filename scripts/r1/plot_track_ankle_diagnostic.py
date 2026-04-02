#!/usr/bin/env python3
"""Plot ankle diagnostic traces recorded from the R1 Track controller."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ANKLE_NAMES = [
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot ankle diagnostic CSV.")
    parser.add_argument("--csv", required=True, help="Path to ankle_diagnostic.csv")
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

    for joint_name in ANKLE_NAMES:
        obs_pos = [float(r[f"obs_joint_pos_rel_{joint_name}"]) for r in rows]
        obs_vel = [float(r[f"obs_joint_vel_{joint_name}"]) for r in rows]
        ref_pos = [float(r[f"ref_joint_pos_rel_{joint_name}"]) for r in rows]
        raw_action = [float(r[f"raw_action_{joint_name}"]) for r in rows]
        target_q = [float(r[f"target_q_{joint_name}"]) for r in rows]
        actual_q = [float(r[f"actual_q_{joint_name}"]) for r in rows]
        actual_dq = [float(r[f"actual_dq_{joint_name}"]) for r in rows]
        tau_cmd = [float(r[f"tau_cmd_{joint_name}"]) for r in rows]

        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

        axes[0].plot(time, ref_pos, label="ref_joint_pos_rel", linewidth=1.2)
        axes[0].plot(time, obs_pos, label="obs_joint_pos_rel", linewidth=1.0)
        axes[0].set_ylabel("rel pos (rad)")
        axes[0].legend()

        axes[1].plot(time, raw_action, label="raw_action", linewidth=1.0)
        axes[1].plot(time, target_q, label="target_q", linewidth=1.2)
        axes[1].plot(time, actual_q, label="actual_q", linewidth=1.0)
        axes[1].set_ylabel("angle (rad)")
        axes[1].legend()

        axes[2].plot(time, obs_vel, label="obs_joint_vel", linewidth=1.0)
        axes[2].plot(time, actual_dq, label="actual_dq", linewidth=1.0)
        axes[2].set_ylabel("vel (rad/s)")
        axes[2].legend()

        axes[3].plot(time, tau_cmd, label="tau_cmd", linewidth=1.0)
        axes[3].set_ylabel("tau")
        axes[3].set_xlabel("time (s)")
        axes[3].legend()

        fig.suptitle(joint_name)
        fig.tight_layout()
        fig.savefig(output_dir / f"ankle_diag_{joint_name}.png", dpi=150)
        plt.close(fig)


if __name__ == "__main__":
    main()
