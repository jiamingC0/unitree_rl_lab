#!/usr/bin/env python3
"""Plot one joint's sequential sine diagnostic segment."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import List, Tuple

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot one sequential R1 joint sine diagnostic CSV.")
    parser.add_argument("--csv", required=True, help="Path to per-joint diagnostic CSV")
    parser.add_argument("--output-dir", required=True, help="Directory for plot outputs")
    parser.add_argument("--joint-name", required=True, help="Joint name for titles and output names")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv)
    output_dir = Path(args.output_dir)
    joint_name = args.joint_name
    output_dir.mkdir(parents=True, exist_ok=True)

    with csv_path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        return

    time = [float(r["local_time_s"]) for r in rows]
    target_q = [float(r["target_q"]) for r in rows]
    actual_q = [float(r["actual_q"]) for r in rows]
    actual_dq = [float(r["actual_dq"]) for r in rows]

    other_joint_columns = [
        key for key in rows[0].keys() if key.startswith("actual_q__") and key != f"actual_q__{joint_name}"
    ]
    coupled_series: List[Tuple[str, List[float], float]] = []
    for column in other_joint_columns:
        series = [float(r[column]) for r in rows]
        amplitude = max(series) - min(series) if series else 0.0
        coupled_series.append((column.removeprefix("actual_q__"), series, amplitude))
    coupled_series.sort(key=lambda item: item[2], reverse=True)
    top_coupled = coupled_series[:4]

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    axes[0].plot(time, target_q, label="target_q", linewidth=1.2)
    axes[0].plot(time, actual_q, label="actual_q", linewidth=1.0)
    axes[0].set_ylabel("joint position (rad)")
    axes[0].set_title(f"{joint_name} sine diagnostic")
    axes[0].legend()

    axes[1].plot(time, actual_dq, label="actual_dq", linewidth=1.0)
    axes[1].set_ylabel("joint velocity (rad/s)")
    axes[1].legend()

    if top_coupled:
        for coupled_name, series, amplitude in top_coupled:
            axes[2].plot(time, series, label=f"{coupled_name} ({amplitude:.3f})", linewidth=1.0)
        axes[2].set_ylabel("other joint q (rad)")
        axes[2].set_title("Largest coupled joint responses")
        axes[2].legend(fontsize=8)
    else:
        axes[2].text(0.5, 0.5, "No additional joint traces recorded", ha="center", va="center")
        axes[2].set_ylabel("other joint q (rad)")
    axes[2].set_xlabel("local test time (s)")

    fig.tight_layout()
    fig.savefig(output_dir / f"{joint_name}.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
