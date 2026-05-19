#!/usr/bin/env python3
"""Generate one plot per tested ET1 JointTest joint."""

from __future__ import annotations

import argparse
import csv
import os
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REQUIRED_COLUMNS = (
    "time_s",
    "round",
    "joint_id",
    "joint_name",
    "sdk_slot",
    "phase",
    "q_cmd",
    "q_actual",
    "dq_actual",
    "lower",
    "upper",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot every joint found in an ET1 JointTest CSV.")
    parser.add_argument("--csv", required=True, help="Path to joint_test_*.csv")
    parser.add_argument(
        "--output-dir",
        help="Directory for PNG outputs. Defaults to <csv_stem>_plots beside the CSV.",
    )
    parser.add_argument(
        "--sample-dt",
        type=float,
        default=0.002,
        help="Fallback sample period when the CSV time_s column has been rounded flat.",
    )
    return parser.parse_args()


def is_complete(row: Dict[str, str]) -> bool:
    return all(row.get(column) not in (None, "") for column in REQUIRED_COLUMNS)


def load_rows(csv_path: Path) -> Tuple[Dict[Tuple[int, str], List[Dict[str, str]]], int]:
    rows_by_joint: Dict[Tuple[int, str], List[Dict[str, str]]] = defaultdict(list)
    skipped = 0

    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        missing_columns = [column for column in REQUIRED_COLUMNS if column not in (reader.fieldnames or [])]
        if missing_columns:
            raise RuntimeError(f"Missing required columns: {', '.join(missing_columns)}")

        for row in reader:
            if not is_complete(row):
                skipped += 1
                continue
            joint_id = int(row["joint_id"])
            joint_name = row["joint_name"]
            rows_by_joint[(joint_id, joint_name)].append(row)

    return rows_by_joint, skipped


def time_axis(rows: List[Dict[str, str]], sample_dt: float) -> Tuple[List[float], bool]:
    raw_time = [float(row["time_s"]) for row in rows]
    t0 = raw_time[0]
    time = [value - t0 for value in raw_time]
    if max(time) - min(time) <= 1e-9 and len(time) > 1:
        return [index * sample_dt for index in range(len(rows))], True
    return time, False


def plot_joint(
    output_dir: Path,
    joint_id: int,
    joint_name: str,
    rows: List[Dict[str, str]],
    sample_dt: float,
) -> Tuple[Path, bool]:
    time, used_fallback_time = time_axis(rows, sample_dt)
    q_cmd = [float(row["q_cmd"]) for row in rows]
    q_actual = [float(row["q_actual"]) for row in rows]
    dq_actual = [float(row["dq_actual"]) for row in rows]
    error = [actual - cmd for actual, cmd in zip(q_actual, q_cmd)]
    lower = float(rows[0]["lower"])
    upper = float(rows[0]["upper"])
    sdk_slot = rows[0]["sdk_slot"]

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(time, q_cmd, label="q_cmd", linewidth=1.4)
    axes[0].plot(time, q_actual, label="q_actual", linewidth=1.0)
    axes[0].axhline(lower, color="tab:red", linestyle="--", linewidth=0.8, label="lower")
    axes[0].axhline(upper, color="tab:green", linestyle="--", linewidth=0.8, label="upper")
    axes[0].set_ylabel("position (rad)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time, error, color="tab:orange", linewidth=1.0)
    axes[1].set_ylabel("actual - cmd (rad)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(time, dq_actual, color="tab:purple", linewidth=1.0)
    axes[2].set_ylabel("dq_actual (rad/s)")
    axes[2].set_xlabel("time from joint start (s)")
    axes[2].grid(True, alpha=0.3)

    fig.suptitle(
        f"ET1 JointTest {joint_id}: {joint_name}  sdk_slot={sdk_slot}  "
        f"samples={len(rows)}  range=[{lower}, {upper}]"
    )
    fig.tight_layout()

    output_path = output_dir / f"joint_{joint_id:02d}_{joint_name}.png"
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path, used_fallback_time


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv)
    output_dir = Path(args.output_dir) if args.output_dir else csv_path.parent / f"{csv_path.stem}_plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    rows_by_joint, skipped = load_rows(csv_path)
    if not rows_by_joint:
        print(f"No complete JointTest rows found in {csv_path}")
        return

    print(f"Found {len(rows_by_joint)} tested joint(s):")
    used_any_fallback_time = False
    for (joint_id, joint_name), rows in sorted(rows_by_joint.items()):
        output_path, used_fallback_time = plot_joint(output_dir, joint_id, joint_name, rows, args.sample_dt)
        used_any_fallback_time = used_any_fallback_time or used_fallback_time
        print(f"  {joint_id:02d} {joint_name}: {len(rows)} samples -> {output_path}")

    if skipped:
        print(f"Skipped {skipped} incomplete row(s).")
    if used_any_fallback_time:
        print(f"time_s was flat for at least one joint; used sample index * {args.sample_dt:g}s as x-axis.")


if __name__ == "__main__":
    main()
