#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import numpy as np


TERMS = [
    "base_ang_vel",
    "projected_gravity",
    "joint_pos_rel",
    "joint_vel_rel",
    "target_q",
]


def load_csv(path: Path, terms):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    cols = rows[0].keys()
    out = {"time_s": np.array([float(r["time_s"]) for r in rows], dtype=np.float64)}
    for term in terms:
        term_cols = [c for c in cols if c.startswith(term + "_")]
        term_cols.sort(key=lambda c: int(c.rsplit("_", 1)[1]))
        out[term] = np.array([[float(r[c]) for c in term_cols] for r in rows], dtype=np.float64)
    return out


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    a = a.reshape(-1)
    b = b.reshape(-1)
    an = np.linalg.norm(a)
    bn = np.linalg.norm(b)
    if an < 1e-12 or bn < 1e-12:
        return 1.0 if an < 1e-12 and bn < 1e-12 else 0.0
    return float(np.dot(a, b) / (an * bn))


def summarize(a, b, term):
    diff = b - a
    return {
        "term": term,
        "rmse": float(np.sqrt(np.mean(diff * diff))),
        "mae": float(np.mean(np.abs(diff))),
        "max": float(np.max(np.abs(diff))),
        "cos": cosine_similarity(a, b),
    }


def main():
    parser = argparse.ArgumentParser(description="Compare Humanoid-GPT obs_compare target replay against local R1 replay.")
    parser.add_argument("--reference", required=True, help="Humanoid-GPT obs_compare.csv")
    parser.add_argument("--replay", required=True, help="Local R1 target_replay.csv")
    args = parser.parse_args()

    ref = load_csv(Path(args.reference), TERMS)
    replay = load_csv(Path(args.replay), TERMS)

    n = min(len(ref["time_s"]), len(replay["time_s"]))
    print(f"reference rows: {len(ref['time_s'])}")
    print(f"replay rows:    {len(replay['time_s'])}")
    print(f"aligned rows:   {n}")
    print()

    for key in ref:
        ref[key] = ref[key][:n]
        replay[key] = replay[key][:n]

    stats = [summarize(ref[t], replay[t], t) for t in TERMS]
    stats.sort(key=lambda s: s["rmse"], reverse=True)

    for s in stats:
        print(
            f"{s['term']:20s} rmse={s['rmse']:8.4f} "
            f"mae={s['mae']:8.4f} max={s['max']:8.4f} cos={s['cos']:7.4f}"
        )

    print("\nWorst joint dims:")
    for term in ["joint_pos_rel", "joint_vel_rel", "target_q"]:
        diff = np.abs(replay[term] - ref[term]).mean(axis=0)
        order = np.argsort(-diff)[:8]
        print(term)
        for idx in order:
            print(f"  dim {idx}: mean_abs={diff[idx]:.5f}")


if __name__ == "__main__":
    main()
