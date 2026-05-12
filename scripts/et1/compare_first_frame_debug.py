#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np


OBS_CURRENT_TERMS = [
    ("command_root_ori_b", 6),
    ("command_xy_yaw_vel", 3),
    ("command_jnt_pos", 26),
    ("projected_gravity", 3),
    ("base_ang_vel", 3),
    ("joint_pos_rel", 26),
    ("joint_vel_rel", 26),
    ("last_action", 26),
]


def max_abs(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.max(np.abs(a.astype(np.float64) - b.astype(np.float64))))


def report_array(name: str, train: np.ndarray, deploy: np.ndarray) -> None:
    if train.shape != deploy.shape:
        print(f"{name}: shape mismatch train={train.shape} deploy={deploy.shape}")
        return
    print(
        f"{name}: max_abs={max_abs(train, deploy):.8f} "
        f"train[min,max]=({train.min():.6f},{train.max():.6f}) "
        f"deploy[min,max]=({deploy.min():.6f},{deploy.max():.6f})"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--train-npz", required=True, type=Path)
    parser.add_argument("--deploy-dir", required=True, type=Path)
    args = parser.parse_args()

    train = np.load(args.train_npz)

    for name in ["obs_current", "obs_history", "action", "target_joint_pos"]:
        deploy_path = args.deploy_dir / f"{name}.npy"
        if not deploy_path.exists():
            print(f"{name}: missing deploy file {deploy_path}")
            continue
        report_array(name, train[name], np.load(deploy_path))

    if (args.deploy_dir / "obs_current.npy").exists():
        train_obs = train["obs_current"][0]
        deploy_obs = np.load(args.deploy_dir / "obs_current.npy")[0]
        if train_obs.shape == deploy_obs.shape:
            offset = 0
            print("\nobs_current terms:")
            for term, size in OBS_CURRENT_TERMS:
                sl = slice(offset, offset + size)
                print(f"  {term:22s} max_abs={max_abs(train_obs[sl], deploy_obs[sl]):.8f}")
                offset += size

    for name in [
        "deploy_joint_pos",
        "deploy_joint_vel",
        "deploy_root_quat_w",
        "lowcmd_q_idl",
        "lowcmd_kp_idl",
        "lowcmd_kd_idl",
        "lowcmd_mode_idl",
    ]:
        path = args.deploy_dir / f"{name}.npy"
        if path.exists():
            arr = np.load(path)
            print(f"{name}: shape={arr.shape} min={arr.min():.6f} max={arr.max():.6f}")


if __name__ == "__main__":
    main()
