#!/usr/bin/env python3
"""Convert an R1 tracking NPZ file into a compact binary cache for deployment.

The runtime controller only needs a subset of the original trajectory:
- 24 joint positions
- pelvis height in gv frame
- pelvis gravity direction in gv frame
- pelvis 6D velocity in gv frame
- yaw command as [cos(yaw), sin(yaw)]
- xy command in gv frame
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np


MAGIC = b"R1TRK01\x00"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert R1 track npz to runtime cache.")
    parser.add_argument("--input", required=True, help="Path to the source npz file.")
    parser.add_argument("--output", required=True, help="Path to the output .r1trk file.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    src = Path(args.input)
    dst = Path(args.output)

    data = np.load(src)
    qpos = np.asarray(data["qpos"], dtype=np.float32)
    kpt2gv_pose = np.asarray(data["kpt2gv_pose"], dtype=np.float32)
    kpt_cvel_in_gv = np.asarray(data["kpt_cvel_in_gv"], dtype=np.float32)

    if qpos.shape[1] != 31:
        raise ValueError(f"Expected qpos second dim to be 31, got {qpos.shape}")
    if kpt2gv_pose.shape[1:] != (14, 4, 4):
        raise ValueError(f"Unexpected kpt2gv_pose shape: {kpt2gv_pose.shape}")
    if kpt_cvel_in_gv.shape[1:] != (14, 6):
        raise ValueError(f"Unexpected kpt_cvel_in_gv shape: {kpt_cvel_in_gv.shape}")

    joint_pos = qpos[:, 7:31].astype(np.float32, copy=False)
    pelvis_pose = kpt2gv_pose[:, 0].astype(np.float32, copy=False)
    pelvis_cvel = kpt_cvel_in_gv[:, 0].astype(np.float32, copy=False)

    root_height = pelvis_pose[:, 2, 3].astype(np.float32, copy=False)
    root_gravity = (-pelvis_pose[:, 0:3, 2]).astype(np.float32, copy=False)

    yaw = np.arctan2(pelvis_pose[:, 1, 0], pelvis_pose[:, 0, 0]).astype(np.float32, copy=False)
    yaw_cmd = np.stack([np.cos(yaw), np.sin(yaw)], axis=1).astype(np.float32, copy=False)
    xy_cmd = pelvis_pose[:, 0:2, 3].astype(np.float32, copy=False)

    frame_count = int(joint_pos.shape[0])
    joint_dim = int(joint_pos.shape[1])

    dst.parent.mkdir(parents=True, exist_ok=True)
    with dst.open("wb") as f:
        f.write(struct.pack("<8sIII", MAGIC, 1, frame_count, joint_dim))
        f.write(joint_pos.tobytes(order="C"))
        f.write(root_height.tobytes(order="C"))
        f.write(root_gravity.tobytes(order="C"))
        f.write(pelvis_cvel.tobytes(order="C"))
        f.write(yaw_cmd.tobytes(order="C"))
        f.write(xy_cmd.tobytes(order="C"))


if __name__ == "__main__":
    main()
