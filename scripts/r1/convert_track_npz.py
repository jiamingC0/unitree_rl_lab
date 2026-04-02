#!/usr/bin/env python3
"""Convert an R1 tracking NPZ file into a raw binary cache for deployment.

The cache stores all arrays from the source NPZ without preprocessing.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np


MAGIC = b"R1TRK01\x00"
CACHE_VERSION = 4

DTYPE_CODES = {
    np.dtype(np.float32): 1,
    np.dtype(np.float64): 2,
    np.dtype(np.bool_): 3,
    np.dtype(np.int32): 4,
    np.dtype(np.int64): 5,
    np.dtype(np.uint8): 6,
}


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
    arrays = {key: np.ascontiguousarray(np.asarray(data[key])) for key in data.files}
    if "qpos" not in arrays or "kpt2gv_pose" not in arrays or "kpt_cvel_in_gv" not in arrays:
        raise ValueError("Expected at least qpos, kpt2gv_pose, and kpt_cvel_in_gv in track npz")

    dst.parent.mkdir(parents=True, exist_ok=True)
    with dst.open("wb") as f:
        f.write(struct.pack("<8sII", MAGIC, CACHE_VERSION, len(arrays)))
        for name, arr in arrays.items():
            dtype = arr.dtype
            if dtype not in DTYPE_CODES:
                raise ValueError(f"Unsupported dtype for {name}: {dtype}")
            name_bytes = name.encode("utf-8")
            shape = arr.shape
            f.write(struct.pack("<I", len(name_bytes)))
            f.write(name_bytes)
            f.write(struct.pack("<II", DTYPE_CODES[dtype], len(shape)))
            if shape:
                f.write(struct.pack("<" + "I" * len(shape), *shape))
            f.write(struct.pack("<Q", arr.nbytes))
            f.write(arr.tobytes(order="C"))


if __name__ == "__main__":
    main()
