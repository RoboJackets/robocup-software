"""Export trained actor weights to a simple binary format for C++.

The C++ RLPosition class reads this binary format directly, avoiding
any dependency on numpy or .npz parsing in the robot software stack.

Binary layout:
    uint32  num_layers
    For each layer:
        uint32  rows   (fan_in)
        uint32  cols   (fan_out)
        float32 weights[rows * cols]   (row-major)
        float32 biases[cols]

Usage:
    python -m scripts.export_weights checkpoints/policy_final
    python -m scripts.export_weights checkpoints/policy_final -o weights.bin
"""
import argparse
import struct
import sys

import numpy as np


def export_actor(npz_path: str, output_path: str) -> None:
    data = np.load(npz_path)

    keys = sorted(data.files)
    weight_keys = sorted(k for k in keys if k.startswith("w"))
    bias_keys = sorted(k for k in keys if k.startswith("b"))
    num_layers = len(weight_keys)

    if num_layers == 0 or len(bias_keys) != num_layers:
        print(f"Error: expected matching w*/b* keys, got {keys}", file=sys.stderr)
        sys.exit(1)

    with open(output_path, "wb") as f:
        f.write(struct.pack("<I", num_layers))

        for i in range(num_layers):
            w = data[f"w{i}"].astype(np.float32)
            b = data[f"b{i}"].astype(np.float32)
            rows, cols = w.shape

            f.write(struct.pack("<II", rows, cols))
            f.write(w.tobytes())
            f.write(b.tobytes())

            print(f"  Layer {i}: {rows} x {cols}  "
                  f"({rows * cols + cols} params)")

    total_params = sum(
        data[f"w{i}"].size + data[f"b{i}"].size for i in range(num_layers)
    )
    print(f"Exported {num_layers} layers ({total_params} params) to {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export RL actor weights to C++ binary format."
    )
    parser.add_argument(
        "checkpoint",
        help="Path prefix for the checkpoint (e.g. checkpoints/policy_final). "
             "The script appends '_actor.npz' automatically.",
    )
    parser.add_argument(
        "-o", "--output",
        default=None,
        help="Output .bin file path. "
             "Defaults to <checkpoint>_actor.bin.",
    )
    args = parser.parse_args()

    npz_path = args.checkpoint + "_actor.npz"
    output_path = args.output or (args.checkpoint + "_actor.bin")

    print(f"Loading {npz_path} ...")
    export_actor(npz_path, output_path)


if __name__ == "__main__":
    main()
