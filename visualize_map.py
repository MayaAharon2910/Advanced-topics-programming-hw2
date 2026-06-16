#!/usr/bin/env python3
"""
visualize_map.py — Visual Simulation Bonus Utility
====================================================
Loads a 3-D voxel map saved as a NumPy .npy file and renders it
interactively using Matplotlib.  Works with maps produced by the
drone_mapper_simulation and with the original input maps.

Usage
-----
    python3 visualize_map.py <map_file.npy> [--threshold 1]

Arguments
---------
  map_file      Path to a .npy file (3-D int8 array).
  --threshold   Voxel value that counts as "occupied" (default: 1).
                Use 0 to show all non-negative voxels.

Requirements
------------
    pip install numpy matplotlib

Examples
--------
    # Visualise the input (hidden) map
    python3 visualize_map.py data_maps/single_voxel_x2_y4_z2.npy

    # Visualise a generated output map
    python3 visualize_map.py output_results/output_map_0.npy

    # Show anything that is not Unmapped (-1) or OutOfBounds (-2)
    python3 visualize_map.py output_results/output_map_0.npy --threshold 0
"""

import sys
import argparse
import pathlib

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3-D projection)


# ---------------------------------------------------------------------------
# Voxel value legend (matches VoxelOccupancy in the C++ code)
# ---------------------------------------------------------------------------
VOXEL_LABELS = {
    -3: ("PotentiallyOccupied", "orange"),
    -2: ("OutOfBounds",         "gray"),
    -1: ("Unmapped",            "lightblue"),
     0: ("Empty",               "whitesmoke"),
     1: ("Occupied",            "darkred"),
}


def load_map(path: pathlib.Path) -> np.ndarray:
    data = np.load(path)
    if data.ndim != 3:
        raise ValueError(f"Expected a 3-D array, got shape {data.shape}")
    return data.astype(np.int8)


def build_voxel_grid(data: np.ndarray, threshold: int):
    """Return a boolean mask of voxels to render and their colours."""
    filled = data >= threshold
    colours = np.empty(data.shape, dtype=object)
    colours[:] = VOXEL_LABELS.get(threshold, ("Custom", "steelblue"))[1]

    # Colour each voxel by its exact value so the plot is informative
    for value, (_, colour) in VOXEL_LABELS.items():
        colours[data == value] = colour

    # Only keep colours where filled is True
    face_colours = np.empty(filled.shape, dtype=object)
    face_colours[filled] = colours[filled]
    return filled, face_colours


def main():
    parser = argparse.ArgumentParser(
        description="3-D voxel visualiser for drone_mapper_simulation .npy maps",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("map_file", type=pathlib.Path, help="Path to .npy map file")
    parser.add_argument(
        "--threshold",
        type=int,
        default=1,
        help="Minimum voxel value to display (default: 1 = Occupied only)",
    )
    args = parser.parse_args()

    if not args.map_file.exists():
        print(f"ERROR: file not found: {args.map_file}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading map: {args.map_file}")
    data = load_map(args.map_file)
    print(f"  Shape (X, Y, Z): {data.shape}")
    print(f"  Unique values  : {np.unique(data).tolist()}")
    occupied = int(np.sum(data >= args.threshold))
    print(f"  Voxels shown   : {occupied}  (threshold >= {args.threshold})")

    filled, face_colours = build_voxel_grid(data, args.threshold)

    if not filled.any():
        print("No voxels meet the threshold — nothing to render.")
        sys.exit(0)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.voxels(filled, facecolors=face_colours, edgecolor="k", linewidth=0.2, alpha=0.85)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(
        f"Drone Mapper — {args.map_file.name}\n"
        f"shape={data.shape}  shown={occupied} voxels  (threshold≥{args.threshold})"
    )

    # Legend
    legend_handles = [
        matplotlib.patches.Patch(
            facecolor=colour,
            edgecolor="black",
            label=f"{value}: {label}",
        )
        for value, (label, colour) in VOXEL_LABELS.items()
        if np.any(data == value)
    ]
    ax.legend(handles=legend_handles, loc="upper left", fontsize=8)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
