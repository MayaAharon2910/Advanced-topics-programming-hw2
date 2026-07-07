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
    sudo apt-get install -y python3-numpy python3-matplotlib

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
import os
import matplotlib
# Use non-interactive Agg backend in headless environments (no DISPLAY/WAYLAND_DISPLAY)
if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    matplotlib.use("Agg")
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


def _load_map_raw(path: pathlib.Path) -> np.ndarray:
    """Fallback loader for .npy files whose header dtype string numpy's own
    np.load() rejects (e.g. tinynpy-written output maps use a non-standard
    descr like '<?1' for what is really a plain int8 array). Parses the
    standard .npy header by hand and reinterprets the payload as int8, since
    every map this tool produces or consumes is a 3-D int8 voxel grid."""
    with open(path, "rb") as f:
        magic = f.read(6)
        if magic != b"\x93NUMPY":
            raise ValueError(f"Not a .npy file (bad magic): {path}")
        major, _minor = f.read(1), f.read(1)
        header_len_size = 2 if major == b"\x01" else 4
        header_len = int.from_bytes(f.read(header_len_size), "little")
        header = f.read(header_len).decode("latin1")
        raw = f.read()
    shape_str = header.split("'shape':")[1].split(")")[0].split("(")[1]
    shape = tuple(int(x) for x in shape_str.split(",") if x.strip())
    return np.frombuffer(raw, dtype=np.int8).reshape(shape)


def load_map(path: pathlib.Path) -> np.ndarray:
    try:
        data = np.load(path)
    except (ValueError, TypeError):
        # Malformed dtype descriptor from tinynpy-written output maps.
        data = _load_map_raw(path)
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
        "--threshold", type=int, default=1,
        help="Show voxels with value >= threshold (default: 1)"
    )
    parser.add_argument(
        "--save", action="store_true",
        help="Save to <map_file>.png instead of opening an interactive window"
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

    # --save or headless environment: write PNG instead of opening a window
    if args.save or not _has_display():
        out = pathlib.Path(args.map_file).stem + ".png"
        plt.savefig(out, dpi=150)
        print(f"  Saved to: {out}")
    else:
        plt.show()


def _has_display() -> bool:
    """Return True if a graphical display is available."""
    import os
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


if __name__ == "__main__":
    main()
