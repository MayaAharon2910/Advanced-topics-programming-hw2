# Drone Mapper - HW2


## Contributors

- Name: Maya Aharon, ID: 324126630
- Name: Itay Ebenspanger, ID: 322532698

A 3-D autonomous drone mapping simulator for Assignment 2. The drone explores an unknown voxel space using a **Hybrid Exploration Strategy (Adaptive Sweep + BFS)** combined with **Targeted Scans** for blind spots. Multiple configurations are run as a Cartesian product and scored against the ground-truth map.

---

## Build Instructions

Prerequisites: CMake >= 3.20, a C++20-capable compiler, and `vcpkg` available in the course container (with `mp-units`, `yaml-cpp`, `tinynpy`, and `GTest` installed locally).

### 1. Configure and Build

Run all commands from the **project root directory** — relative paths to
`inputs/`, `data_maps/`, and other scenario directories are resolved from CWD.

```bash
cmake --preset default
cmake --build --preset default
```

For Release mode (recommended for large maps — 3–5× faster):

```bash
cmake --preset default -DCMAKE_BUILD_TYPE=Release
cmake --build --preset default
```

---

## Run Instructions

```bash
./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]
```

- If `<simulation.yaml>` is omitted, the program reads `simulation.yaml` from the current working directory.
- If `<output_path>` is omitted, output files are written to the current working directory.
- `<simulation.yaml>` may be a filename only, a relative path, or an absolute path.

### Required Input Files

The composition YAML file references one or more of the following per-component config files:

| File | Purpose |
|---|---|
| `simulation_config.yaml` | Map file path, resolution, initial drone position and heading |
| `mission_config.yaml` | Mission boundaries, max steps, GPS resolution, output resolution factor |
| `drone_config.yaml` | Drone sphere diameter (`dimensions_cm`), max rotation, max advance, max elevate |
| `lidar_config.yaml` | Lidar z_min, z_max, beam spacing (`d_cm`), number of FOV circles |
| `simulation_compositions.yaml` | Cartesian product: links simulations to their missions; lists drone and lidar configs |

All paths inside a composition file are resolved relative to that composition file's directory.
`map_filename` is resolved safely in this order: current working directory, composition YAML directory, then simulation YAML directory.

### Map File Format

Map input files are binary `.npy` files as produced by NumPy. The first file dimension is X, second is Y, third is Z. Non-zero values indicate occupied voxels.

### Output Files

| File | Purpose |
|---|---|
| `simulation_output.yaml` | Score report for all runs (see `readme.txt` for full schema) |
| `output_results/output_map_N.npy` | Reconstructed voxel map for run N |
| `output_results/error_log.txt` | All errors, written immediately when they occur |

---

## Examples

```bash
# Quickstart (run from project root)
cmake --preset default
cmake --build --preset default
./build/drone_mapper_simulation_test --gtest_filter=Integration.*
./build/drone_mapper_simulation inputs/sim_compose.yaml output
./build/maps_comparison origin_map.npy target_map.npy

# Default: reads simulation.yaml from CWD, writes output to CWD
./build/drone_mapper_simulation

# Explicit composition file and output directory
./build/drone_mapper_simulation configs/simulation.yaml output/

# Staff benchmark map (4 drone sizes × 1 mission = 4 runs)
./build/drone_mapper_simulation complex_scenario/composition.yaml

# hw1 edge-case scenarios (converted to hw2 YAML format)
./build/drone_mapper_simulation hw1_scenarios/composition_case2.yaml
./build/drone_mapper_simulation hw1_scenarios/composition_case4.yaml

# Maps comparison utility (prints a single score 0-100 to stdout)
./build/maps_comparison maps/original.npy output_results/output_map_0.npy
./build/maps_comparison maps/original.npy output_results/output_map_0.npy comparison_config=configs/comparison.yaml

# Run all tests
./build/drone_mapper_simulation_test

# Run integration tests only
./build/drone_mapper_simulation_test --gtest_filter=Integration.*

# Run a specific component suite
./build/drone_mapper_simulation_test --gtest_filter=MockLidar.*
./build/drone_mapper_simulation_test --gtest_filter=SimulationRun.*
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
./build/drone_mapper_simulation_test --gtest_filter=MissionControl.*
./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
./build/drone_mapper_simulation_test --gtest_filter=SimulationManager.*
./build/drone_mapper_simulation_test --gtest_filter=MapsComparison.*

# BONUS: Visualise any .npy map as a 3-D voxel diagram
# Requirements: sudo apt-get install -y python3-numpy python3-matplotlib
#
# Headless devcontainer (no display) — saves PNG to current directory:
python3 visualize_map.py data_maps/benchmark_map.npy --save
#   Output: ./benchmark_map.png  (open in VS Code Explorer)
#
# Visualise a generated output map (run simulator first):
python3 visualize_map.py output_results/output_map_0.npy --save
#   Output: ./output_map_0.png
```

---

## External Library Usage

| Library | Purpose |
|---|---|
| `mp-units` 2.5.0 | Strong physical types (`XLength`, `YLength`, `ZLength`, `PhysicalLength`, `HorizontalAngle`, etc.) — prevents unit confusion and raw-number bugs at compile time |
| `yaml-cpp` | Parsing all YAML configuration files |
| `tinynpy` | Reading and writing NumPy `.npy` binary map files |
| GTest / GMock | Unit and integration testing framework |

No external libraries are downloaded by CMake; all dependencies are resolved through the course vcpkg installation.
