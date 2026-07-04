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
cmake --build --preset default -j 2
```

For Release mode (recommended for large maps — 3–5× faster):

```bash
cmake --preset default -DCMAKE_BUILD_TYPE=Release
cmake --build --preset default -j 2
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

All paths inside a composition file are resolved relative to that composition file's directory,
including `simulation_config` and `mission_config` references, and `map_filename` inside a
simulation config (e.g. `map/scenario.npy` is resolved relative to the composition file's folder).

### Map File Format

Map input files are binary `.npy` files as produced by NumPy. The first file dimension is X, second is Y, third is Z. Non-zero values indicate occupied voxels.

### Output Files

| File | Purpose |
|---|---|
| `simulation_output.yaml` | Score report for all runs (see `readme.txt` for full schema) |
| `output_results/output_map_N.npy` | Reconstructed voxel map for run N |
| `output_results/error_log.txt` | All errors, written immediately when they occur |

### A note on integration test scores

Some integration tests (e.g. `HouseScenarioMapLoadsWithSemanticValues`) cap `max_steps` to 200
to keep the test suite fast. With only 200 steps the drone cannot cover a large map, so those
tests may report a score of 0 — this is **expected and intentional**, not a bug. Run the program
manually with the full composition file to get meaningful scores (the house missions use 10 000 steps).

---

## Examples

### Build

```bash
cmake --preset default
cmake --build --preset default -j 2
```

### Run the simulation

```bash
# Full staff composition — 20 runs (may take 30–60 min)
./build/drone_mapper_simulation inputs/sim_compose.yaml output/
grep "score:" output/simulation_output.yaml

# Default: reads simulation.yaml from CWD, writes output to CWD
./build/drone_mapper_simulation
```

### Get a meaningful score quickly

```bash
# HW1 Case 2 — narrow corridor (~30 sec, 100/100 in HW1 and HW2)
./build/drone_mapper_simulation hw1_scenarios/composition_case2.yaml output_case2/
grep "score:" output_case2/simulation_output.yaml

# HW1 Case 4 — large open plan (~2 min, 98/100 in HW1, -1 in HW2 due to stricter collision detection)
./build/drone_mapper_simulation hw1_scenarios/composition_case4.yaml output_case4/
grep "score:" output_case4/simulation_output.yaml
```

### Maps comparison utility

```bash
# Prints a single float 0–100 to stdout.
# First run the simulation to generate an output map, then compare:
./build/drone_mapper_simulation hw1_scenarios/composition_case2.yaml output_case2/
./build/maps_comparison hw1_scenarios/maps/hw1_case2.npy output_case2/output_results/output_map_0.npy
# Expected output: ~82.66

# With optional boundary/resolution config (if maps have different offsets or resolutions):
# ./build/maps_comparison <origin.npy> <target.npy> comparison_config=<path_to_yaml>
```

### Run tests

```bash
# All 104 tests
./build/drone_mapper_simulation_test

# By suite
./build/drone_mapper_simulation_test --gtest_filter=Integration.*
./build/drone_mapper_simulation_test --gtest_filter=SimulationManager.*
./build/drone_mapper_simulation_test --gtest_filter=SimulationRun.*
./build/drone_mapper_simulation_test --gtest_filter=MissionControl.*
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
./build/drone_mapper_simulation_test --gtest_filter=MockLidar.*
./build/drone_mapper_simulation_test --gtest_filter=MapsComparison.*
```

### BONUS: Visualise a map

```bash
# Requirements: sudo apt-get install -y python3-numpy python3-matplotlib
python3 visualize_map.py data_maps/benchmark_map.npy --save        # saves benchmark_map.png
python3 visualize_map.py output_results/output_map_0.npy --save    # saves output_map_0.png
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

---

## Getting a Meaningful Mapping Score

The mapping score reflects how much of the hidden map the drone successfully explored.
To get a real score the drone must run on an actual map — the integration tests cap `max_steps`
to 200 for speed, which is not enough to cover large maps and will produce a score of 0.

**Quick option — HW1 edge-case scenarios (seconds to ~2 min):**

```bash
# Case 2 — narrow corridor, completes in ~30 sec, 100/100 in both HW1 and HW2
./build/drone_mapper_simulation hw1_scenarios/composition_case2.yaml output_case2/ && grep "score:" output_case2/simulation_output.yaml

# Case 4 — large open plan; scored 98/100 in HW1 but returns -1 in HW2 due to
# stricter collision detection (drone hits a wall the output map hadn't yet revealed)
./build/drone_mapper_simulation hw1_scenarios/composition_case4.yaml output_case4/ && grep "score:" output_case4/simulation_output.yaml
```

**Full staff scenarios — 20 runs, may take 30–60 min total:**

```bash
./build/drone_mapper_simulation inputs/sim_compose.yaml output/ && grep "score:" output/simulation_output.yaml
```

The house map missions (`house_mission_full.yaml`) run up to 10 000 steps and produce the most
meaningful score, but are also the slowest. The smaller maps (`small_simulation_*.yaml`) finish
in a few minutes and are a good middle ground.
