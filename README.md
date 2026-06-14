# Drone Mapper - HW2


## Contributors

- Name: Maya Aharon, ID: 324126630
- Name: Itay Ebenspanger, ID: 322532698

A 3-D autonomous drone mapping simulator refactored for Assignment 2. The project keeps the HW1 **Hybrid Exploration Strategy (Adaptive Sweep + BFS / frontier exploration)** and adapts it to the Assignment 2 component architecture, YAML input files, NumPy map files, score report output, component tests, and integration tests.

---

## Build Instructions

Prerequisites: CMake >= 3.20, a C++20-capable compiler, and the official course container / toolchain.

The project includes the required `vcpkg.json`, `vcpkg-configuration.json`, and `CMakePresets.json`. The default preset uses the `VCPKG_ROOT` environment variable and the Ninja generator:

```text
$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake
```

CMake itself does not contain `FetchContent`, `git clone`, or any custom dependency download logic. Dependencies are resolved through the course-provided vcpkg manifest mode.

### 1. Configure

Run from the project root directory:

```bash
cmake --preset default
```

Equivalent explicit command:

```bash
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
```

### 2. Build

```bash
cmake --build --preset default -j 2
```

Equivalent explicit command:

```bash
cmake --build build -j 2
```

---

## Run Instructions

```bash
./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]
```

- If `<simulation.yaml>` is omitted, the program looks for `simulation.yaml` in the current working directory.
- If `<simulation.yaml>` contains only a filename, the file is resolved relative to the current working directory.
- If `<simulation.yaml>` is a relative path, it is resolved relative to the current working directory.
- If `<simulation.yaml>` is an absolute path, it is used as provided.
- If `<output_path>` is omitted, output files are written to the current working directory.
- Existing output files are overwritten.

### Required Input Files

| File | Required? | Purpose |
|---|---:|---|
| Simulation composition YAML | Yes | Lists simulations, mission configs, drone configs, and lidar configs. |
| Simulation config YAML | Yes | Defines `map_filename`, `map_resolution_cm`, `initial_drone_position`, `initial_angle_deg`, and `map_axes_offset`. |
| Mission config YAML | Yes | Defines `max_steps`, `boundaries`, `gps_resolution_cm`, and optional `output_mapping_resolution_factor`. |
| Drone config YAML | Yes | Defines `dimensions_cm`, `max_rotate_deg`, `max_advance_cm`, and `max_elevate_cm`. |
| Lidar config YAML | Yes | Defines `z_min_cm`, `z_max_cm`, `d_cm`, and `fov_circles`. |
| NumPy map file (`.npy`) | Yes | Ground-truth voxel map used by the simulation sensors and scoring code. |

### YAML File Format

The simulation composition file uses the Assignment 2 hierarchy:

```yaml
simulation_compositions:
  simulations:
    - simulation_config: "simulations/office_floor1_at_east.yaml"
      mission_configs:
        - "missions/office_floor1_east/mission_map_all_floor.yaml"
        - "missions/office_floor1_east/mission_map_east_part.yaml"
  drone_configs:
    - "drone_configs/drone_small.yaml"
    - "drone_configs/drone_large.yaml"
  lidar_configs:
    - "lidar_configs/lidar_a.yaml"
    - "lidar_configs/lidar_b.yaml"
```

The simulator runs the Cartesian product of each simulation's mission configs with all drone configs and lidar configs.

### Output Files

| File / Directory | Purpose |
|---|---|
| `simulation_output.yaml` | Hierarchical score report. The root key is `score_report`. |
| `output_results/` | Contains generated `.npy` output maps and immediate error logs. |
| `output_results/error_log.txt` | Created when recoverable errors occur. Errors are logged immediately when detected. |

The score report schema is:

```yaml
score_report:
  composition_file: "simulation.yaml"
  generated_at_utc: "2026-05-30T23:31:10Z"
  metric: "output_map_accuracy"
  score_range:
    min: 0
    max: 100
    error_score: -1
  summary:
    total_runs: 0
    scored_runs: 0
    error_runs: 0
    average_score: 0
    min_score: 0
    max_score: 0
  simulations: []
```

### Maps Comparison Utility

```bash
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

- `origin_map` and `target_map` are `.npy` files.
- The optional comparison config is a YAML file with `comparison_config.original` and `comparison_config.target` entries.
- Each map config may contain `map_res_cm`, `map_offset`, and `map_boundaries`.
- The program prints only the numeric score to standard output.
- On error, the program prints `-1` to standard output and a descriptive message to standard error.

---

## Examples

```bash
# Configure and build with the course vcpkg preset
cmake --preset default
cmake --build --preset default -j 2

# Run the simulator with default simulation.yaml in the current directory
./build/drone_mapper_simulation

# Run the simulator with explicit composition file and output directory
./build/drone_mapper_simulation configs/simulation.yaml out

# Run the maps comparison utility without a config file
./build/maps_comparison maps/original.npy output_results/output_map_0.npy

# Run the maps comparison utility with a comparison config file
./build/maps_comparison maps/original.npy output_results/output_map_0.npy comparison_config=configs/comparison.yaml

# Run all tests
./build/drone_mapper_simulation_test

# Run integration tests only
./build/drone_mapper_simulation_test --gtest_filter=Integration.*

# Run a specific component suite
./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
```

---

## External library usage

The main code uses the course-approved `mp-units` library for strong centimeter and degree types, `tinynpy` for reading and writing NumPy map files, and `yaml-cpp` for reading YAML configuration files. Unit tests use GTest and GMock. The vcpkg manifest files must stay in the submission because they identify the required packages for the course toolchain. CMake does not perform custom downloads; it relies on the official course vcpkg setup through `VCPKG_ROOT`.
