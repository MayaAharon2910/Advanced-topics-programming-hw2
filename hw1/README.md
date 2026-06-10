# Drone Mapper - HW1


## Contributors

- Name: Maya Aharon, ID: 324126630
- Name: Itay Ebenspanger, ID: 322532698

A 3-D autonomous drone mapping simulator. The drone explores an unknown voxel space using a **Hybrid Exploration Strategy (Adaptive Sweep + BFS)** combined with **Targeted Scans** for blind spots. It writes a reconstructed occupancy map at the end of the mission.

---

## Build Instructions

Prerequisites: CMake >= 3.20, a C++20-capable compiler, and `vcpkg` available in the course container (with `mp-units` version 2.5.0 and `GTest` installed locally).

### 1. Configure

Run from the `hw1/` directory. This command configures the project and enables the building of the test suite:

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=/usr/local/vcpkg/scripts/buildsystems/vcpkg.cmake -DBUILD_TESTING=ON
```

### 2. Build

```bash
cmake --build build
```

---

## Run Instructions

```bash
./build/drone_mapper [<input_output_files_path>]
```

- If `<input_output_files_path>` is omitted, the program uses the current working directory.
- The path must point to an existing directory containing the required input files.
- Output files are written back to the same directory.

### Required Input Files

| File | Required? | Purpose |
|---|---:|---|
| `drone_config.txt` | Yes | Drone movement and LiDAR limits: max rotation, advance, elevation, field-of-view, etc. |
| `mission_config.txt` | Yes | Mission boundaries, starting position, starting orientation, resolution and recharge positions. |
| `map_input.txt` | Yes | Ground-truth voxel map used only by the simulator sensors and scoring code. |
| `simulation_config.txt` | No | Optional simulator/runtime configuration. Controls logging and debug observability only. |

The prepared edge-case folders already contain ready-to-run input files. No conversion script is required before running `drone_mapper` on them.

### Map File Format

`map_input.txt` is a plain-text file. The first line contains the map dimensions:

```txt
width height depth
```

The following lines contain the voxel values separated by whitespace. `map_output.txt` is written in the same plain-text format so it can be compared directly with the input map.

### Output Files

| File | Purpose |
|---|---|
| `map_output.txt` | Reconstructed occupancy map produced by the drone. |
| `score.txt` | Final mapping accuracy score and final pose. |
| `simulation_log.txt` | Timestamped structured simulator log. Written by default unless the optional simulator configuration disables it. |
| `input_errors.txt` | Created only when recoverable input parsing issues are found; absent on clean input. |

---

## Examples

```bash
# Run directly on any directory containing ready-to-run input files
./build/drone_mapper /path/to/input/directory

# Run a specific prepared edge case directly
./build/drone_mapper edge_cases/case1

# Run all prepared edge cases through the Python runner
python3 edge_cases/run_all_edge_cases.py

# Run a single prepared edge case through the Python runner
python3 edge_cases/run_all_edge_cases.py --case case1

# Run GTest
cd build
ctest --output-on-failure
```
---

## External library usage

The main code uses the course-approved `mp-units` library, version 2.5.0, for strong types of centimeters and degrees. Unit tests use GTest when `BUILD_TESTING=ON`. No external libraries are downloaded by CMake; dependencies are resolved through the course vcpkg installation.
