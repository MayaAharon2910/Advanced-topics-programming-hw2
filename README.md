# Assignment 2 Refactor Skeleton

This repository is a compilable skeleton for Assignment 2 in the 2026 Advanced
Topics in Programming course. It intentionally provides interfaces, data types,
dependency-injected component stubs, and a preserved mock LiDAR implementation.
It **does not** implement the full simulator or mapping solution. You should not use ANY implementations provided in this repository (aside MockLidar).

## Project Structure

```text
include/drone_mapper/      Public interfaces, data types, and skeleton classes
src/                     Stub implementations and executable entry points
data_maps/               Example NumPy voxel maps
.devcontainer/           Development container setup
CMakeLists.txt           CMake build configuration
vcpkg.json               Dependency list
```


## Building

```bash
cmake --preset default
cmake --build --preset default
```

The main build targets are:

```text
drone_mapper
drone_mapper_simulation
maps_comparison
```

## Running

Simulator skeleton:

```bash
./build/drone_mapper_simulation [simulation.yaml] [output_path]
```

The skeleton wires explicit placeholder components and reports stub results.
You should add YAML parsing, scenario composition, output writing, error
logging, and real simulator behavior etc..

**Output Files and Directory Structure**

- **simulation_output.yaml**: written at the root of the provided `<output_path>` (or current working directory when omitted). This YAML file contains a `simulations` sequence with one entry per simulation. Each simulation entry contains `missions`, each mission contains `runs`, and each run node contains the following keys:
	- **drone_config**: object with `dimensions_cm`, `max_rotation_deg`, `max_advance_cm`, `max_elevate_cm`.
	- **lidar_config**: object with `z_min_cm`, `z_max_cm`, `d_cm`, `fov_circles`.
	- **status**: string, either `Completed` or `Error`.
	- **steps**: integer, number of steps executed.
	- **score**: numeric mission score.
	- **error_ref** (optional): object with `code` and `message` when the run failed.

- **output_results/**: a directory created directly under the provided `<output_path>` (i.e. `<output_path>/output_results`). All generated map files (`.npy`) and the error log file (`error_log.txt`) are placed exclusively inside this folder. The simulator will create `output_results` automatically and overwrite existing files without prompting. Example layout:

```
<output_path>/simulation_output.yaml
<output_path>/output_results/
		output_map_0.npy
		output_map_1.npy
		error_log.txt
		scenario_0/
				additional_files.npy
```

Notes:
- When no `<simulation.yaml>` is provided, the program looks for `simulation.yaml` in the current working directory.
- Paths passed as arguments follow these rules: absolute paths (starting with `/`) are used verbatim; relative paths and filenames are resolved against the current working directory.

Maps comparison skeleton:

```bash
./build/maps_comparison <map1_path> <map2_path> [resolution_ratio=<res1>/<res2>]
```

The provided `MapsComparison` implementation is only a placeholder. You
should replace it with the required scoring behavior.
