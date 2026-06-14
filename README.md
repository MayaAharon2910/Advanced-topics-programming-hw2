Contributors:

- Name: Maya Aharon, ID: 324126630
- Name: Itay Ebenspanger, ID: 322532698

# Assignment 2 Refactor Skeleton

This repository is a compilable skeleton for Assignment 2 in the 2026 Advanced
Topics in Programming course. It intentionally provides interfaces, data types,
dependency-injected component stubs, and a preserved mock LiDAR implementation.
It implements the Assignment 2 simulator flow, YAML parsing, score report output, tests, and the mapping algorithm ported from HW1-style sweep/BFS frontier exploration.

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

The executable parses the Assignment 2 simulation composition YAML, runs the Cartesian product of simulations/missions/drones/lidars, writes maps and immediate error logs under `output_results`, and writes the score report at the output root.

Execution commands (assignment targets):

- Simulator: `./build/drone_mapper_simulation [simulation.yaml] [output_path]`
- Maps comparison: `./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]`

`simulation_output.yaml` schema:

- Root key: `score_report`.
- Metadata: `composition_file`, `generated_at_utc`, `metric: output_map_accuracy`, and `score_range` with `min`, `max`, and `error_score`.
- Summary: `total_runs`, `scored_runs`, `error_runs`, `average_score`, `min_score`, and `max_score`.
- Results: `simulations`, each containing `missions`, `resolution_cm`, `resolution_request_status`, and nested `runs` with `status`, `steps`, `score`, optional `output_map_file`, and optional `error_ref`.

Output folder layout (exact rules):

- The program MUST write `simulation_output.yaml` at the root of the provided `<output_path>` (or current working dir if omitted).
- The program MUST create a directory named `output_results` directly under `<output_path>`.
- All generated `.npy` map files and the `error_log.txt` MUST be placed inside `output_results` (sub-folders allowed). Existing files are overwritten without prompting.

Submission preparation warning:

- Before creating the final ZIP for submission, remove the `build/` directory and any `.DS_Store` files. Including build artifacts or OS-generated files may cause zero-grading by the automated grader.

**Output Files and Directory Structure**

- **simulation_output.yaml**: written at the root of the provided `<output_path>` (or current working directory when omitted). The root is `score_report`, with metadata, summary statistics, and a hierarchical list of simulations/missions/runs.

- **output_results/**: a directory created directly under the provided `<output_path>` (i.e. `<output_path>/output_results`). All generated map files (`.npy`) and the error log file (`error_log.txt`) are placed inside this folder. Existing files are overwritten without prompting. Example layout:

```
<output_path>/simulation_output.yaml
<output_path>/output_results/
        output_map_0.npy
        output_map_1.npy
        error_log.txt
```

Notes:
- When no `<simulation.yaml>` is provided, the program looks for `simulation.yaml` in the current working directory.
- Paths passed as arguments follow these rules: absolute paths (starting with `/`) are used verbatim; relative paths and filenames are resolved against the current working directory.

Maps comparison skeleton:

```bash
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

When the optional comparison config is provided, it must contain `comparison_config.original` and `comparison_config.target` entries with `map_res_cm`, `map_offset`, and `map_boundaries`. The program prints only the numeric score to stdout, or `-1` on stdout plus an error on stderr.
