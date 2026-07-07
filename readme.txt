== Drone Mapper HW2 — Documentation ==

Contributors:
  Maya Aharon, ID: 324126630
  Itay Ebenspanger, ID: 322532698

A 3-D autonomous drone mapping simulator for Assignment 2. The drone explores an
unknown voxel space using a Hybrid Exploration Strategy (Adaptive Sweep + BFS)
combined with Targeted Scans for blind spots. Multiple configurations are run as a
Cartesian product and scored against the ground-truth map.

--------------------------------------------------------------------------------

== Build ==

Prerequisites: CMake >= 3.20, C++20 compiler, vcpkg with mp-units, yaml-cpp,
tinynpy, and GTest installed in the course container.

Run all commands from the project root — relative paths to inputs/, data_maps/,
and other scenario directories are resolved from CWD.

  cmake --preset default
  cmake --build --preset default -j 2

For Release mode (recommended for large maps, 3-5x faster):

  cmake --preset default -DCMAKE_BUILD_TYPE=Release
  cmake --build --preset default -j 2

--------------------------------------------------------------------------------

== Run ==

  ./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]

- If simulation.yaml is omitted, the program reads simulation.yaml from CWD.
- If output_path is omitted, output is written to CWD.
- simulation.yaml may be a filename only, a relative path, or an absolute path.

All paths inside a composition file are resolved relative to that composition
file's directory, including map_filename inside simulation configs.

Input files:

  simulation_config.yaml        Map file path, resolution, initial drone position
  mission_config.yaml           Boundaries, max steps, GPS resolution, res factor
  drone_config.yaml             Sphere diameter (dimensions_cm), max rotation/advance/elevate
  lidar_config.yaml             z_min, z_max, beam spacing (d_cm), FOV circles
  simulation_compositions.yaml  Cartesian product: simulations x missions x drones x lidars

Map files are binary .npy files (NumPy format). Dimensions order: X, Y, Z.
Non-zero values indicate occupied voxels.

--------------------------------------------------------------------------------

== Output Files ==

  simulation_output.yaml              Score report for all runs (schema below)
  output_results/output_map_N.npy     Reconstructed voxel map for run N
  output_results/error_log.txt        All errors, written immediately when they occur

--------------------------------------------------------------------------------

== Getting a Score ==

The integration tests cap some large-map scenarios to 200 steps to keep CI runs fast.
Those capped scenarios are intended to validate end-to-end wiring, output creation,
and error handling under a strict runtime budget. Full manual runs via the main
executable use the scenario configuration and can explore the complete map.

Quick (~30 sec) — HW1 Case 2, narrow corridor, 100/100 in HW1 and HW2:

  ./build/drone_mapper_simulation hw1_scenarios/composition_case2.yaml output_case2/
  grep "score:" output_case2/simulation_output.yaml

HW1 Case 4 — large open-plan validation scenario for manual/extended evaluation:

  ./build/drone_mapper_simulation hw1_scenarios/composition_case4.yaml output_case4/
  grep "score:" output_case4/simulation_output.yaml

Full staff composition — 20 runs, intended for manual/extended evaluation:

  ./build/drone_mapper_simulation inputs/sim_compose.yaml output/
  grep "score:" output/simulation_output.yaml

The house map missions (house_mission_full.yaml, max_steps=10000) produce the
most meaningful score. The smaller maps (small_simulation_*.yaml) are a faster
alternative.

--------------------------------------------------------------------------------

== Maps Comparison Utility ==

Prints a single float 0-100 to stdout. In case of error prints -1 to stdout
and a descriptive message to stderr.

  ./build/maps_comparison <origin.npy> <target.npy>
  ./build/maps_comparison <origin.npy> <target.npy> comparison_config=<path.yaml>

Working example (run simulation first, then compare):

  ./build/drone_mapper_simulation hw1_scenarios/composition_case2.yaml output_case2/
  ./build/maps_comparison hw1_scenarios/maps/hw1_case2.npy \
      output_case2/output_results/output_map_0.npy

--------------------------------------------------------------------------------

== Tests ==

  ./build/drone_mapper_simulation_test                          # all 104 tests
  ./build/drone_mapper_simulation_test --gtest_filter=Integration.*
  ./build/drone_mapper_simulation_test --gtest_filter=SimulationManager.*
  ./build/drone_mapper_simulation_test --gtest_filter=SimulationRun.*
  ./build/drone_mapper_simulation_test --gtest_filter=MissionControl.*
  ./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
  ./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
  ./build/drone_mapper_simulation_test --gtest_filter=MockLidar.*
  ./build/drone_mapper_simulation_test --gtest_filter=MapsComparison.*

Note: Integration tests that use large maps cap max_steps to 200 for speed.
These tests validate the flow and generated artifacts under CI constraints; run
the main executable manually for uncapped scoring.

--------------------------------------------------------------------------------

== Output File Formats ==

--- simulation_output.yaml ---

Root key: score_report

  score_report:
    composition_file: <path to composition yaml>
    generated_at_utc: <ISO-8601 UTC timestamp>
    metric: "output_map_accuracy"
    score_range:
      min: 0
      max: 100
      error_score: -1
    summary:
      total_runs:    <int>
      scored_runs:   <int>
      error_runs:    <int>
      average_score: <float>
      min_score:     <float>
      max_score:     <float>
    simulations:
      - simulation_config: <path>
        missions:
          - mission_config: <path>
            resolution_cm: <float>
            resolution_request_status: ACCEPTED | IGNORED | IGNORED TOO SMALL
            runs:
              - drone_config: <path>
                lidar_config: <path>
                status: completed | max_steps | error
                steps: <int>
                score: <float>           # -1.0 on error
                output_map_file: <path>  # path to .npy
                error_ref:               # only present on error
                  code: <string>
                  message: <string>

Run ordering: Cartesian product (drone_configs x lidar_configs) in the order
they appear in the composition file.

--- output_results/ folder ---

  output_results/
    output_map_0.npy    # run 0 output map
    output_map_1.npy    # run 1 output map
    ...
    error_log.txt       # all errors, written immediately (no buffering)

Naming: output_map_N.npy where N is the global run index (0-based, across all
missions and simulations in the composition).

error_log.txt format:
  [ERROR] <CODE>: <message>
  One line per error. Written immediately when the error occurs.

--------------------------------------------------------------------------------

== BONUS: Visualise a Map ==

Requirements: sudo apt-get install -y python3-numpy python3-matplotlib

Headless devcontainer (no display) — saves PNG to current directory:

  python3 visualize_map.py data_maps/benchmark_map.npy --save
  python3 visualize_map.py output_results/output_map_0.npy --save

--------------------------------------------------------------------------------

== External Libraries ==

  mp-units 2.5.0   Strong physical types (XLength, YLength, ZLength, HorizontalAngle)
                   Prevents unit confusion at compile time
  yaml-cpp         Parsing all YAML configuration files
  tinynpy          Reading and writing NumPy .npy binary map files
  GTest / GMock    Unit and integration testing framework

All dependencies resolved through the course vcpkg installation.
Nothing is downloaded by CMake at build time.
