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

RECOMMENDED: build in Release mode for real runs. Debug builds are 3-5x slower
and can make large-map / long max_steps scenarios take noticeably longer to
finish. Always use Release when measuring scores or timing:

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

== Anatomy of the Input Configs ==

The simulator's only CLI input is a composition file; it references the other
four config types by relative path. Real, working examples already in the repo:

--- inputs/simulation/small_simulation_out.yaml (simulation_config) ---

  simulation_config:
    map_filename: "map/scenario_small.npy"
    map_resolution_cm: 10
    initial_drone_position:
      x_cm: 150
      y_cm: 150
      height_cm: 110
    initial_angle_deg: 0    # 0=east, 90=south, 180=west, 270=north
    map_axes_offset:
      x_offset: 0
      y_offset: 0
      height_offset: 0

--- inputs/mission/small_mission_out.yaml (mission_config) ---

  mission_config:
    max_steps: 2000
    boundaries:
      x_boundary:      { min_cm: 0, max_cm: 200 }
      y_boundary:      { min_cm: 0, max_cm: 200 }
      height_boundary: { min_cm: 0, max_cm: 200 }
    gps_resolution_cm: 5

--- inputs/drone/drone_small.yaml (drone_config) ---

  drone_config:
    dimensions_cm:  8      # sphere diameter drone can pass through
    max_rotate_deg: 90
    max_advance_cm: 30
    max_elevate_cm: 20

--- inputs/lidar/lidar_long.yaml (lidar_config) ---

  lidar_config:
    z_min_cm: 20
    z_max_cm: 150
    d_cm: 2.5
    fov_circles: 3

--- hw1_scenarios/composition_case2.yaml (the composition tying them together) ---

  simulation_compositions:
    simulations:
      - simulation_config: "simulations/case2.yaml"
        mission_configs:
          - "missions/case2/full_corridor.yaml"
          - "missions/case2/left_half.yaml"
    drone_configs:
      - "drones/agile.yaml"
    lidar_configs:
      - "lidars/short_range.yaml"
      - "lidars/long_range.yaml"

Run it (verified working, 4 runs, ~99-100 score each):

  ./build/drone_mapper_simulation hw1_scenarios/composition_case2.yaml /tmp/example_run
  grep "score:" /tmp/example_run/simulation_output.yaml

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

Full staff composition — 24 runs, intended for manual/extended evaluation:

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

Scoring design: strict 4-state exact match. A cell only counts as a match if
the output map's state (Unmapped / Empty / Occupied / PotentiallyOccupied)
exactly equals the hidden map's state at that voxel - Unmapped is never
treated as equivalent to Empty. This is a deliberate choice to comply with
the assignment FAQ ("You should actively map all cells in the required
resolution" - no interpolating or assuming blind spots are empty). A looser
formula that credited Unmapped cells whenever the hidden truth happens to be
Empty would let the score reflect how empty the map naturally is rather than
how much the drone actually verified. Our score is therefore a strict lower
bound: it counts only cells the drone actively confirmed, never space it
assumed.

--------------------------------------------------------------------------------

== Tests ==

  ./build/drone_mapper_simulation_test                          # all 111 tests
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
