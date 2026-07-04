See README.md for full documentation.

== Running the Program ==

=== Full simulation run (all scenarios) ===

  ./drone_mapper_simulation inputs/sim_compose.yaml output/

This runs the full Cartesian product of all simulations × missions × drones × lidars
defined in sim_compose.yaml (currently 20 runs) and writes:
  output/simulation_output.yaml   — scores and run metadata
  output/output_results/          — output maps (.npy) and error log

=== Single quick scenario (house map, full 10 000 steps) ===

  mkdir -p output_house
  ./drone_mapper_simulation inputs/sim_compose.yaml output_house/

The house scenarios (house_mission_full.yaml, max_steps=10 000) will run to
completion and produce a meaningful mapping score.

=== Maps comparison standalone utility ===

  ./maps_comparison <origin_map.npy> <target_map.npy>
  # prints a single float score 0–100 to stdout

  ./maps_comparison <origin.npy> <target.npy> comparison_config=<path.yaml>
  # uses explicit boundary/resolution config

=== Running tests ===

  ./drone_mapper_simulation_test                           # all 104 tests
  ./drone_mapper_simulation_test --gtest_filter=Integration.*
  ./drone_mapper_simulation_test --gtest_filter=SimulationManager.*
  ./drone_mapper_simulation_test --gtest_filter=SimulationRun.*
  ./drone_mapper_simulation_test --gtest_filter=MissionControl.*
  ./drone_mapper_simulation_test --gtest_filter=DroneControl.*
  ./drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
  ./drone_mapper_simulation_test --gtest_filter=MockLidar.*
  ./drone_mapper_simulation_test --gtest_filter=MapsComparison.*

NOTE on integration test scores:
  The Integration tests that use the house map (HouseScenarioMapLoadsWithSemanticValues,
  FullCompositionRunsWithoutCrashing) intentionally cap max_steps to 200 so the test
  suite finishes in seconds. With only 200 steps the drone cannot cover the house map,
  so the score will be 0 — this is expected and does NOT indicate a bug. Run the program
  manually (above) to get meaningful scores with the full 10 000 step budget.

== Output File Formats — Assignment 2 ==

=== simulation_output.yaml ===

Root key: score_report

Structure:
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
            resolution_request_status: ACCEPTED | IGNORED | IGNORED_TOO_SMALL
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

Run ordering: runs within each mission follow the Cartesian product order
  (drone_configs × lidar_configs), in the order they appear in the composition file.

=== output_results/ folder ===

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

=== hw1_scenarios/ ===

Two scenarios converted from Assignment 1 edge cases, runnable as:
  ./drone_mapper_simulation hw1_scenarios/composition_case2.yaml
  ./drone_mapper_simulation hw1_scenarios/composition_case4.yaml

=== complex_scenario/ ===

Staff benchmark map scenario (29x30x31 house map). Run as:
  ./drone_mapper_simulation complex_scenario/composition.yaml
Produces 4 runs (1 sim x 1 mission x 4 drones x 1 lidar).
