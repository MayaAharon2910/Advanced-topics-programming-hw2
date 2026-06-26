See README.md for full documentation.

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
