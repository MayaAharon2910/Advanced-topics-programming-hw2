See README.md for full documentation.

== simulation_output.yaml format ==
Follows the score_report structure from the assignment spec:
  score_report → summary (total/scored/error runs, avg/min/max score)
               → simulations → missions → runs (drone, lidar, status, steps, score, error_ref)

== output_results/ folder ==
  output_results/
    output_map_<N>.npy   — output map for run N (same order as score_report runs)
    error_log.txt        — all errors logged immediately as they occur

== hw1_scenarios/ ==
Two hw1 edge-case scenarios as standard YAML+NPY files, runnable manually:
  ./drone_mapper_simulation hw1_scenarios/composition_case2.yaml
  ./drone_mapper_simulation hw1_scenarios/composition_case4.yaml
