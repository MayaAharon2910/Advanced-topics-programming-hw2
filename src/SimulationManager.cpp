#include <drone_mapper/SimulationManager.h>

#include <stdexcept>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <drone_mapper/Logger.h>

namespace drone_mapper {

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;

    // Prepare output_results directory and logger
    const auto output_results = output_path / "output_results";
    std::filesystem::create_directories(output_results);
    drone_mapper::Logger::setOutputDirectory(output_results);

    YAML::Node root;
    YAML::Node sims_node = YAML::Node(YAML::NodeType::Sequence);

    for (const types::SimulationConfigData& simulation : composition.simulations) {
        YAML::Node sim_node;
        sim_node["simulation"] = simulation.map_filename.string();
        YAML::Node missions_node = YAML::Node(YAML::NodeType::Sequence);

        for (const types::MissionConfigData& mission : composition.missions) {
            YAML::Node mission_node;
            mission_node["mission"] = static_cast<int>(mission.max_steps);
            YAML::Node runs_node = YAML::Node(YAML::NodeType::Sequence);

            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    std::unique_ptr<ISimulationRun> run =
                        run_factory_->create(simulation, mission, drone, lidar, output_path);
                    types::SimulationResult sim_res = run->run();
                    runs.push_back(sim_res);

                    YAML::Node run_node;
                    // Drone config
                    YAML::Node drone_node;
                    drone_node["dimensions_cm"] = drone.dimensions.numerical_value_in(cm);
                    drone_node["max_rotation_deg"] = drone.max_rotate.numerical_value_in(deg);
                    drone_node["max_advance_cm"] = drone.max_advance.numerical_value_in(cm);
                    drone_node["max_elevate_cm"] = drone.max_elevate.numerical_value_in(cm);
                    run_node["drone_config"] = drone_node;

                    // Lidar config
                    YAML::Node lidar_node;
                    lidar_node["z_min_cm"] = lidar.z_min.numerical_value_in(cm);
                    lidar_node["z_max_cm"] = lidar.z_max.numerical_value_in(cm);
                    lidar_node["d_cm"] = lidar.d.numerical_value_in(cm);
                    lidar_node["fov_circles"] = static_cast<int>(lidar.fov_circles);
                    run_node["lidar_config"] = lidar_node;

                    // Outcome
                    double score = sim_res.mission_score;
                    const auto& mr = sim_res.mission_results.empty() ? types::MissionRunResult{} : sim_res.mission_results.front();
                    run_node["status"] = (mr.status == types::MissionRunStatus::Completed) ? "Completed" : "Error";
                    run_node["steps"] = static_cast<int>(mr.steps);
                    run_node["score"] = score;

                    if (mr.status == types::MissionRunStatus::Error && !mr.errors.empty()) {
                        YAML::Node err_node;
                        err_node["code"] = mr.errors.front().code;
                        err_node["message"] = mr.errors.front().message;
                        run_node["error_ref"] = err_node;
                    }

                    runs_node.push_back(run_node);
                }
            }

            mission_node["runs"] = runs_node;
            missions_node.push_back(mission_node);
        }

        sim_node["missions"] = missions_node;
        sims_node.push_back(sim_node);
    }

    root["simulations"] = sims_node;

    // Write simulation_output.yaml at output_path (not inside output_results)
    const auto yaml_file = output_path / "simulation_output.yaml";
    try {
        std::ofstream out(yaml_file);
        out << root;
    } catch (const std::exception& ex) {
        drone_mapper::Logger::logError("SIMULATION_OUTPUT_WRITE_FAILED", ex.what());
    }

    return types::SimulationManagerReport{"stub", "stub", {}, -1, std::move(runs)};
}

} // namespace drone_mapper
