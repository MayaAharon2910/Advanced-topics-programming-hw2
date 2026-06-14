#include <drone_mapper/SimulationManager.h>

#include <stdexcept>
#include <sstream>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <drone_mapper/Logger.h>

namespace drone_mapper {
namespace {

[[nodiscard]] const char* missionStatusToString(types::MissionRunStatus status) {
    switch (status) {
        case types::MissionRunStatus::Completed: return "completed";
        case types::MissionRunStatus::MaxSteps: return "max_steps";
        case types::MissionRunStatus::Error: return "error";
    }
    return "error";
}

[[nodiscard]] const char* resolutionStatusToString(types::ResolutionRequestStatus status) {
    switch (status) {
        case types::ResolutionRequestStatus::Accepted: return "ACCEPTED";
        case types::ResolutionRequestStatus::Ignored: return "IGNORED";
        case types::ResolutionRequestStatus::IgnoredTooSmall: return "IGNORED_TOO_SMALL";
    }
    return "IGNORED";
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;
    std::ostringstream yaml_text;
    yaml_text << "simulations:\n";

    // Prepare output_results directory and logger
    const auto output_results = output_path / "output_results";
    std::filesystem::create_directories(output_results);
    drone_mapper::Logger::setOutputDirectory(output_results);

    for (const types::SimulationConfigData& simulation : composition.simulations) {
        yaml_text << "  - simulation: " << simulation.map_filename.string() << "\n";
        yaml_text << "    missions:\n";
        for (const types::MissionConfigData& mission : composition.missions) {
            std::string mission_resolution_status = "IGNORED";
            std::ostringstream mission_runs_text;
            mission_runs_text << "        runs:\n";

            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    std::unique_ptr<ISimulationRun> run =
                        run_factory_->create(simulation, mission, drone, lidar, output_path);
                    types::SimulationResult sim_res = run->run();
                    runs.push_back(sim_res);

                    // Outcome
                    double score = sim_res.mission_score;
                    const auto& mr = sim_res.mission_results.empty() ? types::MissionRunResult{} : sim_res.mission_results.front();
                    const std::string run_resolution_status =
                        resolutionStatusToString(sim_res.resolution_request_status);
                    if (mission_resolution_status == "IGNORED") {
                        mission_resolution_status = run_resolution_status;
                    }

                    mission_runs_text << "          - drone_config:\n";
                    mission_runs_text << "              dimensions_cm: " << drone.dimensions.numerical_value_in(cm) << "\n";
                    mission_runs_text << "              max_rotation_deg: " << drone.max_rotate.numerical_value_in(deg) << "\n";
                    mission_runs_text << "              max_advance_cm: " << drone.max_advance.numerical_value_in(cm) << "\n";
                    mission_runs_text << "              max_elevate_cm: " << drone.max_elevate.numerical_value_in(cm) << "\n";
                    mission_runs_text << "            lidar_config:\n";
                    mission_runs_text << "              z_min_cm: " << lidar.z_min.numerical_value_in(cm) << "\n";
                    mission_runs_text << "              z_max_cm: " << lidar.z_max.numerical_value_in(cm) << "\n";
                    mission_runs_text << "              d_cm: " << lidar.d.numerical_value_in(cm) << "\n";
                    mission_runs_text << "              fov_circles: " << static_cast<int>(lidar.fov_circles) << "\n";
                    mission_runs_text << "            status: " << missionStatusToString(mr.status) << "\n";
                    mission_runs_text << "            steps: " << static_cast<int>(mr.steps) << "\n";
                    mission_runs_text << "            score: " << score << "\n";
                    mission_runs_text << "            resolution_request_status: " << run_resolution_status << "\n";

                    if (mr.status == types::MissionRunStatus::Error && !mr.errors.empty()) {
                        mission_runs_text << "            error_ref:\n";
                        mission_runs_text << "              code: " << mr.errors.front().code << "\n";
                        mission_runs_text << "              message: " << mr.errors.front().message << "\n";
                    }
                }
            }

            yaml_text << "      - mission: " << static_cast<int>(mission.max_steps) << "\n";
            yaml_text << "        resolution_cm: " << mission.gps_resolution.numerical_value_in(cm) << "\n";
            yaml_text << "        resolution_request_status: " << mission_resolution_status << "\n";
            yaml_text << mission_runs_text.str();
        }
    }

    // Write simulation_output.yaml at output_path (not inside output_results)
    const auto yaml_file = output_path / "simulation_output.yaml";
    try {
        std::ofstream out(yaml_file);
        out << yaml_text.str();
    } catch (const std::exception& ex) {
        drone_mapper::Logger::logError("SIMULATION_OUTPUT_WRITE_FAILED", ex.what());
    }

    return types::SimulationManagerReport{"stub", "stub", {}, -1, std::move(runs)};
}

} // namespace drone_mapper
