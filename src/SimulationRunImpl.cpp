#include <drone_mapper/SimulationRunImpl.h>

#include <drone_mapper/MapsComparison.h>
#include <iostream>
#include <drone_mapper/Logger.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     types::DroneConfigData drone_config,
                                     types::LidarConfigData lidar_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      drone_config_(std::move(drone_config)),
      lidar_config_(std::move(lidar_config)),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ ||
        !output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !drone_control_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    types::SimulationResult sim_result{};
    try {
        // Run the mission control for this simulation
        types::MissionRunResult mission_result = mission_control_->runMission();

        sim_result.simulation_config = simulation_config_;
        sim_result.mission_config = mission_config_;
        sim_result.drone_config = drone_config_;
        sim_result.lidar_config = lidar_config_;
        sim_result.resolution_request_status = mission_config_.output_mapping_resolution_factor < 1.0
            ? types::ResolutionRequestStatus::IgnoredTooSmall
            : types::ResolutionRequestStatus::Accepted;
        sim_result.mission_results.push_back(mission_result);
        sim_result.output_map_file = output_map_file_;
        sim_result.output_map_config = output_map_->getMapConfig();

        // If mission had errors, mark a failure score
        if (mission_result.status == types::MissionRunStatus::Error) {
            sim_result.mission_score = -1.0;
            return sim_result;
        }

        // Compare maps (hidden vs output) to compute a score.
        auto scores = MapsComparison::compare(*hidden_map_, {output_map_.get()});
        if (!scores.empty()) {
            sim_result.mission_score = scores.front();
        } else {
            sim_result.mission_score = 0.0;
        }
    } catch (const std::exception& ex) {
        const std::string msg = std::string("SIMULATION_RUN_EXCEPTION: ") + ex.what();
        std::cerr << msg << std::endl;
        drone_mapper::Logger::logError("SIMULATION_RUN_EXCEPTION", ex.what());
        sim_result.mission_score = -1.0;
        types::MissionRunResult mr;
        mr.status = types::MissionRunStatus::Error;
        mr.errors.push_back(types::ErrorRef{"SIMULATION_RUN_EXCEPTION", ex.what()});
        sim_result.mission_results.push_back(mr);
    }

    return sim_result;
}

} // namespace drone_mapper
