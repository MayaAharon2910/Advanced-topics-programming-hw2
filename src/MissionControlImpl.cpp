#include <drone_mapper/MissionControlImpl.h>

#include <utility>
#include <iostream>
#include <drone_mapper/Logger.h>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    types::MissionRunResult result{};
    result.status = types::MissionRunStatus::Completed;
    std::size_t steps = 0;

    try {
        // Validate mission/output map boundaries before starting.
        const auto map_config = output_map_.getMapConfig();
        const auto& b = map_config.boundaries;
        bool invalid_bounds = false;
        if (b.min_x.numerical_value_in(cm) >= b.max_x.numerical_value_in(cm)) invalid_bounds = true;
        if (b.min_y.numerical_value_in(cm) >= b.max_y.numerical_value_in(cm)) invalid_bounds = true;
        if (b.min_height.numerical_value_in(cm) >= b.max_height.numerical_value_in(cm)) invalid_bounds = true;
        if (invalid_bounds) {
            std::cerr << "MISSION_BOUNDARY_INVALID: mission bounds are invalid" << std::endl;
            result.status = types::MissionRunStatus::Error;
            result.errors.push_back(types::ErrorRef{"MISSION_BOUNDARY_INVALID", "mission bounds are invalid"});
            return result;
        }
        // Run the mission by stepping the drone until it indicates completion
        // or the mission step limit is reached.
        for (std::size_t i = 0; i < mission_.max_steps; ++i) {
            auto step_res = drone_control_.step();
            ++steps;
                if (step_res.status == types::DroneStepStatus::Error) {
                    // Log and record error, but do not throw.
                    drone_mapper::Logger::logError("DRONE_STEP_ERROR", step_res.message);
                    result.status = types::MissionRunStatus::Error;
                    result.errors.push_back(types::ErrorRef{"DRONE_STEP_ERROR", step_res.message});
                    break;
                }

            if (step_res.status == types::DroneStepStatus::Completed) {
                break;
            }
        }

        result.steps = steps;
        // Save output map regardless of errors (best-effort)
        try {
            output_map_.save(output_map_file_);
        } catch (const std::exception& ex) {
            std::cerr << "MISSION_CONTROL: Failed to save output map: " << ex.what() << std::endl;
            result.errors.push_back(types::ErrorRef{"OUTPUT_MAP_SAVE_FAILED", ex.what()});
            result.status = types::MissionRunStatus::Error;
        }
        } catch (const std::exception& ex) {
        drone_mapper::Logger::logError("MISSION_CONTROL_EXCEPTION", ex.what());
        result.status = types::MissionRunStatus::Error;
        result.errors.push_back(types::ErrorRef{"MISSION_CONTROL_EXCEPTION", ex.what()});
    }

    return result;
}

} // namespace drone_mapper
