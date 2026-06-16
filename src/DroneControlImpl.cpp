#include <drone_mapper/DroneControlImpl.h>

#include <utility>
#include <drone_mapper/Logger.h>
#include <drone_mapper/ScanResultToVoxels.h>

namespace drone_mapper {

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_config_(mapping_algorithm.getLidarConfig()),   // FIX 3: no ctor param
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    try {
        const types::DroneState current_state = this->state();

        // FIX 1: Pass last scan result to the algorithm so it is NOT blind.
        // latest_scan_ is null on the very first step (before any scan),
        // then updated every step from the previous scan.
        const types::LidarScanResult* scan_ptr =
            last_scan_.has_value() ? &last_scan_.value() : nullptr;

        types::MappingStepCommand cmd =
            mapping_algorithm_.nextStep(current_state, scan_ptr);

        // --- Execute movement ---
        if (cmd.movement.has_value()) {
            const types::MovementCommand& move = *cmd.movement;
            types::MovementResult res{true, {}};
            switch (move.type) {
                case types::MovementCommandType::Hover:
                    break;
                case types::MovementCommandType::Rotate:
                    res = movement_.rotate(move.rotation, move.angle);
                    break;
                case types::MovementCommandType::Advance:
                    res = movement_.advance(move.distance);
                    break;
                case types::MovementCommandType::Elevate:
                    res = movement_.elevate(move.distance);
                    break;
            }
            if (!res) {
                Logger::logError("DRONE_HITS_OBSTACLE", res.message);
                return types::DroneStepResult{types::DroneStepStatus::Error, res.message};
            }
        }

        // --- Execute scan and update map ---
        if (cmd.scan_orientation.has_value()) {
            const types::LidarScanResult scan = lidar_.scan(*cmd.scan_orientation);
            // Store result so next call can pass it to the algorithm (FIX 1)
            last_scan_ = scan;
            const types::DroneState post_move_state = this->state();
            ScanResultToVoxels::applyToMap(output_map_,
                                           post_move_state.position,
                                           post_move_state.heading,
                                           scan,
                                           lidar_config_);
        }

        ++step_index_;

        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            return types::DroneStepResult{types::DroneStepStatus::Completed, ""};
        }

        return types::DroneStepResult{types::DroneStepStatus::Continue, ""};
    } catch (const std::exception& ex) {
        Logger::logError("DRONE_CONTROL_EXCEPTION", ex.what());
        return types::DroneStepResult{types::DroneStepStatus::Error, ex.what()};
    }
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
