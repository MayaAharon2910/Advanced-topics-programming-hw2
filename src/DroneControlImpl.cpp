#include <drone_mapper/DroneControlImpl.h>

#include <utility>
#include <vector>
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
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    try {
        // Build current state snapshot
        types::DroneState state = this->state();

        // Perform a scan using the lidar. Use a zero-relative scan orientation
        // (the mapping algorithm expects relative angles in its own logic).
        types::LidarScanResult scan = lidar_.scan(Orientation{0.0 * deg, 0.0 * deg});
        std::vector<types::MappedVoxel> observed_voxels =
            ScanResultToVoxels::convert(state.position, state.heading, scan);
        for (const auto& voxel : observed_voxels) {
            output_map_.set(voxel.position, voxel.value);
        }
        mapping_algorithm_.applyVoxelUpdates(observed_voxels);

        // Let the mapping algorithm inspect the latest scan and produce the next move.
        types::MovementCommand cmd = mapping_algorithm_.nextMove(state, scan);

        // Execute movement according to the returned command.
        types::MovementResult res{true, {}};
        switch (cmd.type) {
            case types::MovementCommandType::Hover:
                // A Hover command is used by the mapping algorithm to signal that
                // exploration is complete. Returning Completed lets MissionControl
                // finish naturally instead of waiting until max_steps.
                ++step_index_;
                return types::DroneStepResult{types::DroneStepStatus::Completed, ""};
            case types::MovementCommandType::Rotate: {
                res = movement_.rotate(cmd.rotation, cmd.angle);
                break;
            }
            case types::MovementCommandType::Advance: {
                res = movement_.advance(cmd.distance);
                break;
            }
            case types::MovementCommandType::Elevate: {
                res = movement_.elevate(cmd.distance);
                break;
            }
        }

        if (!res) {
            // Immediate error logging and propagate an error step result.
            drone_mapper::Logger::logError("DRONE_HITS_OBSTACLE", res.message);
            return types::DroneStepResult{types::DroneStepStatus::Error, res.message};
        }

        // Successful step
        ++step_index_;
        return types::DroneStepResult{types::DroneStepStatus::Continue, ""};
    } catch (const std::exception& ex) {
        drone_mapper::Logger::logError("DRONE_CONTROL_EXCEPTION", ex.what());
        return types::DroneStepResult{types::DroneStepStatus::Error, ex.what()};
    }
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
