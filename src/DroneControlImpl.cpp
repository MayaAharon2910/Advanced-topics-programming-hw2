#include <drone_mapper/DroneControlImpl.h>

#include <utility>

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

        // Let the mapping algorithm inspect the latest scan and produce the next move.
        types::MovementCommand cmd = mapping_algorithm_.nextMove(state, scan);

        // Execute movement according to the returned command.
        types::MovementResult res{true, {}};
        switch (cmd.type) {
            case types::MovementCommandType::Hover:
                // Nothing to do
                break;
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
            std::cerr << "DRONE_HITS_OBSTACLE: movement failed: " << res.message << std::endl;
            return types::DroneStepResult{types::DroneStepStatus::Error, res.message};
        }

        // Successful step
        ++step_index_;
        return types::DroneStepResult{types::DroneStepStatus::Continue, ""};
    } catch (const std::exception& ex) {
        std::cerr << "DRONE_CONTROL_EXCEPTION: " << ex.what() << std::endl;
        return types::DroneStepResult{types::DroneStepStatus::Error, ex.what()};
    }
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
