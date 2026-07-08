#include <drone_mapper/DroneControlImpl.h>

#include <cmath>
#include <utility>
#include <drone_mapper/Logger.h>
#include <drone_mapper/ScanResultToVoxels.h>

namespace drone_mapper {

namespace {

constexpr double kMinMoveCm = 0.5;

[[nodiscard]] PhysicalLength clampAdvance(PhysicalLength distance, PhysicalLength max_distance) {
    double v = distance.force_numerical_value_in(cm);
    const double mx = max_distance.force_numerical_value_in(cm);
    if (v < 0.0) v = 0.0;
    if (v > mx) v = mx;
    return v * cm;
}

[[nodiscard]] PhysicalLength clampElevate(PhysicalLength distance, PhysicalLength max_distance) {
    double v = distance.force_numerical_value_in(cm);
    const double mx = max_distance.force_numerical_value_in(cm);
    if (v > mx) v = mx;
    if (v < -mx) v = -mx;
    return v * cm;
}

[[nodiscard]] HorizontalAngle clampAngle(HorizontalAngle angle, HorizontalAngle max_angle) {
    double v = angle.force_numerical_value_in(deg);
    const double mx = max_angle.force_numerical_value_in(deg);
    if (v < 0.0) v = 0.0;
    if (v > mx) v = mx;
    return v * horizontal_angle[deg];
}

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_config_{},          // default; caller MUST call setLidarConfig() before step()
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

void DroneControlImpl::setLidarConfig(const types::LidarConfigData& config) {
    lidar_config_ = config;
    lidar_config_set_ = true;
}

types::DroneStepResult DroneControlImpl::step() {
    if (!lidar_config_set_) {
        throw std::runtime_error("LiDAR config not set: call setLidarConfig() before step()");
    }
    try {
        const types::DroneState current_state = this->state();

        // Pass last scan result to the algorithm — null on first step.
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
                    res = movement_.rotate(move.rotation, clampAngle(move.angle, drone_.max_rotate));
                    break;
                case types::MovementCommandType::Advance:
                {
                    const PhysicalLength dist = clampAdvance(move.distance, drone_.max_advance);
                    res = movement_.advance(dist);
                    if (!res) {
                        const double half = dist.force_numerical_value_in(cm) / 2.0;
                        if (half > kMinMoveCm) {
                            res = movement_.advance(half * cm);
                        }
                    }
                    break;
                }
                case types::MovementCommandType::Elevate:
                {
                    const PhysicalLength dist = clampElevate(move.distance, drone_.max_elevate);
                    res = movement_.elevate(dist);
                    if (!res) {
                        const double half = dist.force_numerical_value_in(cm) / 2.0;
                        if (std::abs(half) > kMinMoveCm) {
                            res = movement_.elevate(half * cm);
                        }
                    }
                    break;
                }
            }
            if (!res) {
                Logger::logError("DRONE_HITS_OBSTACLE", res.message);
                return types::DroneStepResult{types::DroneStepStatus::Error, res.message};
            }
        }

        // --- Execute scan after movement (the API allows both in one step;
        //     movement must run first). The scan is fed to the algorithm on
        //     the NEXT nextStep call. ---
        if (cmd.scan_orientation.has_value()) {
            const types::LidarScanResult scan = lidar_.scan(*cmd.scan_orientation);
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
