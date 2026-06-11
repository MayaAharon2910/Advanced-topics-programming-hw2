#include <drone_mapper/MappingAlgorithmImpl.h>

#include <utility>
#include <vector>
#include <set>
#include <cmath>

#include <drone_mapper/Map3DImpl.h>
#include <mp-units/systems/si/math.h>

namespace drone_mapper {

MappingAlgorithmImpl::MappingAlgorithmImpl(types::MissionConfigData mission)
    : mission_(std::move(mission)) {}

// Small helpers adapted from hw1 Drone.cpp (fully-qualified types)
namespace {
struct Vec3 { double x; double y; double z; };

drone_mapper::Vec3 dirFromAngles(const drone_mapper::HorizontalAngle& ha, const drone_mapper::AltitudeAngle& aa) {
    const auto cos_alt = mp_units::si::cos(aa);
    const double dx = (cos_alt * mp_units::si::cos(ha)).force_numerical_value_in(mp_units::one);
    const double dy = (cos_alt * mp_units::si::sin(ha)).force_numerical_value_in(mp_units::one);
    const double dz = mp_units::si::sin(aa).force_numerical_value_in(mp_units::one);
    return {dx, dy, dz};
}

} // namespace

types::MovementCommand MappingAlgorithmImpl::nextMove(const types::DroneState& state,
                                               const types::LidarScanResult& latest_scan) {
    // Update pose
    current_position_ = state.position;
    orientation_ = state.heading;

    // Ensure internal map exists
    if (!internal_map_) {
        auto nptr = std::make_shared<NpyArray>();
        drone_mapper::types::MapConfig cfg = drone_mapper::types::MapConfig{};
        cfg.offset = drone_mapper::Position3D{};
        cfg.resolution = drone_mapper::PhysicalLength{1.0 * drone_mapper::cm};
        internal_map_ = std::make_unique<Map3DImpl>(nptr, cfg);
    }

    // Process beams: latest_scan contains relative orientations; convert to absolute
    for (const auto& hit : latest_scan) {
        drone_mapper::Orientation abs_or{
            drone_mapper::HorizontalAngle{orientation_.horizontal + hit.angle.horizontal},
            drone_mapper::AltitudeAngle{orientation_.altitude + hit.angle.altitude}
        };
        const double maxd = hit.distance.force_numerical_value_in(drone_mapper::cm);

        // step along ray and mark free; use half-resolution step to be conservative
        const double res = internal_map_->getMapConfig().resolution.force_numerical_value_in(drone_mapper::cm);
        const double step = std::max(0.5, res * 0.5);
        double t = 0.0;
        while (t < maxd) {
            const drone_mapper::Vec3 dir = dirFromAngles(abs_or.horizontal, abs_or.altitude);
            const double distance_cm = t; // t is in cm
            drone_mapper::Position3D sample{
                current_position_.x + dir.x * distance_cm * drone_mapper::x_extent[drone_mapper::cm],
                current_position_.y + dir.y * distance_cm * drone_mapper::y_extent[drone_mapper::cm],
                current_position_.z + dir.z * distance_cm * drone_mapper::z_extent[drone_mapper::cm]
            };
            internal_map_->set(sample, drone_mapper::types::VoxelOccupancy::Empty);
            t += step;
        }
        if (std::isfinite(maxd)) {
            const drone_mapper::Vec3 dir = dirFromAngles(abs_or.horizontal, abs_or.altitude);
            const double distance_cm = maxd;
            drone_mapper::Position3D hitpos{
                current_position_.x + dir.x * distance_cm * drone_mapper::x_extent[drone_mapper::cm],
                current_position_.y + dir.y * distance_cm * drone_mapper::y_extent[drone_mapper::cm],
                current_position_.z + dir.z * distance_cm * drone_mapper::z_extent[drone_mapper::cm]
            };
            internal_map_->set(hitpos, drone_mapper::types::VoxelOccupancy::Occupied);
        }
    }

    // For Phase 1, keep the drone hovering until DroneControlImpl is fully ported.
    drone_mapper::types::MovementCommand cmd;
    cmd.type = drone_mapper::types::MovementCommandType::Hover;
    return cmd;
}

void MappingAlgorithmImpl::applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) {
    if (!internal_map_) return;
    for (const auto& v : voxels) {
        internal_map_->set(v.position, v.value);
    }
}

} // namespace drone_mapper
