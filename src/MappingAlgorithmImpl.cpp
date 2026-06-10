#include <drone_mapper/MappingAlgorithmImpl.h>

#include <utility>
#include <vector>
#include <set>
#include <cmath>

namespace drone_mapper {

MappingAlgorithmImpl::MappingAlgorithmImpl(types::MissionConfigData mission)
    : mission_(std::move(mission)) {}

// Small helpers
namespace {
using drone_mapper::types::Position3D;
using drone_mapper::types::HorizontalAngle;
using drone_mapper::types::AltitudeAngle;
using drone_mapper::types::PhysicalLength;
using drone_mapper::XLength;
using drone_mapper::YLength;
using drone_mapper::ZLength;

struct Vec3 { double x; double y; double z; };

Vec3 dirFromAngles(const HorizontalAngle& ha, const AltitudeAngle& aa) {
    using namespace mp_units::si;
    const auto cos_alt = si::cos(aa);
    const double dx = (cos_alt * si::cos(ha)).force_numerical_value_in(mp_units::one);
    const double dy = (cos_alt * si::sin(ha)).force_numerical_value_in(mp_units::one);
    const double dz = si::sin(aa).force_numerical_value_in(mp_units::one);
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
        types::MapConfig cfg = types::MapConfig{};
        cfg.offset = types::Position3D{};
        cfg.resolution = types::PhysicalLength{1.0 * cm};
        internal_map_ = std::make_unique<Map3DImpl>(nptr, cfg);
    }

    // Process beams: latest_scan contains relative orientations; convert to absolute
    for (const auto& hit : latest_scan) {
        types::Orientation abs_or{
            HorizontalAngle{orientation_.horizontal + hit.angle.horizontal},
            AltitudeAngle{orientation_.altitude + hit.angle.altitude}
        };
        const Vec3 d = dirFromAngles(abs_or.horizontal, abs_or.altitude);
        const double maxd = hit.distance.force_numerical_value_in(cm);

        // step along ray and mark free; use half-resolution step to be conservative
        const double res = internal_map_->getMapConfig().resolution.force_numerical_value_in(cm);
        const double step = std::max(0.5, res * 0.5);
        double t = 0.0;
        while (t < maxd) {
            Position3D sample{
                XLength{ (current_position_.x + XLength{t * cm}).x },
                YLength{ (current_position_.y + YLength{t * cm}).y },
                ZLength{ (current_position_.z + ZLength{t * cm}).z }
            };
            internal_map_->set(sample, types::VoxelOccupancy::Empty);
            t += step;
        }
        if (std::isfinite(maxd)) {
            Position3D hitpos{
                XLength{ (current_position_.x + XLength{maxd * cm}).x },
                YLength{ (current_position_.y + YLength{maxd * cm}).y },
                ZLength{ (current_position_.z + ZLength{maxd * cm}).z }
            };
            internal_map_->set(hitpos, types::VoxelOccupancy::Occupied);
        }
    }

    // For Phase 1, keep the drone hovering until DroneControlImpl is fully ported.
    types::MovementCommand cmd;
    cmd.type = types::MovementCommandType::Hover;
    return cmd;
}

void MappingAlgorithmImpl::applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) {
    if (!internal_map_) return;
    for (const auto& v : voxels) {
        internal_map_->set(v.position, v.value);
    }
}

} // namespace drone_mapper
