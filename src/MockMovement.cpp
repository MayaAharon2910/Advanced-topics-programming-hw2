#include <drone_mapper/MockMovement.h>

#include <cmath>
#include <mp-units/systems/si/math.h>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

MockMovement::MockMovement(MockGPS& gps,
                           const IMap3D& hidden_map,
                           const types::MappingBounds& bounds,
                           PhysicalLength drone_radius)
    : gps_(gps),
      hidden_map_(&hidden_map),
      bounds_(bounds),
      drone_radius_(drone_radius),
      has_collision_check_(true) {}

// -----------------------------------------------------------------------------

bool MockMovement::outOfBounds(const Position3D& pos) const {
    if (!has_collision_check_) return false;
    const double x = pos.x.force_numerical_value_in(cm);
    const double y = pos.y.force_numerical_value_in(cm);
    const double z = pos.z.force_numerical_value_in(cm);
    const bool unset = bounds_.min_x.force_numerical_value_in(cm) == 0.0 &&
                       bounds_.max_x.force_numerical_value_in(cm) == 0.0 &&
                       bounds_.min_y.force_numerical_value_in(cm) == 0.0 &&
                       bounds_.max_y.force_numerical_value_in(cm) == 0.0 &&
                       bounds_.min_height.force_numerical_value_in(cm) == 0.0 &&
                       bounds_.max_height.force_numerical_value_in(cm) == 0.0;
    if (unset) return false;
    return x < bounds_.min_x.force_numerical_value_in(cm) ||
           x > bounds_.max_x.force_numerical_value_in(cm) ||
           y < bounds_.min_y.force_numerical_value_in(cm) ||
           y > bounds_.max_y.force_numerical_value_in(cm) ||
           z < bounds_.min_height.force_numerical_value_in(cm) ||
           z > bounds_.max_height.force_numerical_value_in(cm);
}

// Uses the same safety-sphere occupancy rule as the HW1 movement model:
// checks the full safety sphere, not just the center voxel.
bool MockMovement::canDroneOccupy(const Position3D& center) const {
    if (!has_collision_check_ || hidden_map_ == nullptr) return true;

    const double r_cm = drone_radius_.force_numerical_value_in(cm);
    const double r2   = r_cm * r_cm;
    const int    r    = static_cast<int>(std::ceil(r_cm));

    const double cx = center.x.force_numerical_value_in(cm);
    const double cy = center.y.force_numerical_value_in(cm);
    const double cz = center.z.force_numerical_value_in(cm);

    for (int dz = -r; dz <= r; ++dz) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                // Skip voxels outside the sphere
                if (static_cast<double>(dx*dx + dy*dy + dz*dz) >= r2) continue;
                const Position3D sample{
                    (cx + static_cast<double>(dx)) * x_extent[cm],
                    (cy + static_cast<double>(dy)) * y_extent[cm],
                    (cz + static_cast<double>(dz)) * z_extent[cm],
                };
                if (hidden_map_->atVoxel(sample) == types::VoxelOccupancy::Occupied)
                    return false;
            }
        }
    }
    return true;
}

// -----------------------------------------------------------------------------

types::MovementResult MockMovement::rotate(types::RotationDirection direction,
                                           HorizontalAngle angle) {
    // The drone is a sphere: rotation never changes its collision footprint.
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D  current_pos = gps_.position();
    const Orientation heading     = gps_.heading();

    const auto   cos_alt = si::cos(heading.altitude);
    const double dx = (cos_alt * si::cos(heading.horizontal)).force_numerical_value_in(mp::one);
    const double dy = (cos_alt * si::sin(heading.horizontal)).force_numerical_value_in(mp::one);
    const double dz = si::sin(heading.altitude).force_numerical_value_in(mp::one);

    const double dist_cm = distance.force_numerical_value_in(cm);
    const double sign    = dist_cm >= 0.0 ? 1.0 : -1.0;
    const double total   = std::abs(dist_cm);
    // Sample every centimeter so long moves cannot jump through a wall (HW1 pattern).
    const int steps = std::max(1, static_cast<int>(std::ceil(total)));

    for (int i = 1; i <= steps; ++i) {
        const double t = std::min(static_cast<double>(i), total);
        const Position3D sample{
            current_pos.x + (t * sign * dx) * x_extent[cm],
            current_pos.y + (t * sign * dy) * y_extent[cm],
            current_pos.z + (t * sign * dz) * z_extent[cm],
        };
        if (outOfBounds(sample))
            return types::MovementResult{false, "advance would leave mission boundaries"};
        if (!canDroneOccupy(sample))
            return types::MovementResult{false, "advance collides with obstacle"};
    }

    const Position3D new_pos{
        current_pos.x + (dist_cm * dx) * x_extent[cm],
        current_pos.y + (dist_cm * dy) * y_extent[cm],
        current_pos.z + (dist_cm * dz) * z_extent[cm],
    };
    gps_.setPosition(new_pos);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const double h_cm  = distance.force_numerical_value_in(cm);
    const double sign  = h_cm >= 0.0 ? 1.0 : -1.0;
    const double total = std::abs(h_cm);
    // Same conservative sampling as advance (HW1 pattern).
    const int steps = std::max(1, static_cast<int>(std::ceil(total)));

    for (int i = 1; i <= steps; ++i) {
        const double t = std::min(static_cast<double>(i), total);
        const Position3D sample{
            current_pos.x,
            current_pos.y,
            current_pos.z + (t * sign) * z_extent[cm],
        };
        if (outOfBounds(sample))
            return types::MovementResult{false, "elevate would leave mission boundaries"};
        if (!canDroneOccupy(sample))
            return types::MovementResult{false, "elevate collides with obstacle"};
    }

    gps_.setPosition(Position3D{
        current_pos.x,
        current_pos.y,
        current_pos.z + h_cm * z_extent[cm],
    });
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
