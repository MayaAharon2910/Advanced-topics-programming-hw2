#include <drone_mapper/MockMovement.h>

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

// ─── private helpers ────────────────────────────────────────────────────────

bool MockMovement::outOfBounds(const Position3D& pos) const {
    if (!has_collision_check_) return false;
    const double x = pos.x.force_numerical_value_in(cm);
    const double y = pos.y.force_numerical_value_in(cm);
    const double z = pos.z.force_numerical_value_in(cm);
    // All-zero bounds means no restriction
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

bool MockMovement::collidesAt(const Position3D& pos) const {
    if (!has_collision_check_ || hidden_map_ == nullptr) return false;
    return hidden_map_->atVoxel(pos) == types::VoxelOccupancy::Occupied;
}

// ─── IDroneMovement interface ────────────────────────────────────────────────

types::MovementResult MockMovement::rotate(types::RotationDirection direction,
                                           HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D current_pos  = gps_.position();
    const Orientation heading     = gps_.heading();

    const auto cos_alt = si::cos(heading.altitude);
    const double dx = (cos_alt * si::cos(heading.horizontal)).force_numerical_value_in(mp::one);
    const double dy = (cos_alt * si::sin(heading.horizontal)).force_numerical_value_in(mp::one);
    const double dz = si::sin(heading.altitude).force_numerical_value_in(mp::one);
    const double dist_cm = distance.force_numerical_value_in(cm);

    const Position3D new_pos{
        current_pos.x + dx * dist_cm * x_extent[cm],
        current_pos.y + dy * dist_cm * y_extent[cm],
        current_pos.z + dz * dist_cm * z_extent[cm],
    };

    if (outOfBounds(new_pos)) {
        return types::MovementResult{false,
            "advance would leave mission boundaries"};
    }
    if (collidesAt(new_pos)) {
        return types::MovementResult{false,
            "advance collides with obstacle"};
    }

    gps_.setPosition(new_pos);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const Position3D new_pos{
        current_pos.x,
        current_pos.y,
        current_pos.z + distance.force_numerical_value_in(cm) * z_extent[cm],
    };

    if (outOfBounds(new_pos)) {
        return types::MovementResult{false,
            "elevate would leave mission boundaries"};
    }
    if (collidesAt(new_pos)) {
        return types::MovementResult{false,
            "elevate collides with obstacle"};
    }

    gps_.setPosition(new_pos);
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
