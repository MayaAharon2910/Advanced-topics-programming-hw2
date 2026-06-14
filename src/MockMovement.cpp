#include <drone_mapper/MockMovement.h>

#include <mp-units/systems/si/math.h>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D current_position = gps_.position();
    const Orientation current_heading = gps_.heading();
    const auto cos_altitude = si::cos(current_heading.altitude);
    const double dx = (cos_altitude * si::cos(current_heading.horizontal)).force_numerical_value_in(mp::one);
    const double dy = (cos_altitude * si::sin(current_heading.horizontal)).force_numerical_value_in(mp::one);
    const double dz = si::sin(current_heading.altitude).force_numerical_value_in(mp::one);
    const double distance_cm = distance.force_numerical_value_in(cm);

    gps_.setPosition(Position3D{
        current_position.x + dx * distance_cm * x_extent[cm],
        current_position.y + dy * distance_cm * y_extent[cm],
        current_position.z + dz * distance_cm * z_extent[cm],
    });
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D current_position = gps_.position();
    gps_.setPosition(Position3D{
        current_position.x,
        current_position.y,
        current_position.z + distance.force_numerical_value_in(cm) * z_extent[cm],
    });
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
