#include "MockMovementDriver.h"
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>

namespace {

constexpr double FULL_CIRCLE_DEG    = 360.0;
constexpr double STRAIGHT_ANGLE_DEG = 180.0;
constexpr double DEG_TO_RAD = std::numbers::pi_v<double> / STRAIGHT_ANGLE_DEG;

// True when (x,y,z) is a valid index into map.
bool inBounds(const Map3D& map, int x, int y, int z) {
    return x >= 0 && y >= 0 && z >= 0 &&
           x < static_cast<int>(map.width()) &&
           y < static_cast<int>(map.height()) &&
           z < static_cast<int>(map.depth());
}

} // namespace

MockMovementDriver::MockMovementDriver(StrongPosition3D& position,
                                       mp_units::quantity<mp_units::si::unit_symbols::deg>& orientation,
                                       const Map3D& ground_truth_map,
                                       double sphere_radius_cm,
                                       mp_units::quantity<mp_units::si::unit_symbols::deg> max_rotation,
                                       mp_units::quantity<mp_units::si::unit_symbols::cm> max_advance,
                                       mp_units::quantity<mp_units::si::unit_symbols::cm> max_elevation)
    : position_(position),
      orientation_(orientation),
      ground_truth_map_(ground_truth_map),
      sphere_radius_cm_(sphere_radius_cm),
      max_rotation_(max_rotation),
      max_advance_(max_advance),
      max_elevation_(max_elevation) {}

bool MockMovementDriver::canDroneOccupy(const StrongPosition3D& pos) const {
    const Map3D& map = ground_truth_map_;
    const int cx = static_cast<int>(std::round(pos.x.numerical_value_in(mp_units::si::unit_symbols::cm)));
    const int cy = static_cast<int>(std::round(pos.y.numerical_value_in(mp_units::si::unit_symbols::cm)));
    const int cz = static_cast<int>(std::round(pos.z.numerical_value_in(mp_units::si::unit_symbols::cm)));

    const int    r  = static_cast<int>(std::ceil(sphere_radius_cm_));
    const double r2 = sphere_radius_cm_ * sphere_radius_cm_;

    // Check the whole safety sphere, not just the center voxel.
    for (int dz = -r; dz <= r; ++dz) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (static_cast<double>(dx*dx + dy*dy + dz*dz) >= r2) continue;
                const int nx = cx + dx;
                const int ny = cy + dy;
                const int nz = cz + dz;
                // Out-of-map voxels are never FREE.
                if (!inBounds(map, nx, ny, nz) || map.at(nx, ny, nz) != Map3D::FREE) return false;
            }
        }
    }
    return true;
}

bool MockMovementDriver::moveForward(mp_units::quantity<mp_units::si::unit_symbols::cm> distance) {
    if (std::abs(distance.numerical_value_in(mp_units::si::unit_symbols::cm)) >
        std::abs(max_advance_.numerical_value_in(mp_units::si::unit_symbols::cm))) {
        throw std::invalid_argument("Movement exceeds drone capabilities");
    }
    if (collision_detected_) return false;
    if (distance == 0.0 * mp_units::si::unit_symbols::cm) return false;

    const double angle_deg = orientation_.numerical_value_in(mp_units::si::unit_symbols::deg);
    const double rad = angle_deg * DEG_TO_RAD;
    // Unit forward vector for the current orientation.
    const double fwd_x = std::round(std::cos(rad));
    const double fwd_y = std::round(std::sin(rad));

    const double dist_cm = distance.numerical_value_in(mp_units::si::unit_symbols::cm);
    // sign carries the travel direction; total is the unsigned distance to cover.
    const double sign  = dist_cm >= 0.0 ? 1.0 : -1.0;
    const double total = std::abs(dist_cm);
    const int    steps = std::max(1, static_cast<int>(std::ceil(total)));

    // Sample every centimeter so long moves cannot jump through a wall.
    for (int i = 1; i <= steps; ++i) {
        const double t = std::min(static_cast<double>(i), total);
        StrongPosition3D sample = position_;
        sample.x += (t * sign * fwd_x) * mp_units::si::unit_symbols::cm;
        sample.y += (t * sign * fwd_y) * mp_units::si::unit_symbols::cm;

        if (!canDroneOccupy(sample)) {
            collision_detected_ = true;
            const int sx = static_cast<int>(std::round(sample.x.numerical_value_in(mp_units::si::unit_symbols::cm)));
            const int sy = static_cast<int>(std::round(sample.y.numerical_value_in(mp_units::si::unit_symbols::cm)));
            const int sz = static_cast<int>(std::round(sample.z.numerical_value_in(mp_units::si::unit_symbols::cm)));
            std::cout << "Collision Detected! drone cannot advance through ("
                      << sx << ", " << sy << ", " << sz << ")." << std::endl;
            return false;
        }
    }

    // distance already carries the correct sign; fwd_x/fwd_y are unit vectors.
    position_.x += distance * fwd_x;
    position_.y += distance * fwd_y;
    return true;
}

bool MockMovementDriver::rotate(mp_units::quantity<mp_units::si::unit_symbols::deg> angle) {
    if (std::abs(angle.numerical_value_in(mp_units::si::unit_symbols::deg)) >
        std::abs(max_rotation_.numerical_value_in(mp_units::si::unit_symbols::deg))) {
        throw std::invalid_argument("Movement exceeds drone capabilities");
    }
    if (collision_detected_) return false;
    // The drone is a sphere: rotation never changes its collision footprint.
    orientation_ += angle;
    double normalized = std::fmod(orientation_.numerical_value_in(mp_units::si::unit_symbols::deg),
                                  FULL_CIRCLE_DEG);
    if (normalized < 0) normalized += FULL_CIRCLE_DEG;
    orientation_ = normalized * mp_units::si::unit_symbols::deg;
    return true;
}

bool MockMovementDriver::elevate(mp_units::quantity<mp_units::si::unit_symbols::cm> height) {
    if (std::abs(height.numerical_value_in(mp_units::si::unit_symbols::cm)) >
        std::abs(max_elevation_.numerical_value_in(mp_units::si::unit_symbols::cm))) {
        throw std::invalid_argument("Movement exceeds drone capabilities");
    }
    if (collision_detected_) return false;

    const double h_cm  = height.numerical_value_in(mp_units::si::unit_symbols::cm);
    const double sign  = h_cm >= 0.0 ? 1.0 : -1.0;
    const double total = std::abs(h_cm);
    const int    steps = std::max(1, static_cast<int>(std::ceil(total)));

    // Vertical motion uses the same conservative sampling as forward motion.
    for (int i = 1; i <= steps; ++i) {
        const double t = std::min(static_cast<double>(i), total);
        StrongPosition3D sample = position_;
        sample.z += (t * sign) * mp_units::si::unit_symbols::cm;

        if (!canDroneOccupy(sample)) {
            collision_detected_ = true;
            const int sz = static_cast<int>(std::round(sample.z.numerical_value_in(mp_units::si::unit_symbols::cm)));
            std::cout << "Collision Detected! drone cannot elevate through z="
                      << sz << "." << std::endl;
            return false;
        }
    }

    position_.z += height;
    return true;
}
