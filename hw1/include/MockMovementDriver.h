#pragma once

#include "IMovementDriver.h"
#include "Map3D.h"
#include "StrongPosition3D.h"
#include <mp-units/systems/si/unit_symbols.h>

class MockMovementDriver : public IMovementDriver {
public:
    // Creates a movement driver that checks a drone sphere against the ground-truth map.
    MockMovementDriver(StrongPosition3D& position,
                       mp_units::quantity<mp_units::si::unit_symbols::deg>& orientation,
                       const Map3D& ground_truth_map,
                       double sphere_radius_cm,
                       mp_units::quantity<mp_units::si::unit_symbols::deg> max_rotation,
                       mp_units::quantity<mp_units::si::unit_symbols::cm> max_advance,
                       mp_units::quantity<mp_units::si::unit_symbols::cm> max_elevation);

    // Moves forward or backward along the current heading if the path is collision-free.
    bool moveForward(mp_units::quantity<mp_units::si::unit_symbols::cm> distance) override;
    // Rotates the heading while preserving the spherical collision footprint.
    bool rotate(mp_units::quantity<mp_units::si::unit_symbols::deg> angle) override;
    // Moves vertically if the sampled path is collision-free.
    bool elevate(mp_units::quantity<mp_units::si::unit_symbols::cm> height) override;
    // Returns true when the drone sphere centered at pos fits entirely within free voxels.
    bool canDroneOccupy(const StrongPosition3D& pos) const;

private:
    StrongPosition3D& position_;
    mp_units::quantity<mp_units::si::unit_symbols::deg>& orientation_;
    const Map3D& ground_truth_map_;
    double sphere_radius_cm_ = 0.0;
    mp_units::quantity<mp_units::si::unit_symbols::deg> max_rotation_;
    mp_units::quantity<mp_units::si::unit_symbols::cm> max_advance_;
    mp_units::quantity<mp_units::si::unit_symbols::cm> max_elevation_;
    bool collision_detected_ = false;
};
