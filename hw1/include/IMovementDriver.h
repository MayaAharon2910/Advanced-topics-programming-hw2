#pragma once

#include <mp-units/systems/si/unit_symbols.h>

class IMovementDriver {
public:
    // Allows deleting movement-driver implementations through the interface pointer.
    virtual ~IMovementDriver() = default;
    // Attempts to move along the current heading and reports whether it succeeded.
    virtual bool moveForward(mp_units::quantity<mp_units::si::unit_symbols::cm> distance) = 0;
    // Attempts to rotate the drone and reports whether it succeeded.
    virtual bool rotate(mp_units::quantity<mp_units::si::unit_symbols::deg> angle) = 0;
    // Attempts to change altitude and reports whether it succeeded.
    virtual bool elevate(mp_units::quantity<mp_units::si::unit_symbols::cm> height) = 0;
};
