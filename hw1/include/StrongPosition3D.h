#pragma once

#include <mp-units/systems/si/unit_symbols.h>

struct StrongPosition3D {
    mp_units::quantity<mp_units::si::unit_symbols::cm> x;
    mp_units::quantity<mp_units::si::unit_symbols::cm> y;
    mp_units::quantity<mp_units::si::unit_symbols::cm> z;
};

struct Pose3D {
    StrongPosition3D position;
    mp_units::quantity<mp_units::si::unit_symbols::deg> orientation;
};