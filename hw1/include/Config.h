#pragma once

#include "StrongPosition3D.h"
#include <mp-units/systems/si/unit_symbols.h>
#include <vector>

struct Config {
    mp_units::quantity<mp_units::si::unit_symbols::deg> max_rotation;
    mp_units::quantity<mp_units::si::unit_symbols::cm> max_elevation;
    mp_units::quantity<mp_units::si::unit_symbols::cm> max_advance;
    mp_units::quantity<mp_units::si::unit_symbols::cm> min_pass_width_cm;
    mp_units::quantity<mp_units::si::unit_symbols::cm> min_pass_length_cm;
    mp_units::quantity<mp_units::si::unit_symbols::cm> min_pass_height_cm;
    mp_units::quantity<mp_units::si::unit_symbols::cm> lidar_z_min_cm;
    mp_units::quantity<mp_units::si::unit_symbols::cm> lidar_z_max_cm;
    int lidar_fovc;
    mp_units::quantity<mp_units::si::unit_symbols::cm> lidar_d_cm;
};

struct MissionConfig {
    mp_units::quantity<mp_units::si::unit_symbols::cm> min_x;
    mp_units::quantity<mp_units::si::unit_symbols::cm> max_x;
    mp_units::quantity<mp_units::si::unit_symbols::cm> min_y;
    mp_units::quantity<mp_units::si::unit_symbols::cm> max_y;
    mp_units::quantity<mp_units::si::unit_symbols::cm> min_height;
    mp_units::quantity<mp_units::si::unit_symbols::cm> max_height;
    StrongPosition3D start_position;
    mp_units::quantity<mp_units::si::unit_symbols::deg> start_orientation;
    bool has_bounds = false;
    int resolution_xy_digits = 0;
    int resolution_height_digits = 0;
    // Not used by the navigation logic in Exercise 1; parsed and stored to support the required mission-file format.
    std::vector<StrongPosition3D> recharge_positions;
};
