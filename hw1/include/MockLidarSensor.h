#pragma once

#include "ILidarSensor.h"
#include "Map3D.h"
#include "StrongPosition3D.h"
#include <mp-units/systems/si/unit_symbols.h>
#include <vector>

class MockLidarSensor : public ILidarSensor {
public:
    // Creates a simulated LiDAR over the ground-truth map with the given range and beam settings.
    MockLidarSensor(const Map3D& map,
                    mp_units::quantity<mp_units::si::unit_symbols::cm> z_min,
                    mp_units::quantity<mp_units::si::unit_symbols::cm> z_max,
                    int fovc,
                    mp_units::quantity<mp_units::si::unit_symbols::cm> d);

    // Casts the configured LiDAR beam pattern and returns only beams that hit an occupied voxel.
    std::vector<ScanResult> scan(const StrongPosition3D& position,
                                 mp_units::quantity<mp_units::si::unit_symbols::deg, double> azimuth,
                                 mp_units::quantity<mp_units::si::unit_symbols::deg, double> elevation) const override;

private:
    const Map3D& map_;
    double z_min_;
    double z_max_;
    int fovc_;
    double d_;
};
