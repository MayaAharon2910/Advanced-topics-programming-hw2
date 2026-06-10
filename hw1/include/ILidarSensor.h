#pragma once

#include <vector>
#include "StrongPosition3D.h"
#include <mp-units/systems/si/unit_symbols.h>
#include "ScanResult.h"

class ILidarSensor {
public:
    // Allows deleting LiDAR implementations through the interface pointer.
    virtual ~ILidarSensor() = default;
    // Scans from a pose direction and returns beam distances and directions.
    virtual std::vector<ScanResult> scan(const StrongPosition3D& position,
                                         mp_units::quantity<mp_units::si::unit_symbols::deg, double> azimuth,
                                         mp_units::quantity<mp_units::si::unit_symbols::deg, double> elevation) const = 0;
};
