#pragma once

#include "StrongPosition3D.h"

class IPositionSensor {
public:
    // Allows deleting position-sensor implementations through the interface pointer.
    virtual ~IPositionSensor() = default;
    // Returns the latest measured position and orientation.
    virtual Pose3D getPosition() const = 0;
};
