#pragma once

#include "IPositionSensor.h"
#include "StrongPosition3D.h"

class MockPositionSensor : public IPositionSensor {
public:
    // Creates a sensor that reports live references to the simulated pose.
    MockPositionSensor(const StrongPosition3D& position,
                       const mp_units::quantity<mp_units::si::unit_symbols::deg>& orientation);
    // Returns the current referenced position and orientation.
    Pose3D getPosition() const override;

private:
    const StrongPosition3D& position_;
    const mp_units::quantity<mp_units::si::unit_symbols::deg>& orientation_;
};
