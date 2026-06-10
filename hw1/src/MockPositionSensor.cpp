#include "MockPositionSensor.h"

MockPositionSensor::MockPositionSensor(const StrongPosition3D& position,
                                       const mp_units::quantity<mp_units::si::unit_symbols::deg>& orientation)
    : position_(position), orientation_(orientation) {}

Pose3D MockPositionSensor::getPosition() const {
    // Return a snapshot of the live simulator pose references.
    return { position_, orientation_ };
}
