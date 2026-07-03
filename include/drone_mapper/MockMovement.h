#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/MockGPS.h>

namespace drone_mapper {

class MockMovement final : public IDroneMovement {
public:
    // Basic constructor (no collision detection)
    explicit MockMovement(MockGPS& gps);
    // Extended constructor: enables collision detection against hidden_map
    // and boundary clamping within bounds.
    MockMovement(MockGPS& gps,
                 const IMap3D& hidden_map,
                 const types::MappingBounds& bounds,
                 PhysicalLength drone_radius);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    // Returns true when the drone sphere centered at pos fits entirely within free voxels.
    // Ported from HW1 MockMovementDriver::canDroneOccupy.
    [[nodiscard]] bool canDroneOccupy(const Position3D& center) const;
    // Returns true if position is outside mission bounds
    [[nodiscard]] bool outOfBounds(const Position3D& pos) const;

    MockGPS& gps_;
    const IMap3D* hidden_map_ = nullptr;   // optional — null means no collision check
    types::MappingBounds bounds_{};
    PhysicalLength drone_radius_{0.0 * cm};
    bool has_collision_check_ = false;
};

} // namespace drone_mapper
