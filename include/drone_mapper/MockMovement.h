#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/MockGPS.h>

namespace drone_mapper {

class MockMovement final : public IDroneMovement {
public:
    // Constructor for tests that only need kinematic movement.
    explicit MockMovement(MockGPS& gps);
    // Constructor used by simulations: checks collisions against the hidden map
    // and prevents leaving the mission bounds.
    MockMovement(MockGPS& gps,
                 const IMap3D& hidden_map,
                 const types::MappingBounds& bounds,
                 PhysicalLength drone_radius);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    // True when the entire drone sphere fits in free space at the requested position.
    // Uses the same safety-sphere occupancy rule as the HW1 movement model.
    [[nodiscard]] bool canDroneOccupy(const Position3D& center) const;
    // True when the center position is outside the allowed mission bounds.
    [[nodiscard]] bool outOfBounds(const Position3D& pos) const;

    MockGPS& gps_;
    const IMap3D* hidden_map_ = nullptr;
    types::MappingBounds bounds_{};
    PhysicalLength drone_radius_{0.0 * cm};
    bool has_collision_check_ = false;
};

} // namespace drone_mapper
