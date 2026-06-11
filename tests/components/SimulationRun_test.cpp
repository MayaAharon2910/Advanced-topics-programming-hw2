#include <gtest/gtest.h>

#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>

TEST(SimulationRun, MockGPSAndMovement) {
    drone_mapper::Position3D pos{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    drone_mapper::Orientation head{0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg};
    drone_mapper::MockGPS gps(pos, head);
    drone_mapper::MockMovement movement(gps);

    // Rotate left by 90 degrees
    auto res = movement.rotate(drone_mapper::types::RotationDirection::Left, 90.0 * drone_mapper::deg);
    EXPECT_TRUE(res);
    auto new_heading = gps.heading();
    EXPECT_NEAR(new_heading.horizontal.numerical_value_in(drone_mapper::deg), 90.0, 1e-6);
}
