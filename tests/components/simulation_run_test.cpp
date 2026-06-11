#include <gtest/gtest.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>

TEST(SimulationRun, MockGPSAndMovement) {
    drone_mapper::Position3D pos{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    drone_mapper::Orientation heading{0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg};
    drone_mapper::MockGPS gps(pos, heading);
    drone_mapper::MockMovement movement(gps);

    // Rotate left by 90 degrees
    auto res = movement.rotate(drone_mapper::types::RotationDirection::Left, 90.0 * drone_mapper::deg);
    EXPECT_TRUE(res);
}
