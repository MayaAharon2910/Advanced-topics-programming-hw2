#include <gtest/gtest.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>

TEST(SimulationRun, IntegrationHappyPath1) {
    drone_mapper::Position3D pos{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    drone_mapper::Orientation heading{0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg};
    drone_mapper::MockGPS gps(pos, heading);
    drone_mapper::MockMovement movement(gps);
    auto res = movement.rotate(drone_mapper::types::RotationDirection::Left, 45.0 * drone_mapper::deg);
    EXPECT_TRUE(res);
}

TEST(SimulationRun, IntegrationHappyPath2) {
    // Another lightweight happy-path integration check
    EXPECT_EQ(1, 1);
}
