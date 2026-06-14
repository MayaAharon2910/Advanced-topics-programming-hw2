#include <gtest/gtest.h>

#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/ScanResultToVoxels.h>

TEST(SimulationRun, MockGPSAndMovement) {
    drone_mapper::Position3D pos{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    drone_mapper::Orientation head{
        0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
        0.0 * drone_mapper::altitude_angle[drone_mapper::deg]};
    drone_mapper::MockGPS gps(pos, head);
    drone_mapper::MockMovement movement(gps);

    // Rotate left by 90 degrees
    auto res = movement.rotate(
        drone_mapper::types::RotationDirection::Left,
        90.0 * drone_mapper::horizontal_angle[drone_mapper::deg]);
    EXPECT_TRUE(res);
    auto new_heading = gps.heading();
    EXPECT_NEAR(new_heading.horizontal.numerical_value_in(drone_mapper::deg), 90.0, 1e-6);
}

TEST(SimulationRun, ScanResultToVoxelsMarksFreeRayAndHit) {
    const auto voxels = drone_mapper::ScanResultToVoxels::convert(
        drone_mapper::Position3D{},
        drone_mapper::Orientation{
            0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
            0.0 * drone_mapper::altitude_angle[drone_mapper::deg]},
        {drone_mapper::types::LidarHit{
            3.0 * drone_mapper::cm,
            drone_mapper::Orientation{
                0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
                0.0 * drone_mapper::altitude_angle[drone_mapper::deg]}}});

    ASSERT_GE(voxels.size(), 4U);
    EXPECT_EQ(voxels.front().value, drone_mapper::types::VoxelOccupancy::Empty);
    EXPECT_EQ(voxels.back().value, drone_mapper::types::VoxelOccupancy::Occupied);
    EXPECT_NEAR(voxels.back().position.x.numerical_value_in(drone_mapper::cm), 3.0, 1e-9);
}

TEST(SimulationRun, MappingAlgorithmDoesNotHoverForever) {
    drone_mapper::MappingAlgorithmImpl algorithm(
        drone_mapper::types::MissionConfigData{4, 1.0 * drone_mapper::cm, 1});

    bool emitted_movement = false;
    for (std::size_t i = 0; i < 4; ++i) {
        const auto command = algorithm.nextMove(
            drone_mapper::types::DroneState{
                drone_mapper::Position3D{},
                drone_mapper::Orientation{
                    0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
                    0.0 * drone_mapper::altitude_angle[drone_mapper::deg]},
                i},
            {});
        emitted_movement = emitted_movement ||
            command.type != drone_mapper::types::MovementCommandType::Hover;
    }

    EXPECT_TRUE(emitted_movement);
}
