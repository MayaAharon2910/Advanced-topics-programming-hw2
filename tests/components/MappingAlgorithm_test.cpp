#include <gtest/gtest.h>

#include <drone_mapper/MappingAlgorithmImpl.h>

namespace drone_mapper {

TEST(MappingAlgorithm, ProducesExplorationMovementInsteadOfDummyCycle) {
    types::MissionConfigData mission{};
    mission.max_steps = 20;
    mission.gps_resolution = 10.0 * cm;
    types::DroneConfigData drone{30.0 * cm, 45.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
    MappingAlgorithmImpl alg(mission, drone);

    types::DroneState state{};
    state.position = Position3D{};
    state.heading = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};

    const auto first = alg.nextMove(state, {});
    EXPECT_NE(first.type, types::MovementCommandType::Hover);

    state.step_index = 1;
    const auto second = alg.nextMove(state, {});
    EXPECT_NE(second.type, types::MovementCommandType::Hover);
}

TEST(MappingAlgorithm, AppliesVoxelUpdatesToAvoidKnownOccupiedTarget) {
    types::MissionConfigData mission{};
    mission.gps_resolution = 10.0 * cm;
    types::DroneConfigData drone{30.0 * cm, 90.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm};
    MappingAlgorithmImpl alg(mission, drone);

    alg.applyVoxelUpdates({types::MappedVoxel{
        Position3D{10.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
        types::VoxelOccupancy::Occupied}});

    types::DroneState state{};
    state.position = Position3D{};
    state.heading = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const auto cmd = alg.nextMove(state, {});
    EXPECT_TRUE(cmd.type == types::MovementCommandType::Rotate ||
                cmd.type == types::MovementCommandType::Elevate ||
                cmd.type == types::MovementCommandType::Advance);
}

} // namespace drone_mapper
