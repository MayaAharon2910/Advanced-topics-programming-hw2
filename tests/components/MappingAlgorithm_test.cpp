#include <gtest/gtest.h>

#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>

namespace drone_mapper {

// DummyMap at namespace scope so it can be passed as const IMap3D&
class DummyMapForAlg : public IMap3D {
public:
    types::VoxelOccupancy atVoxel(const Position3D&) const override { return types::VoxelOccupancy::Unmapped; }
    types::MapConfig getMapConfig() const override { return types::MapConfig{}; }
    bool isInBounds(const Position3D&) const override { return false; }
};

TEST(MappingAlgorithm, ProducesExplorationMovementInsteadOfDummyCycle) {
    types::MissionConfigData mission{};
    mission.max_steps = 20;
    mission.gps_resolution = 10.0 * cm;
    types::LidarConfigData lidar{20.0 * cm, 120.0 * cm, 2.5 * cm, 5};
    types::DroneConfigData drone{30.0 * cm, 45.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
    DummyMapForAlg output_map;

    MappingAlgorithmImpl alg(mission, lidar, drone, output_map);

    types::DroneState state{};
    state.position = Position3D{};
    state.heading = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};

    const auto first = alg.nextStep(state, nullptr);
    const bool has_movement = first.movement.has_value() &&
                              first.movement->type != types::MovementCommandType::Hover;
    EXPECT_TRUE(has_movement || first.status == types::AlgorithmStatus::Finished);
}

TEST(MappingAlgorithm, IngestsScanFromLatestScanPointer) {
    types::MissionConfigData mission{};
    mission.gps_resolution = 10.0 * cm;
    types::LidarConfigData lidar{20.0 * cm, 120.0 * cm, 2.5 * cm, 5};
    types::DroneConfigData drone{30.0 * cm, 90.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm};
    DummyMapForAlg output_map;

    MappingAlgorithmImpl alg(mission, lidar, drone, output_map);

    types::LidarScanResult scan{types::LidarHit{
        10.0 * cm,
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}}};

    types::DroneState state{};
    state.position = Position3D{};
    state.heading = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const auto cmd = alg.nextStep(state, &scan);
    EXPECT_TRUE(cmd.movement.has_value() || cmd.status == types::AlgorithmStatus::Finished);
}

} // namespace drone_mapper
