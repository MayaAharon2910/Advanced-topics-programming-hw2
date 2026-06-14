#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>

namespace drone_mapper {

class DummyMap : public IMutableMap3D {
public:
    types::VoxelOccupancy atVoxel(const Position3D& pos) const override { (void)pos; return types::VoxelOccupancy::Empty; }
    types::MapConfig getMapConfig() const override { return types::MapConfig{}; }
    MOCK_METHOD(void, set, (const Position3D& pos, types::VoxelOccupancy value), (override));
    void save(const std::filesystem::path& output_path) const override { (void)output_path; }
};

class MockAlgorithm : public IMappingAlgorithm {
public:
    MOCK_METHOD(types::MovementCommand, nextMove, (const types::DroneState& state, const types::LidarScanResult& latest_scan), (override));
    MOCK_METHOD(void, applyVoxelUpdates, (const std::vector<types::MappedVoxel>& voxels), (override));
};

TEST(DroneControl, AppliesScanVoxelsAndExecutesAlgorithmCommand) {
    types::DroneConfigData drone_cfg{};
    types::MissionConfigData mission_cfg{};
    DummyMap output_map;
    MockAlgorithm alg;

    class DummyLidarImpl : public ILidar {
    public:
        types::LidarScanResult scan(Orientation) const override {
            return {types::LidarHit{2.0 * cm, Orientation{0.0 * deg, 0.0 * altitude_angle[deg]}}};
        }
    } lidar_impl;
    class DummyGPSImpl2 : public IGPS {
    public:
        Position3D position() const override { return Position3D{}; }
        Orientation heading() const override { return Orientation{}; }
    } gps_impl;
    class DummyMovementImpl : public IDroneMovement {
    public:
        MOCK_METHOD(types::MovementResult, rotate, (types::RotationDirection, HorizontalAngle), (override));
        MOCK_METHOD(types::MovementResult, advance, (PhysicalLength), (override));
        types::MovementResult elevate(PhysicalLength) override { return {}; }
    } movement_impl;

    EXPECT_CALL(output_map, set(testing::_, testing::_)).Times(testing::AtLeast(3));
    EXPECT_CALL(alg, applyVoxelUpdates(testing::SizeIs(testing::Ge(3U))));
    EXPECT_CALL(alg, nextMove(testing::_, testing::SizeIs(1)))
        .WillOnce(testing::Return(types::MovementCommand{
            types::MovementCommandType::Advance,
            types::RotationDirection::Left,
            0.0 * horizontal_angle[deg],
            5.0 * cm,
        }));
    EXPECT_CALL(movement_impl, advance(testing::_))
        .WillOnce(testing::Return(types::MovementResult{true, ""}));

    DroneControlImpl control(drone_cfg, mission_cfg, lidar_impl, gps_impl, movement_impl, output_map, alg);
    const auto result = control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
}

} // namespace drone_mapper
