#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>

namespace drone_mapper {

class DummyMap : public IMutableMap3D {
public:
    types::VoxelOccupancy atVoxel(const Position3D& pos) const override { (void)pos; return types::VoxelOccupancy::Empty; }
    types::MapConfig getMapConfig() const override { return types::MapConfig{}; }
    bool isInBounds(const Position3D& pos) const override { (void)pos; return true; }
    MOCK_METHOD(void, set, (const Position3D& pos, types::VoxelOccupancy value), (override));
    void save(const std::filesystem::path& output_path) const override { (void)output_path; }
};

class MockAlgorithm : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    MOCK_METHOD(types::MappingStepCommand, nextStep, (const types::DroneState& state, const types::LidarScanResult* latest_scan), (override));
};

TEST(DroneControl, ExecutesAlgorithmMovementCommand) {
    types::DroneConfigData drone_cfg{30.0 * cm, 45.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
    types::MissionConfigData mission_cfg{10, 10.0 * cm, {}, 1};
    types::LidarConfigData lidar_cfg{20.0 * cm, 120.0 * cm, 2.5 * cm, 5};
    DummyMap output_map;

    class DummyLidarImpl : public ILidar {
    public:
        types::LidarScanResult scan(Orientation) const override { return {}; }
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

    MockAlgorithm alg(mission_cfg, lidar_cfg, drone_cfg, output_map);
    EXPECT_CALL(alg, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            types::MovementCommand{
                types::MovementCommandType::Advance,
                types::RotationDirection::Left,
                0.0 * horizontal_angle[deg],
                5.0 * cm,
            },
            std::nullopt,
            types::AlgorithmStatus::Working,
        }));
    EXPECT_CALL(movement_impl, advance(testing::_))
        .WillOnce(testing::Return(types::MovementResult{true, ""}));

    DroneControlImpl control(drone_cfg, mission_cfg, lidar_cfg, lidar_impl, gps_impl, movement_impl, output_map, alg);
    const auto result = control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
}

TEST(DroneControl, ReturnsCompletedWhenAlgorithmFinishes) {
    types::DroneConfigData drone_cfg{30.0 * cm, 45.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
    types::MissionConfigData mission_cfg{10, 10.0 * cm, {}, 1};
    types::LidarConfigData lidar_cfg{20.0 * cm, 120.0 * cm, 2.5 * cm, 5};
    DummyMap output_map;

    class DummyLidar2 : public ILidar {
    public:
        types::LidarScanResult scan(Orientation) const override { return {}; }
    } lidar_impl;
    class DummyGPS2 : public IGPS {
    public:
        Position3D position() const override { return Position3D{}; }
        Orientation heading() const override { return Orientation{}; }
    } gps_impl;
    class DummyMovement2 : public IDroneMovement {
    public:
        types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override { return {true, {}}; }
        types::MovementResult advance(PhysicalLength) override { return {true, {}}; }
        types::MovementResult elevate(PhysicalLength) override { return {true, {}}; }
    } movement_impl;

    MockAlgorithm alg(mission_cfg, lidar_cfg, drone_cfg, output_map);
    EXPECT_CALL(alg, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            std::nullopt,
            std::nullopt,
            types::AlgorithmStatus::Finished,
        }));

    DroneControlImpl control(drone_cfg, mission_cfg, lidar_cfg, lidar_impl, gps_impl, movement_impl, output_map, alg);
    const auto result = control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Completed);
}

} // namespace drone_mapper
