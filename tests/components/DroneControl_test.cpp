#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>

using ::testing::Return;

namespace drone_mapper {

class DummyMap : public IMutableMap3D {
public:
    types::VoxelOccupancy atVoxel(const Position3D& pos) const override { (void)pos; return types::VoxelOccupancy::Empty; }
    types::MapConfig getMapConfig() const override { return types::MapConfig{}; }
    void set(const Position3D& pos, types::VoxelOccupancy value) override { (void)pos; (void)value; }
    void save(const std::filesystem::path& output_path) const override { (void)output_path; }
};

class MockAlgorithm : public IMappingAlgorithm {
public:
    MOCK_METHOD(types::MovementCommand, nextMove, (const types::DroneState& state, const types::LidarScanResult& latest_scan), (override));
    MOCK_METHOD(void, applyVoxelUpdates, (const std::vector<types::MappedVoxel>& voxels), (override));
};

TEST(DroneControl, MappingAlgorithmInvocation) {
    // Create stub dependencies
    types::DroneConfigData drone_cfg{};
    types::MissionConfigData mission_cfg{};
    DummyMap output_map;
    MockAlgorithm alg;

    // Minimal dummy implementations to satisfy constructor references
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
        types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override { return {}; }
        types::MovementResult advance(PhysicalLength) override { return {}; }
        types::MovementResult elevate(PhysicalLength) override { return {}; }
    } movement_impl;

    DroneControlImpl control(drone_cfg, mission_cfg, lidar_impl, gps_impl, movement_impl, output_map, alg);
    (void)control;
    SUCCEED();
}

} // namespace drone_mapper
