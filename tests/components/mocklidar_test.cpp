#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/MockLidar.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IGPS.h>


namespace drone_mapper {

class MockMap3D : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D& pos), (const, override));
    MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D& pos), (const, override));
};

class MockGPSImpl : public IGPS {
public:
    explicit MockGPSImpl(Position3D p, Orientation h) : pos_(p), head_(h) {}
    Position3D position() const override { return pos_; }
    Orientation heading() const override { return head_; }
private:
    Position3D pos_;
    Orientation head_;
};

TEST(MockLidar, RayMissesEmptyMap) {
    // Setup a mock map that always returns Unmapped/Empty
    MockMap3D map;
    // Provide a map config with reasonable resolution so MockLidar uses correct stepping
    types::MapConfig map_cfg;
    map_cfg.offset = Position3D{0.0 * cm, 0.0 * cm, 0.0 * cm};
    map_cfg.resolution = 1.0 * cm;
    EXPECT_CALL(map, getMapConfig()).WillRepeatedly(::testing::Return(map_cfg));
    EXPECT_CALL(map, isInBounds(testing::_)).WillRepeatedly(::testing::Return(true));

    // The mock map returns Empty for all voxels (no obstacles)
    EXPECT_CALL(map, atVoxel(testing::_)).WillRepeatedly(::testing::Return(types::VoxelOccupancy::Empty));

    Position3D origin{0.0 * cm, 0.0 * cm, 0.0 * cm};
    Orientation heading{0.0 * deg, 0.0 * deg};
    MockGPSImpl gps(origin, heading);

    types::LidarConfigData lidar_cfg;
    lidar_cfg.z_min = 10.0 * cm;
    lidar_cfg.z_max = 100.0 * cm;
    lidar_cfg.d = 2.5 * cm;
    lidar_cfg.fov_circles = 1;

    MockLidar lidar(lidar_cfg, map, gps);

    // Trace a beam straight ahead; with empty map, should return a distance >= z_max
    Orientation beam{0.0 * deg, 0.0 * deg};
    auto results = lidar.scan(beam);
    ASSERT_FALSE(results.empty());
    const auto& hit = results.front();
    EXPECT_GE(hit.distance.numerical_value_in(cm), lidar_cfg.z_max.numerical_value_in(cm));
}

    
    // end of namespace tests above
} // namespace drone_mapper

namespace drone_mapper {

TEST(MockLidar, Basic) {
    Position3D origin{0.0 * cm, 0.0 * cm, 0.0 * cm};
    Orientation heading{0.0 * deg, 0.0 * deg};
    MockGPSImpl gps(origin, heading);

    // Simple map that has an obstacle at x >= 50 cm
    class SimpleMap : public IMap3D {
    public:
        types::MapConfig getMapConfig() const override {
            types::MapConfig c; c.offset = Position3D{0.0 * cm, 0.0 * cm, 0.0 * cm}; c.resolution = 1.0 * cm; return c;
        }
        types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
            const double xcm = pos.x.numerical_value_in(cm);
            if (xcm >= 50.0) return types::VoxelOccupancy::Occupied;
            return types::VoxelOccupancy::Empty;
        }
    } map;

    types::LidarConfigData lidar_cfg;
    lidar_cfg.z_min = 1.0 * cm;
    lidar_cfg.z_max = 200.0 * cm;
    lidar_cfg.d = 2.5 * cm;
    lidar_cfg.fov_circles = 1;

    MockLidar lidar(lidar_cfg, map, gps);
    Orientation beam{0.0 * deg, 0.0 * deg};
    auto results = lidar.scan(beam);
    ASSERT_FALSE(results.empty());
    // Closest hit should be around 50 cm
    const auto& hit = results.front();
    EXPECT_NEAR(hit.distance.numerical_value_in(cm), 50.0, 5.0);
}

} // namespace drone_mapper
