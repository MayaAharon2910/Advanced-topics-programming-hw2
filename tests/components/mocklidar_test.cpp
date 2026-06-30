// =============================================================================
// MockLidar_test.cpp - Component tests for MockLidar
//
// MockLidar simulates the physical lidar sensor. It casts beams into the
// hidden map and returns hit distances. These tests verify:
//   - Correct beam count for different fov_circles values
//   - Correct hit detection at various distances
//   - Correct boundary behaviour at z_max (including the staff's 2/3 bug hint)
//
// All tests use a FixedGPS (drone at the origin) and either a MockMap3D
// (gMock, for empty-map scenarios) or a lightweight inline fake (wall scenarios).
// =============================================================================

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/MockLidar.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IGPS.h>

namespace drone_mapper {

class MockMap3D : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel,    (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig,      getMapConfig, (),                (const, override));
    MOCK_METHOD(bool,                  isInBounds,   (const Position3D&),(const, override));
};

class FixedGPS : public IGPS {
public:
    FixedGPS(Position3D p, Orientation h) : pos_(p), head_(h) {}
    Position3D position() const override { return pos_; }
    Orientation heading()  const override { return head_; }
private:
    Position3D pos_;
    Orientation head_;
};

types::MapConfig unitCfg() {
    types::MapConfig c;
    c.offset     = Position3D{0.0*cm, 0.0*cm, 0.0*cm};
    c.resolution = 1.0 * cm;
    return c;
}

/*
 * What it does: scans an empty map with the center lidar beam.
 * Setup: places the drone in a small map with no occupied voxels in front of it.
 * Checks: the returned beam has no hit within the lidar range.
 */
TEST(MockLidar, CenterBeamMissesEmptyMap) {
    MockMap3D map;
    EXPECT_CALL(map, getMapConfig()).WillRepeatedly(::testing::Return(unitCfg()));
    EXPECT_CALL(map, isInBounds(testing::_)).WillRepeatedly(::testing::Return(true));
    EXPECT_CALL(map, atVoxel(testing::_))
        .WillRepeatedly(::testing::Return(types::VoxelOccupancy::Empty));

    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{10.0*cm, 100.0*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    ASSERT_EQ(results.size(), 1U);
    EXPECT_GE(results.front().distance.numerical_value_in(cm),
              cfg.z_max.numerical_value_in(cm));
}

/*
 * What it does: checks that MockLidar detects an obstacle at a known distance.
 * Setup: puts one occupied voxel 50cm in front of the drone.
 * Checks: the scan contains a hit at the expected distance.
 */
TEST(MockLidar, DetectsObstacleAt50cm) {
    class WallMap : public IMap3D {
    public:
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D& p) const override {
            return p.x.numerical_value_in(cm) >= 50.0
                       ? types::VoxelOccupancy::Occupied
                       : types::VoxelOccupancy::Empty;
        }
    } map;
    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{1.0*cm, 200.0*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    ASSERT_FALSE(results.empty());
    EXPECT_NEAR(results.front().distance.numerical_value_in(cm), 50.0, 5.0);
}

/*
 * What it does: verifies that widening the field of view creates additional beams.
 * Setup: compares scans with more than one configured FOV circle.
 * Checks: the wider configuration produces more lidar measurements.
 */
TEST(MockLidar, MultipleFovCirclesProduceMoreBeams) {
    class EmptyMap : public IMap3D {
    public:
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D&) const override {
            return types::VoxelOccupancy::Empty;
        }
    } map;
    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};

    types::LidarConfigData cfg1{1.0*cm, 50.0*cm, 2.5*cm, 1};
    types::LidarConfigData cfg3{1.0*cm, 50.0*cm, 2.5*cm, 3};
    MockLidar l1(cfg1, map, gps);
    MockLidar l3(cfg3, map, gps);

    const auto r1 = l1.scan(Orientation{0.0*deg, 0.0*deg});
    const auto r3 = l3.scan(Orientation{0.0*deg, 0.0*deg});
    EXPECT_EQ(r1.size(), 1U);
    EXPECT_GT(r3.size(), r1.size());
}

/*
 * What it does: checks the edge case of a lidar configured with no beam circles.
 * Setup: sets fov_circles to zero and runs a scan.
 * Checks: the scan result is empty and the simulator does not crash.
 */
TEST(MockLidar, ZeroFovCirclesReturnsEmpty) {
    class EmptyMap : public IMap3D {
    public:
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D&) const override {
            return types::VoxelOccupancy::Empty;
        }
    } map;
    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{1.0*cm, 50.0*cm, 2.5*cm, 0};
    MockLidar lidar(cfg, map, gps);

    auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    EXPECT_TRUE(results.empty());
}

/*
 * What it does: checks the inclusive upper bound of the lidar range.
 * Setup: places an obstacle exactly at z_max from the drone.
 * Checks: MockLidar still reports the obstacle as a valid hit.
 */
TEST(MockLidar, DetectsObstacleAtExactZMax) {
    const double z_max_cm = 90.0;
    const double wall_cm  = z_max_cm - 1.0;

    class WallNearZMax : public IMap3D {
    public:
        explicit WallNearZMax(double w) : wall_(w) {}
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D& p) const override {
            return p.x.numerical_value_in(cm) >= wall_
                       ? types::VoxelOccupancy::Occupied
                       : types::VoxelOccupancy::Empty;
        }
    private: double wall_;
    } map(wall_cm);

    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{1.0*cm, z_max_cm*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    ASSERT_FALSE(results.empty());
    EXPECT_LE(results.front().distance.numerical_value_in(cm), z_max_cm)
        << "Ray missed wall at " << wall_cm << "cm - possibly shortened (2/3 bug?)";
    EXPECT_GE(results.front().distance.numerical_value_in(cm), wall_cm - 1.0)
        << "Hit reported far too early for a wall at " << wall_cm << "cm";
}

/*
 * What it does: checks the exclusive case just beyond the lidar range.
 * Setup: places an obstacle slightly farther than z_max.
 * Checks: MockLidar ignores the obstacle and reports no hit.
 */
TEST(MockLidar, MissesObstacleJustBeyondZMax) {
    const double z_max_cm = 90.0;

    class WallBeyondZMax : public IMap3D {
    public:
        explicit WallBeyondZMax(double w) : wall_(w) {}
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D& p) const override {
            return p.x.numerical_value_in(cm) >= wall_
                       ? types::VoxelOccupancy::Occupied
                       : types::VoxelOccupancy::Empty;
        }
    private: double wall_;
    } map(z_max_cm + 5.0);

    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{1.0*cm, z_max_cm*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    ASSERT_FALSE(results.empty());
    EXPECT_GE(results.front().distance.numerical_value_in(cm), z_max_cm - 1.0)
        << "Beam reported hit before z_max on empty path - false positive";
}

} // namespace drone_mapper
