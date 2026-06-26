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

// ── Test 1: single-circle (center beam only), empty map → miss ───────────────
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

// ── Test 2: obstacle at 50 cm → hit ──────────────────────────────────────────
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

// ── Test 3: fov_circles > 1 produces more beams ──────────────────────────────
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

    types::LidarConfigData cfg1{1.0*cm, 50.0*cm, 2.5*cm, 1}; // 1 beam
    types::LidarConfigData cfg3{1.0*cm, 50.0*cm, 2.5*cm, 3}; // 1 + 4 + 16 = 21 beams
    MockLidar l1(cfg1, map, gps);
    MockLidar l3(cfg3, map, gps);

    const auto r1 = l1.scan(Orientation{0.0*deg, 0.0*deg});
    const auto r3 = l3.scan(Orientation{0.0*deg, 0.0*deg});
    EXPECT_EQ(r1.size(), 1U);
    EXPECT_GT(r3.size(), r1.size());
}

// ── Test 4: fov_circles = 0 → no beams ───────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
// Staff hint: "a bug can be introduced where rays travel only 2/3rds of z_max".
// Test 5: obstacle AT z_max → must hit.  If bug shortens rays to 2/3*z_max
//         (e.g. 60 cm instead of 90 cm), the wall at 90 cm is missed → FAIL.
// Test 6: obstacle just BEYOND z_max → must miss (beam already stopped).
// ─────────────────────────────────────────────────────────────────────────────

TEST(MockLidar, DetectsObstacleAtExactZMax) {
    const double z_max_cm = 90.0;

    class WallAtZMax : public IMap3D {
    public:
        explicit WallAtZMax(double w) : wall_(w) {}
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D& p) const override {
            return p.x.numerical_value_in(cm) >= wall_
                       ? types::VoxelOccupancy::Occupied
                       : types::VoxelOccupancy::Empty;
        }
    private: double wall_;
    } map(z_max_cm);

    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{1.0*cm, z_max_cm*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    ASSERT_FALSE(results.empty());
    EXPECT_LE(results.front().distance.numerical_value_in(cm), z_max_cm + 1.0)
        << "Ray missed wall at z_max — possibly shortened (2/3 bug?)";
    EXPECT_GE(results.front().distance.numerical_value_in(cm), z_max_cm - 2.0)
        << "Hit reported far too early for a wall at z_max";
}

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
    // No occupied voxel within range — distance should be at z_max (beam exhausted).
    EXPECT_GE(results.front().distance.numerical_value_in(cm), z_max_cm - 1.0)
        << "Beam reported hit before z_max on empty path — false positive";
}

} // namespace drone_mapper
