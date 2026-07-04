// =============================================================================
// MockLidar_test.cpp - Component tests for MockLidar
// MockLidar simulates the physical lidar sensor. It casts beams into the
// hidden map and returns hit distances. These tests verify:
//   - Correct beam count for different fov_circles values
//   - Correct hit detection at various distances
//   - Correct boundary behaviour at z_max (including the staff's 2/3 bug hint)
// All tests use a FixedGPS (drone at the origin) and either a MockMap3D
// (gMock, for empty-map scenarios) or a lightweight inline fake (wall scenarios).
// =============================================================================

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>

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
 * What it does: fires a single center beam into a completely empty map.
 * Setup: fov_circles=1 (one beam only), all voxels return Empty.
 * Checks: the scan returns exactly one result and its distance is >= z_max,
 *         confirming the beam reached the end without hitting anything.
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
 * What it does: places a solid wall at x=50cm and fires a beam east.
 * Setup: inline WallMap returns Occupied for x >= 50, Empty otherwise.
 * Checks: the reported hit distance is approximately 50cm (±5cm tolerance
 *         for the 0.1cm step size).
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
 * What it does: compares beam count between fov_circles=1 and fov_circles=3.
 * Setup: all-empty map, two lidars with different fov_circles.
 * Checks: fov_circles=1 produces exactly 1 beam; fov_circles=3 produces more,
 *         confirming the ring-expansion logic scales correctly.
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
 * What it does: configures a lidar with zero FOV circles and scans.
 * Setup: fov_circles=0, all-empty map.
 * Checks: the result vector is empty — no beams are fired when fov_circles=0.
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
 * What it does: places a wall 1cm before z_max to catch the staff's 2/3 bug.
 * Setup: wall at x=89cm, z_max=90cm. If a bug shortens rays to 2/3*z_max
 *        (60cm), the wall at 89cm is never reached and this test fails.
 *        The wall is placed 1cm inside z_max rather than exactly at z_max
 *        to avoid floating-point accumulation: 900 steps of 0.1cm reach
 *        ~89.999cm, not exactly 90.0cm.
 * Checks: hit distance is <= z_max (beam did not overshoot) and >= 88cm
 *         (beam did not stop too early).
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
 * What it does: places a thin wall exactly at the configured z_max endpoint.
 * Setup: the map reports Occupied only inside a tiny tolerance band around
 *        x=z_max, while the path before that is Empty. This specifically checks
 *        that traceBeam includes the final z_max sample and does not stop early.
 * Checks: the returned distance is approximately z_max rather than max-distance
 *         or a premature hit.
 */
TEST(MockLidar, DetectsThinObstacleAtZMaxEndpoint) {
    const double z_max_cm = 90.0;

    class ThinWallAtZMax : public IMap3D {
    public:
        explicit ThinWallAtZMax(double endpoint) : endpoint_(endpoint) {}
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D& p) const override {
            const double x = p.x.numerical_value_in(cm);
            return std::abs(x - endpoint_) <= 0.051
                       ? types::VoxelOccupancy::Occupied
                       : types::VoxelOccupancy::Empty;
        }
    private:
        double endpoint_;
    } map(z_max_cm);

    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{1.0*cm, z_max_cm*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    ASSERT_FALSE(results.empty());
    EXPECT_NEAR(results.front().distance.numerical_value_in(cm), z_max_cm, 0.11)
        << "The final z_max sample was not scanned or was reported inaccurately";
}

/*
 * What it does: places a wall 5cm beyond z_max and verifies the beam stops.
 * Setup: wall at x=95cm, z_max=90cm. The beam should exhaust at 90cm without
 *        hitting anything.
 * Checks: returned distance is >= z_max-1, confirming no false positive hit
 *         was reported for an obstacle beyond the beam's reach.
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

/*
 * What it does: places an obstacle closer than z_min and verifies the lidar
 *               reports a "too close" sentinel (distance == 0).
 * Setup: wall at x=5cm, z_min=20cm so the hit is inside the blind zone.
 * Checks: scan returns at least one result and its distance is exactly 0,
 *         which is the protocol for a hit that cannot be measured accurately.
 */
TEST(MockLidar, ObstacleCloserThanZMinReturnsZero) {
    class WallAt5cm : public IMap3D {
    public:
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D& p) const override {
            return p.x.numerical_value_in(cm) >= 5.0
                       ? types::VoxelOccupancy::Occupied
                       : types::VoxelOccupancy::Empty;
        }
    } map;
    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    // z_min=20cm, obstacle at ~5cm — inside the blind zone
    types::LidarConfigData cfg{20.0*cm, 200.0*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    ASSERT_FALSE(results.empty());
    EXPECT_DOUBLE_EQ(results.front().distance.numerical_value_in(cm), 0.0)
        << "Obstacle inside z_min blind zone must return distance 0";
}

/*
 * What it does: rotates the GPS heading to +Y (90 degrees = south) and
 *               verifies the center beam follows the drone heading.
 * Setup: obstacle at (0,50,0) — along the +Y axis; GPS heading=90deg.
 * Checks: scan returns a hit at ~50cm, confirming the beam direction is
 *         correctly derived from the drone's heading, not a fixed axis.
 */
TEST(MockLidar, HeadingRotatesBeamDirection) {
    class WallAlongY : public IMap3D {
    public:
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D& p) const override {
            // Wall at y >= 50, x and z near 0
            const double x = p.x.numerical_value_in(cm);
            const double y = p.y.numerical_value_in(cm);
            const double z = p.z.numerical_value_in(cm);
            return (std::abs(x) < 2.0 && y >= 50.0 && std::abs(z) < 2.0)
                       ? types::VoxelOccupancy::Occupied
                       : types::VoxelOccupancy::Empty;
        }
    } map;
    // Heading 90deg = facing +Y (south per the assignment's convention)
    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {90.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{1.0*cm, 200.0*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    ASSERT_FALSE(results.empty());
    EXPECT_NEAR(results.front().distance.numerical_value_in(cm), 50.0, 5.0)
        << "Heading-rotated beam should hit the +Y wall at ~50cm";
}

/*
 * What it does: steers the scan orientation 90 degrees relative to a heading
 *               of 0 (east), effectively looking south, and detects a +Y wall.
 * Setup: GPS heading=0 (east), scan orientation=90deg; wall at y>=50.
 * Checks: the steered beam hits the wall, confirming scan_orientation is
 *         added to the heading rather than overriding it.
 */
TEST(MockLidar, ScanOrientationSteersBeamRelativeToHeading) {
    class WallAlongY : public IMap3D {
    public:
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D& p) const override {
            const double x = p.x.numerical_value_in(cm);
            const double y = p.y.numerical_value_in(cm);
            const double z = p.z.numerical_value_in(cm);
            return (std::abs(x) < 2.0 && y >= 50.0 && std::abs(z) < 2.0)
                       ? types::VoxelOccupancy::Occupied
                       : types::VoxelOccupancy::Empty;
        }
    } map;
    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}}; // facing east
    types::LidarConfigData cfg{1.0*cm, 200.0*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    // Steer 90 degrees relative to heading — should now point south (+Y)
    const auto results = lidar.scan(Orientation{90.0*deg, 0.0*deg});
    ASSERT_FALSE(results.empty());
    EXPECT_NEAR(results.front().distance.numerical_value_in(cm), 50.0, 5.0)
        << "90-degree scan orientation must steer beam to +Y wall";
}

/*
 * What it does: verifies that fov_circles=1 produces exactly 1 beam (circle 0
 *               only — the central beam).
 * Setup: all-empty map, fov_circles=1.
 * Checks: exactly 1 scan result — no outer rings are emitted.
 */
TEST(MockLidar, OneFovCircleYieldsSingleBeam) {
    class EmptyMap : public IMap3D {
    public:
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D&) const override {
            return types::VoxelOccupancy::Empty;
        }
    } map;
    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{1.0*cm, 50.0*cm, 2.5*cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    EXPECT_EQ(results.size(), 1U)
        << "fov_circles=1 must produce exactly 1 beam (center only)";
}

/*
 * What it does: verifies that fov_circles=2 produces exactly 5 beams:
 *               1 center + 4 beams on the first outer ring.
 * Setup: all-empty map, fov_circles=2.
 * Checks: exactly 5 scan results — the outer ring multiplies by 4.
 */
TEST(MockLidar, TwoFovCirclesYieldFiveBeams) {
    class EmptyMap : public IMap3D {
    public:
        types::MapConfig getMapConfig() const override { return unitCfg(); }
        bool isInBounds(const Position3D&) const override { return true; }
        types::VoxelOccupancy atVoxel(const Position3D&) const override {
            return types::VoxelOccupancy::Empty;
        }
    } map;
    FixedGPS gps{{0.0*cm, 0.0*cm, 0.0*cm}, {0.0*deg, 0.0*deg}};
    types::LidarConfigData cfg{1.0*cm, 50.0*cm, 2.5*cm, 2};
    MockLidar lidar(cfg, map, gps);

    const auto results = lidar.scan(Orientation{0.0*deg, 0.0*deg});
    EXPECT_EQ(results.size(), 5U)
        << "fov_circles=2 must produce 1 center + 4 outer = 5 beams";
}

} // namespace drone_mapper
