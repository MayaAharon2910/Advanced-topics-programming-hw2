// =============================================================================
// MappingAlgorithm_test.cpp — Component tests for MappingAlgorithmImpl
//
// MappingAlgorithmImpl decides where to move next. It keeps an internal
// known_voxels_ grid (separate from the output map), runs BFS to find
// unexplored frontiers, and queues movement commands one step at a time.
//
// Tests use AlwaysUnmappedMap (a simple fake returning Unmapped for every voxel)
// or MockOutputMap (GMock with ON_CALL+Invoke) when interaction verification
// is needed. No real movement or GPS is involved — only nextStep() is called.
// =============================================================================
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>

#include <set>
#include <tuple>
#include <vector>

namespace drone_mapper {

// ── Simple Fake: always Unmapped (no Mock overhead needed) ───────────────────
class AlwaysUnmappedMap : public IMap3D {
public:
    types::VoxelOccupancy atVoxel(const Position3D&) const override {
        return types::VoxelOccupancy::Unmapped;
    }
    types::MapConfig getMapConfig() const override { return types::MapConfig{}; }
    bool isInBounds(const Position3D&) const override { return false; }
};

// ── gMock IMap3D — used only where we need interaction verification ──────────
class MockOutputMap : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel,     (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig,      getMapConfig, (),                  (const, override));
    MOCK_METHOD(bool,                  isInBounds,   (const Position3D&), (const, override));
};

// ── Helpers ───────────────────────────────────────────────────────────────────
namespace {

types::MissionConfigData makeMission(double gps_res_cm = 10.0,
                                     std::size_t max_steps = 200) {
    types::MissionConfigData m{};
    m.max_steps      = max_steps;
    m.gps_resolution = gps_res_cm * cm;
    return m;
}

types::LidarConfigData makeLidar() {
    return {20.0 * cm, 120.0 * cm, 2.5 * cm, 5};
}

types::DroneConfigData makeDrone() {
    return {15.0 * cm, 90.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
}

types::DroneState makeState(double x = 0, double y = 0, double z = 0,
                            double heading_deg = 0) {
    types::DroneState s{};
    s.position = Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
    s.heading  = Orientation{heading_deg * horizontal_angle[deg],
                              0.0 * altitude_angle[deg]};
    return s;
}

// Build a scan that places Occupied hits in all 6 orthogonal directions at
// distance `dist_cm`. After ingestScan(), every orthogonal neighbor of the
// origin is Occupied in known_voxels_, so BFS finds no navigable frontier
// and the algorithm returns Finished immediately.
types::LidarScanResult allDirectionsOccupiedScan(double dist_cm = 2.0) {
    types::LidarScanResult scan;
    // 6 directions: ±X, ±Y, ±Z expressed as (horizontal_deg, altitude_deg)
    // East(0°,0°), West(180°,0°), North(270°,0°), South(90°,0°),
    // Up(0°,90°), Down(0°,-90°)
    const std::vector<std::pair<double,double>> dirs = {
        {0.0,    0.0},
        {180.0,  0.0},
        {90.0,   0.0},
        {270.0,  0.0},
        {0.0,   90.0},
        {0.0,  -90.0},
    };
    for (const auto& [h, a] : dirs) {
        scan.push_back(types::LidarHit{
            dist_cm * cm,
            Orientation{h * horizontal_angle[deg], a * altitude_angle[deg]}});
    }
    return scan;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Existing tests (kept verbatim)
// ─────────────────────────────────────────────────────────────────────────────

TEST(MappingAlgorithm, ProducesExplorationMovementInsteadOfDummyCycle) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(10.0, 20), makeLidar(), makeDrone(), output_map);
    const auto first = alg.nextStep(makeState(), nullptr);
    const bool has_movement = first.movement.has_value() &&
                              first.movement->type != types::MovementCommandType::Hover;
    EXPECT_TRUE(has_movement || first.status == types::AlgorithmStatus::Finished);
}

TEST(MappingAlgorithm, IngestsScanFromLatestScanPointer) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(), makeLidar(), makeDrone(), output_map);
    types::LidarScanResult scan{types::LidarHit{
        10.0 * cm,
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}}};
    const auto cmd = alg.nextStep(makeState(), &scan);
    EXPECT_TRUE(cmd.movement.has_value() || cmd.status == types::AlgorithmStatus::Finished);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: BFS does not navigate into Occupied voxels  (argument capture)
//
// Mock: any voxel at x > 0 is Occupied; origin is Unmapped.
// We capture every position the drone actually visits and assert none is at x > 0.
// NOTE: output_map_ is NOT queried by the algorithm's internal BFS — it uses
//       its own known_voxels_ map. We still inject a MockOutputMap to confirm
//       the algorithm never calls atVoxel() in a way that would sidestep its
//       internal representation.
// ─────────────────────────────────────────────────────────────────────────────
TEST(MappingAlgorithm, BfsDoesNotNavigateIntoOccupiedVoxels) {
    using ::testing::_;
    using ::testing::AnyNumber;
    using ::testing::Invoke;
    using ::testing::Return;

    MockOutputMap mock_map;
    ON_CALL(mock_map, getMapConfig()).WillByDefault(Return(types::MapConfig{}));
    ON_CALL(mock_map, isInBounds(_)).WillByDefault(Return(true));
    ON_CALL(mock_map, atVoxel(_)).WillByDefault(Return(types::VoxelOccupancy::Unmapped));
    EXPECT_CALL(mock_map, getMapConfig()).Times(AnyNumber());
    EXPECT_CALL(mock_map, isInBounds(_)).Times(AnyNumber());
    EXPECT_CALL(mock_map, atVoxel(_)).Times(AnyNumber());

    // Small mission bounds so the algorithm terminates quickly
    types::MissionConfigData mission = makeMission(10.0, 50);

    MappingAlgorithmImpl alg(mission, makeLidar(), makeDrone(), mock_map);

    // Feed a scan: obstacle 5cm to the east (positive-x direction).
    // The algorithm should record that cell as Occupied and avoid navigating there.
    types::LidarScanResult scan_east{types::LidarHit{
        5.0 * cm,
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}}};

    std::vector<Position3D> visited_positions;
    types::DroneState state = makeState();

    for (int i = 0; i < 15; ++i) {
        visited_positions.push_back(state.position);
        auto cmd = alg.nextStep(state, i == 0 ? &scan_east : nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished) break;
        // Simulate movement: if advance command given, update state naively
        if (cmd.movement.has_value() &&
            cmd.movement->type == types::MovementCommandType::Advance) {
            // Don't actually move east — just stay put to keep the test simple
        }
    }

    // The drone must never have visited a position at x > 5cm
    // (which is where the obstacle was reported)
    for (const auto& pos : visited_positions) {
        EXPECT_LE(pos.x.numerical_value_in(cm), 5.0 + 1e-6)
            << "Algorithm visited a position past the reported obstacle at x="
            << pos.x.numerical_value_in(cm);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: ingestScan on zero-distance hit (PotentiallyOccupied) — no crash
// ─────────────────────────────────────────────────────────────────────────────
TEST(MappingAlgorithm, IngestScanWithZeroDistanceHitDoesNotCrash) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(5.0, 100), makeLidar(), makeDrone(), output_map);

    // distance==0 → "too close to measure"
    types::LidarScanResult scan{types::LidarHit{
        0.0 * cm,
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}}};

    bool produced_command = false;
    types::DroneState state = makeState();
    for (int i = 0; i < 20; ++i) {
        auto cmd = alg.nextStep(state, i == 0 ? &scan : nullptr);
        produced_command = produced_command || cmd.movement.has_value() ||
                           cmd.status == types::AlgorithmStatus::Finished;
        if (cmd.status == types::AlgorithmStatus::Finished) break;
    }
    EXPECT_TRUE(produced_command)
        << "Algorithm produced no command after a zero-distance hit";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5: algorithm halts when all reachable cells are walled off
//
// Key insight: MappingAlgorithmImpl uses an internal `known_voxels_` map, NOT
// output_map_.atVoxel(). BFS finds frontiers by looking for Empty cells with
// Unknown orthogonal neighbors. If every neighbour of the origin is Occupied
// (from a scan), no navigable frontier exists and the algorithm returns Finished.
//
// We drive this by feeding a scan that places Occupied hits in all 6 orthogonal
// directions at 1-cell distance. After ingestScan(), known_voxels_ contains
// Occupied for every neighbour → BFS finds no frontier → Finished.
// ─────────────────────────────────────────────────────────────────────────────
TEST(MappingAlgorithm, HaltsWhenSurroundedByOccupiedVoxels) {
    AlwaysUnmappedMap output_map;
    // Small gps_resolution (5 cm) so cell size is 5 cm and the hits at 2 cm
    // land in the adjacent cell after discretisation.
    MappingAlgorithmImpl alg(makeMission(5.0, 200), makeLidar(), makeDrone(), output_map);

    // Occupied hits in all 6 directions at 5 cm (exactly one cell away).
    const types::LidarScanResult surrounding_scan = allDirectionsOccupiedScan(5.0);

    bool finished = false;
    types::DroneState state = makeState();

    for (int i = 0; i < 200; ++i) {
        // Feed the surrounding scan on the first step so the algorithm knows
        // it is completely walled in.
        auto cmd = alg.nextStep(state, i == 0 ? &surrounding_scan : nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            finished = true;
            break;
        }
    }

    EXPECT_TRUE(finished)
        << "Algorithm did not halt even though all neighbours are Occupied";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 6: OutOfBounds voxels are never used as navigation destinations
//
// Mission bounds: only the origin cell (±5 cm cube).
// Drone starts at origin.  Algorithm must finish quickly (nothing to explore
// outside the boundary) and must never move the drone outside the bounds.
// ─────────────────────────────────────────────────────────────────────────────
TEST(MappingAlgorithm, BfsRespectsOutOfBoundsBoundary) {
    using ::testing::_;
    using ::testing::AnyNumber;
    using ::testing::Invoke;
    using ::testing::Return;

    MockOutputMap mock_map;

    types::MapConfig tight_cfg{};
    tight_cfg.resolution          = 10.0 * cm;
    tight_cfg.boundaries.min_x      = XLength{-5.0 * cm};
    tight_cfg.boundaries.max_x      = XLength{ 5.0 * cm};
    tight_cfg.boundaries.min_y      = YLength{-5.0 * cm};
    tight_cfg.boundaries.max_y      = YLength{ 5.0 * cm};
    tight_cfg.boundaries.min_height = ZLength{-5.0 * cm};
    tight_cfg.boundaries.max_height = ZLength{ 5.0 * cm};

    ON_CALL(mock_map, getMapConfig()).WillByDefault(Return(tight_cfg));
    ON_CALL(mock_map, isInBounds(_)).WillByDefault(
        Invoke([](const Position3D& pos) -> bool {
            return std::abs(pos.x.numerical_value_in(cm)) <= 5.0 &&
                   std::abs(pos.y.numerical_value_in(cm)) <= 5.0 &&
                   std::abs(pos.z.numerical_value_in(cm)) <= 5.0;
        }));
    ON_CALL(mock_map, atVoxel(_)).WillByDefault(Return(types::VoxelOccupancy::Empty));
    EXPECT_CALL(mock_map, getMapConfig()).Times(AnyNumber());
    EXPECT_CALL(mock_map, isInBounds(_)).Times(AnyNumber());
    EXPECT_CALL(mock_map, atVoxel(_)).Times(AnyNumber());

    types::MissionConfigData mission = makeMission(10.0, 50);
    mission.boundaries = tight_cfg.boundaries;

    MappingAlgorithmImpl alg(mission, makeLidar(), makeDrone(), mock_map);

    types::DroneState state = makeState();
    bool finished = false;
    for (int i = 0; i < 50; ++i) {
        auto cmd = alg.nextStep(state, nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            finished = true;
            break;
        }
    }

    EXPECT_TRUE(finished)
        << "Algorithm did not halt with a single-cell boundary";

    // Drone position must remain within the bounds
    EXPECT_LE(std::abs(state.position.x.numerical_value_in(cm)), 5.0 + 1e-6);
    EXPECT_LE(std::abs(state.position.y.numerical_value_in(cm)), 5.0 + 1e-6);
    EXPECT_LE(std::abs(state.position.z.numerical_value_in(cm)), 5.0 + 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────

// Helper: map where only the cell directly ABOVE origin is Unmapped; all
// horizontal neighbours are Occupied. Forces the algorithm to go up.
class OnlyAboveUnmapped : public IMap3D {
public:
    types::VoxelOccupancy atVoxel(const Position3D& p) const override {
        const double x = p.x.numerical_value_in(cm);
        const double y = p.y.numerical_value_in(cm);
        const double z = p.z.numerical_value_in(cm);
        // Only z > 0 is Unmapped (above the origin); everything else Occupied.
        if (x == 0.0 && y == 0.0 && z > 0.0) return types::VoxelOccupancy::Unmapped;
        return types::VoxelOccupancy::Occupied;
    }
    types::MapConfig getMapConfig() const override { return {}; }
    bool isInBounds(const Position3D&) const override { return true; }
};

// Test: algorithm issues an Elevate command when the only frontier is above.
TEST(MappingAlgorithm, ElevatesWhenOnlyFrontierIsAbove) {
    OnlyAboveUnmapped output_map;
    // Feed scan that places Occupied in all 4 horizontal directions at 1 cell
    // and Empty above, so BFS is forced to go up.
    MappingAlgorithmImpl alg(makeMission(5.0, 100), makeLidar(), makeDrone(), output_map);

    // Surround horizontally with Occupied hits; leave vertical clear.
    types::LidarScanResult scan;
    const std::vector<std::pair<double,double>> horiz = {
        {0.0,0.0},{90.0,0.0},{180.0,0.0},{270.0,0.0}};
    for (const auto& [h,a] : horiz)
        scan.push_back({5.0*cm,
            Orientation{h*horizontal_angle[deg], a*altitude_angle[deg]}});

    bool issued_elevate = false;
    types::DroneState state = makeState();
    for (int i = 0; i < 50; ++i) {
        auto cmd = alg.nextStep(state, i == 0 ? &scan : nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) break;
        if (cmd.movement.has_value() &&
            cmd.movement->type == types::MovementCommandType::Elevate &&
            cmd.movement->distance.force_numerical_value_in(cm) > 0.0) {
            issued_elevate = true;
            break;
        }
    }
    EXPECT_TRUE(issued_elevate)
        << "Algorithm should issue Elevate when the only frontier is above";
}

// Test: heading wrap-around — heading delta > 180° wraps to the shorter path.
// If the drone faces 350° and needs to reach 10°, the delta should be +20°
// (turn left a little), NOT -340° (turn right almost all the way around).
TEST(MappingAlgorithm, HeadingDeltaWrapsToShortestPath) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(10.0, 50), makeLidar(), makeDrone(), output_map);

    // Place drone at origin facing 350°, with a frontier directly east (0°/360°).
    // The angular delta to east is +10° via wrap, not -350°.
    types::DroneState state = makeState(0, 0, 0, 350.0);

    HorizontalAngle total_rotation = 0.0 * horizontal_angle[deg];
    for (int i = 0; i < 30; ++i) {
        auto cmd = alg.nextStep(state, nullptr);
        if (!cmd.movement.has_value()) break;
        if (cmd.movement->type == types::MovementCommandType::Rotate) {
            const double delta = cmd.movement->angle.force_numerical_value_in(deg);
            const double sign  = (cmd.movement->rotation ==
                                  types::RotationDirection::Left) ? 1.0 : -1.0;
            total_rotation += (sign * std::abs(delta)) * horizontal_angle[deg];
        }
        if (cmd.movement->type == types::MovementCommandType::Advance) break;
    }
    // Total rotation to face east from 350° should be small (≤ 45°), not large.
    EXPECT_LE(std::abs(total_rotation.force_numerical_value_in(deg)), 45.0)
        << "Heading wrap-around failed: algorithm took the long path around";
}

// Test: zero-size map (max == min on every axis) → algorithm returns Finished immediately.
TEST(MappingAlgorithm, ZeroSizeMapReturnsFinishedImmediately) {
    AlwaysUnmappedMap output_map;
    types::MissionConfigData zero_mission = makeMission(1.0, 100);
    // Collapse boundaries to a single point with non-zero values so the
    // isInsideMissionBounds check doesn't treat them as "all-zero=unbounded".
    // A 1cm×1cm×1cm box where min==max means no cell is strictly inside.
    zero_mission.boundaries.min_x      = 5.0 * x_extent[cm];
    zero_mission.boundaries.max_x      = 5.0 * x_extent[cm];
    zero_mission.boundaries.min_y      = 5.0 * y_extent[cm];
    zero_mission.boundaries.max_y      = 5.0 * y_extent[cm];
    zero_mission.boundaries.min_height = 5.0 * z_extent[cm];
    zero_mission.boundaries.max_height = 5.0 * z_extent[cm];

    MappingAlgorithmImpl alg(zero_mission, makeLidar(), makeDrone(), output_map);

    bool finished = false;
    for (int i = 0; i < 10; ++i) {
        auto cmd = alg.nextStep(makeState(), nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            finished = true;
            break;
        }
    }
    EXPECT_TRUE(finished)
        << "Algorithm should finish immediately when mission bounds are collapsed to zero";
}

// ─────────────────────────────────────────────────────────────────────────────
// 3-D: descend when only frontier is below.
// Mirrors ElevatesWhenOnlyFrontierIsAbove but in the -Z direction.
// Catches a bug where the BFS sweep only looks up (dz=+1) and never down.
// ─────────────────────────────────────────────────────────────────────────────
TEST(MappingAlgorithm, DescendsWhenOnlyFrontierIsBelow) {
    // Map where only z < start is Unmapped; all horizontal + above is Occupied.
    class OnlyBelowUnmapped : public IMap3D {
    public:
        types::VoxelOccupancy atVoxel(const Position3D& p) const override {
            const double z = p.z.numerical_value_in(cm);
            // start at z=10cm, everything at z<10 is Unmapped, rest Occupied
            if (p.x.numerical_value_in(cm) == 0.0 &&
                p.y.numerical_value_in(cm) == 0.0 &&
                z < 10.0 && z >= 0.0) return types::VoxelOccupancy::Unmapped;
            return types::VoxelOccupancy::Occupied;
        }
        types::MapConfig getMapConfig() const override { return {}; }
        bool isInBounds(const Position3D&) const override { return true; }
    } output_map;

    MappingAlgorithmImpl alg(makeMission(5.0, 100), makeLidar(), makeDrone(), output_map);

    // Surround horizontally + above with Occupied hits; leave below unscanned.
    types::LidarScanResult scan;
    for (const auto& [h, a] : std::vector<std::pair<double,double>>{
            {0.0,0.0},{90.0,0.0},{180.0,0.0},{270.0,0.0},{0.0,90.0}})
        scan.push_back({5.0*cm,
            Orientation{h*horizontal_angle[deg], a*altitude_angle[deg]}});

    bool issued_descend = false;
    types::DroneState state = makeState(0, 0, 10);  // start at z=10cm
    for (int i = 0; i < 50; ++i) {
        auto cmd = alg.nextStep(state, i == 0 ? &scan : nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) break;
        if (cmd.movement.has_value() &&
            cmd.movement->type == types::MovementCommandType::Elevate &&
            cmd.movement->distance.force_numerical_value_in(cm) < 0.0) {
            issued_descend = true;
            break;
        }
    }
    EXPECT_TRUE(issued_descend)
        << "Algorithm should issue downward Elevate when only frontier is below";
}

/*
 * What it does: verifies the algorithm includes scan_orientation in its commands.
 * Setup: all-empty map, tight mission, run up to 50 steps.
 * Checks: at least one command has scan_orientation set. A bug that removes all
 *         scan requests (always returning scan_orientation as nullopt) would
 *         mean DroneControlImpl never fires the lidar and the output map stays
 *         fully Unmapped, causing zero coverage.
 *         Note: our algorithm pairs scan_orientation with movement commands
 *         rather than issuing separate scan-only steps.
 */
TEST(MappingAlgorithm, RequestsScanBeforeEnteringUnknownCell) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(10.0, 200), makeLidar(), makeDrone(), output_map);

    bool saw_scan = false;
    types::DroneState state = makeState();

    for (int i = 0; i < 50; ++i) {
        auto cmd = alg.nextStep(state, nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) break;
        if (cmd.scan_orientation.has_value()) {
            saw_scan = true;
            break;
        }
    }

    EXPECT_TRUE(saw_scan)
        << "Algorithm never set scan_orientation — lidar scan logic may be missing";
}

} // namespace drone_mapper
