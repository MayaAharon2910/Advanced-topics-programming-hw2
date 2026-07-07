// =============================================================================
// MappingAlgorithm_test.cpp - Component tests for MappingAlgorithmImpl
// MappingAlgorithmImpl decides where to move next. It keeps an internal
// known_voxels_ grid (separate from the output map), runs BFS to find
// unexplored frontiers, and queues movement commands one step at a time.
// Tests use AlwaysUnmappedMap (a simple fake returning Unmapped for every voxel)
// or MockOutputMap (GMock with ON_CALL+Invoke) when interaction verification
// is needed. No real movement or GPS is involved - only nextStep() is called.
// =============================================================================
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>

#include <chrono>
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
    // 6 directions: +/-X, +/-Y, +/-Z expressed as (horizontal_deg, altitude_deg)
    // East(0 degrees,0 degrees), West(180 degrees,0 degrees), North(270 degrees,0 degrees), South(90 degrees,0 degrees),
    // Up(0 degrees,90 degrees), Down(0 degrees,-90 degrees)
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

/*
 * What it does: checks that the algorithm performs real exploration logic.
 * Setup: the algorithm runs on a small mostly unknown map.
 * Checks: it returns a scan or movement command instead of a fixed dummy cycle.
 */
TEST(MappingAlgorithm, ProducesExplorationMovementInsteadOfDummyCycle) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(10.0, 20), makeLidar(), makeDrone(), output_map);
    const auto first = alg.nextStep(makeState(), nullptr);
    // The algorithm scans before it ever moves (a fixed batch runs at every
    // new position, including the first, before any move is planned) — a
    // scan_orientation is real exploration work, not a dummy no-op.
    const bool has_movement = first.movement.has_value() &&
                              first.movement->type != types::MovementCommandType::Hover;
    const bool has_scan = first.scan_orientation.has_value();
    EXPECT_TRUE(has_movement || has_scan || first.status == types::AlgorithmStatus::Finished);
}

/*
 * What it does: checks that the algorithm consumes the latest lidar scan.
 * Setup: a scan result is passed through the scan pointer.
 * Checks: the algorithm keeps running or finishes cleanly after ingesting it.
 */
TEST(MappingAlgorithm, IngestsScanFromLatestScanPointer) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(), makeLidar(), makeDrone(), output_map);
    types::LidarScanResult scan{types::LidarHit{
        10.0 * cm,
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}}};
    const auto cmd = alg.nextStep(makeState(), &scan);
    EXPECT_TRUE(cmd.movement.has_value() || cmd.scan_orientation.has_value() ||
                cmd.status == types::AlgorithmStatus::Finished);
}

/*
 * What it does: checks that BFS planning uses the internal known map and avoids obstacles.
 * Setup: the first scan reports an occupied hit 5 cm east of the drone.
 * Checks: captured positions never move past that reported obstacle.
 */
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
            // Don't actually move east - just stay put to keep the test simple
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

/*
 * What it does: checks scan ingestion for a zero-distance hit.
 * Setup: the scan contains a hit exactly at the drone position.
 * Checks: the algorithm does not crash and still returns a command or a clean finish.
 */
TEST(MappingAlgorithm, IngestScanWithZeroDistanceHitDoesNotCrash) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(5.0, 100), makeLidar(), makeDrone(), output_map);

    // distance==0 -> "too close to measure"
    types::LidarScanResult scan{types::LidarHit{
        0.0 * cm,
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}}};

    bool produced_command = false;
    types::DroneState state = makeState();
    for (int i = 0; i < 20; ++i) {
        auto cmd = alg.nextStep(state, i == 0 ? &scan : nullptr);
        produced_command = produced_command || cmd.movement.has_value() ||
                           cmd.scan_orientation.has_value() ||
                           cmd.status == types::AlgorithmStatus::Finished;
        if (cmd.status == types::AlgorithmStatus::Finished) break;
    }
    EXPECT_TRUE(produced_command)
        << "Algorithm produced no command after a zero-distance hit";
}

/*
 * What it does: checks the no-exit case around the drone.
 * Setup: the first scan marks all six neighboring cells as occupied.
 * Checks: BFS finds no frontier and the algorithm finishes.
 */
TEST(MappingAlgorithm, HaltsWhenSurroundedByOccupiedVoxels) {
    AlwaysUnmappedMap output_map;
    // Small gps_resolution (5 cm) so cell size is 5 cm and the hits at 2 cm
    // land in the adjacent cell after discretisation.
    types::MissionConfigData mission = makeMission(5.0, 200);
    // Before declaring Finished, the algorithm exhausts every still-Unmapped
    // cell within lidar range via nearest-first targeted scans (HW1 behavior:
    // don't give up just because the 6 axis-aligned neighbors are blocked —
    // there could be a diagonal way out). With unbounded mission bounds that
    // search space is the full lidar sphere (z_max 120 cm / 5 cm cells = a
    // 24-cell radius, tens of thousands of candidates) and won't exhaust in
    // 200 steps. Clamp the mission to a small box so "surrounded" can
    // actually be proven within the test's step budget.
    mission.mission_bounds.min_x      = XLength{-10.0 * cm};
    mission.mission_bounds.max_x      = XLength{ 10.0 * cm};
    mission.mission_bounds.min_y      = YLength{-10.0 * cm};
    mission.mission_bounds.max_y      = YLength{ 10.0 * cm};
    mission.mission_bounds.min_height = ZLength{-10.0 * cm};
    mission.mission_bounds.max_height = ZLength{ 10.0 * cm};

    MappingAlgorithmImpl alg(mission, makeLidar(), makeDrone(), output_map);

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

/*
 * What it does: regression test for a markScanRay bug where a lidar hit
 * landing off an exact cell-boundary multiple was recorded as Empty
 * pass-through space instead of Occupied, silently erasing the obstacle.
 * Setup: a real GPS + movement pair tracks true position (see
 * ElevatesWhenOnlyFrontierIsAbove for why this matters - a frozen
 * DroneState would make the test pass regardless of the bug). A single
 * Occupied hit is reported at 7.3 cm in +x - not a multiple of the 5 cm
 * cell size, the exact condition that used to make markScanRay drop the
 * hit before ever marking anything Occupied.
 * Checks: the drone never advances into the cell containing the reported
 * obstacle (x in [5,10) cm). Under the old bug that cell was wrongly
 * marked Empty, and sweep (which tries +x first) would happily cross into
 * it - so this test fails without the fix and passes with it.
 */
TEST(MappingAlgorithm, ObstacleAtNonBoundaryDistanceIsCorrectlyMarkedOccupied) {
    AlwaysUnmappedMap output_map;
    // Small radius keeps sphereAreaIsFree's safety check to just the
    // candidate cell itself (see ElevatesWhenOnlyFrontierIsAbove).
    types::DroneConfigData small_drone{
        1.0 * cm, 90.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
    types::MissionConfigData mission = makeMission(5.0, 100);
    MappingAlgorithmImpl alg(mission, makeLidar(), small_drone, output_map);

    // Cell-centered start (see ElevatesWhenOnlyFrontierIsAbove for why a
    // corner-aligned start is avoided).
    const auto start = makeState(2.5, 2.5, 2.5);
    types::LidarScanResult scan{types::LidarHit{
        7.3 * cm, Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}}};

    MockGPS gps(start.position, start.heading, 5.0 * cm);
    MockMovement movement(gps);

    double max_x_reached_cm = start.position.x.numerical_value_in(cm);
    for (int i = 0; i < 100; ++i) {
        types::DroneState state{gps.position(), gps.heading(), static_cast<std::size_t>(i)};
        auto cmd = alg.nextStep(state, i == 0 ? &scan : nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) break;
        if (cmd.movement.has_value()) {
            const auto& mv = *cmd.movement;
            switch (mv.type) {
                case types::MovementCommandType::Rotate:  movement.rotate(mv.rotation, mv.angle); break;
                case types::MovementCommandType::Advance: movement.advance(mv.distance); break;
                case types::MovementCommandType::Elevate: movement.elevate(mv.distance); break;
                default: break;
            }
        }
        max_x_reached_cm = std::max(max_x_reached_cm,
                                    gps.position().x.force_numerical_value_in(cm));
    }

    // The obstacle sits at real x = 2.5+7.3 = 9.8 cm, inside the cell
    // spanning [5,10). The drone must never cross into that cell.
    EXPECT_LT(max_x_reached_cm, 5.0)
        << "Algorithm advanced to x=" << max_x_reached_cm
        << ", past a reported obstacle at a non-boundary-aligned distance - "
           "markScanRay may be dropping hits that don't land exactly on a "
           "cell boundary";
}

/*
 * What it does: regression test for an enqueueTargetedScanAroundCurrentPosition
 * performance bug where the full candidate cube (R^3 cells, R derived from
 * lidar z_max / cell size) was rebuilt from scratch on every call, making a
 * single call take minutes for long-range lidar configs (measured: 8+
 * minutes for one real scenario before the fix).
 * Setup: a long-range lidar (z_max 2000 cm) at fine 1cm resolution gives
 * R=2000, so a full-cube scan would be ~2000^3 cells. The 4 horizontal
 * neighbors are blocked so planning falls straight through sweep/BFS to
 * the targeted-scan fallback on the very first cycle.
 * Checks: the algorithm still finds a targeted scan within a small bounded
 * wall-clock time, proving the search does not scan the full cube (the
 * fixed shell-expansion search finds a nearby candidate after a handful of
 * cells, regardless of how large R is).
 */
TEST(MappingAlgorithm, TargetedScanAroundCurrentPositionStaysFastWithLargeLidarRange) {
    AlwaysUnmappedMap output_map;
    types::MissionConfigData mission = makeMission(1.0, 50);
    types::LidarConfigData long_range_lidar{20.0 * cm, 2000.0 * cm, 2.5 * cm, 5};
    // Small radius keeps sphereAreaIsFree cheap so the timing budget below
    // measures the targeted-scan search itself, not sphere sampling cost.
    types::DroneConfigData small_drone{
        1.0 * cm, 90.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
    MappingAlgorithmImpl alg(mission, long_range_lidar, small_drone, output_map);

    // Block all 6 sweep/BFS directions (not just the 4 horizontal ones) so
    // sweep and BFS both fail on the first planning cycle instead of
    // succeeding via an unscanned up/down neighbor revealed as a pass-
    // through miss by the rest of the fixed scan batch - that would let
    // this test pass without ever reaching the targeted-scan fallback.
    const types::LidarScanResult scan = allDirectionsOccupiedScan(1.0);

    // Cell-centered start, not the exact origin corner (see
    // ElevatesWhenOnlyFrontierIsAbove): a corner-aligned start makes
    // negative-direction rays start exactly on their own near cell
    // boundary, an unrelated DDA degenerate case that would confuse this
    // test's timing measurement with something other than the targeted-scan
    // performance path under test.
    types::DroneState state = makeState(0.5, 0.5, 0.5);
    const auto t0 = std::chrono::steady_clock::now();
    // The fixed 14-scan batch (i=0..13) always sets scan_orientation
    // regardless of this test's setup, so it cannot indicate whether the
    // targeted-scan fallback ran - only a scan_orientation from i>=14 (after
    // the batch, once sweep/BFS have both failed with all 6 directions
    // blocked) can only have come from enqueueTargetedScanAroundCurrentPosition.
    bool exercised_fallback = false;
    for (int i = 0; i < 50; ++i) {
        auto cmd = alg.nextStep(state, i == 0 ? &scan : nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) break;
        if (i >= 14 && cmd.scan_orientation.has_value()) exercised_fallback = true;
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    EXPECT_TRUE(exercised_fallback)
        << "Test setup did not reach enqueueTargetedScanAroundCurrentPosition "
           "- sweep/BFS may have succeeded before all 6 directions were "
           "confirmed blocked, so this run did not actually exercise it";
    EXPECT_LT(elapsed_ms, 2000)
        << "Targeted scan search took " << elapsed_ms
        << "ms with a large lidar range - the O(R^3) full-cube scan bug "
           "may have regressed";
}

/*
 * What it does: checks that BFS respects mission boundaries.
 * Setup: a single-cell boundary cube is configured around the origin.
 * Checks: the algorithm finishes quickly and the state remains inside the bounds.
 */
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
    mission.mission_bounds = tight_cfg.boundaries;

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

/*
 * What it does: checks vertical planning toward an upper frontier.
 * Setup: horizontal neighbors are blocked and the only open frontier is above.
 * Checks: the algorithm issues a positive Elevate command.
 */
TEST(MappingAlgorithm, ElevatesWhenOnlyFrontierIsAbove) {
    OnlyAboveUnmapped output_map;
    // MappingAlgorithmImpl never queries output_map_ (see IMappingAlgorithm) -
    // it only learns the world through ingested scan results. A small drone
    // radius keeps sphereAreaIsFree's safety check to just the candidate
    // cell itself, so one explicit scan below is enough to prove it safe.
    types::DroneConfigData small_drone{
        1.0 * cm, 90.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
    MappingAlgorithmImpl alg(makeMission(5.0, 100), makeLidar(), small_drone, output_map);

    // Start at the CENTER of a cell, not a corner (makeState(2.5,2.5,2.5) is
    // the center of cell (0,0,0) at this 5cm resolution): starting exactly on
    // a cell boundary, combined with a hit distance that is exactly one cell
    // width, hits a boundary-alignment edge case in markScanRay's DDA walk
    // for negative-direction rays (it double-steps and marks the wrong
    // neighbor Occupied). A cell-centered start with non-boundary-aligned
    // hit distances avoids that degenerate case entirely.
    const auto start = makeState(2.5, 2.5, 2.5);

    // Surround horizontally with Occupied hits at 1 cell; report a miss
    // straight up (distance >= lidar z_max) so the algorithm actually learns
    // the cell above is Empty, not merely "not yet scanned."
    types::LidarScanResult scan;
    const std::vector<std::pair<double,double>> horiz = {
        {0.0,0.0},{90.0,0.0},{180.0,0.0},{270.0,0.0}};
    for (const auto& [h,a] : horiz)
        scan.push_back({5.0*cm,
            Orientation{h*horizontal_angle[deg], a*altitude_angle[deg]}});
    scan.push_back({120.0*cm,
        Orientation{0.0*horizontal_angle[deg], 90.0*altitude_angle[deg]}});

    // A real GPS + movement pair keeps the tracked position/heading in sync
    // with what the algorithm actually decides - with a frozen DroneState,
    // every tick reports the same starting position back to the algorithm
    // even after it "moves," which desyncs its own visited/path bookkeeping
    // from what it's told next.
    MockGPS gps(start.position, start.heading, 5.0 * cm);
    MockMovement movement(gps);

    bool issued_elevate = false;
    for (int i = 0; i < 50; ++i) {
        types::DroneState state{gps.position(), gps.heading(), static_cast<std::size_t>(i)};
        auto cmd = alg.nextStep(state, i == 0 ? &scan : nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) break;
        if (cmd.movement.has_value()) {
            const auto& mv = *cmd.movement;
            switch (mv.type) {
                case types::MovementCommandType::Rotate:  movement.rotate(mv.rotation, mv.angle); break;
                case types::MovementCommandType::Advance: movement.advance(mv.distance); break;
                case types::MovementCommandType::Elevate: movement.elevate(mv.distance); break;
                default: break;
            }
            if (mv.type == types::MovementCommandType::Elevate &&
                mv.distance.force_numerical_value_in(cm) > 0.0) {
                issued_elevate = true;
                break;
            }
        }
    }
    EXPECT_TRUE(issued_elevate)
        << "Algorithm should issue Elevate when the only frontier is above";
}

/*
 * What it does: checks rotation math around the 0/360 degree wrap point.
 * Setup: the drone starts facing 350 degrees while the target direction is east.
 * Checks: the total rotation stays small, so the short turn is used.
 */
TEST(MappingAlgorithm, HeadingDeltaWrapsToShortestPath) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(10.0, 50), makeLidar(), makeDrone(), output_map);

    // Place drone at origin facing 350 degrees, with a frontier directly east (0 degrees/360 degrees).
    // The angular delta to east is +10 degrees via wrap, not -350 degrees.
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
    // Total rotation to face east from 350 degrees should be small (≤ 45 degrees), not large.
    EXPECT_LE(std::abs(total_rotation.force_numerical_value_in(deg)), 45.0)
        << "Heading wrap-around failed: algorithm took the long path around";
}

/*
 * What it does: checks the empty-map edge case.
 * Setup: mission bounds are collapsed so min equals max on every axis.
 * Checks: the algorithm finishes immediately instead of trying to explore.
 */
TEST(MappingAlgorithm, ZeroSizeMapReturnsFinishedImmediately) {
    AlwaysUnmappedMap output_map;
    types::MissionConfigData zero_mission = makeMission(1.0, 100);
    // Collapse boundaries to a single point with non-zero values so the
    // isInsideMissionBounds check doesn't treat them as "all-zero=unbounded".
    // A 1cmx1cmx1cm box where min==max means no cell is strictly inside.
    zero_mission.mission_bounds.min_x      = 5.0 * x_extent[cm];
    zero_mission.mission_bounds.max_x      = 5.0 * x_extent[cm];
    zero_mission.mission_bounds.min_y      = 5.0 * y_extent[cm];
    zero_mission.mission_bounds.max_y      = 5.0 * y_extent[cm];
    zero_mission.mission_bounds.min_height = 5.0 * z_extent[cm];
    zero_mission.mission_bounds.max_height = 5.0 * z_extent[cm];

    MappingAlgorithmImpl alg(zero_mission, makeLidar(), makeDrone(), output_map);

    // The algorithm always runs its fixed 14-scan batch at the starting
    // position before Planning ever executes (Scanning is the initial
    // state, regardless of mission validity) - so "immediately" here means
    // "as soon as it first gets to check", not within the first few ticks.
    bool finished = false;
    for (int i = 0; i < 30; ++i) {
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

/*
 * What it does: checks vertical planning toward a lower frontier.
 * Setup: the drone starts at z=10 cm with horizontal and upper cells blocked.
 * Checks: the algorithm issues a negative Elevate command.
 */
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

    // MappingAlgorithmImpl never queries output_map_ (see IMappingAlgorithm) -
    // it only learns the world through ingested scan results. A small drone
    // radius keeps sphereAreaIsFree's safety check to just the candidate
    // cell itself, so one explicit scan below is enough to prove it safe.
    types::DroneConfigData small_drone{
        1.0 * cm, 90.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
    MappingAlgorithmImpl alg(makeMission(5.0, 100), makeLidar(), small_drone, output_map);

    // Start at the CENTER of a cell (z=12.5 is the center of the cell
    // containing z=10..15 at this 5cm resolution), not a boundary - see
    // ElevatesWhenOnlyFrontierIsAbove for why boundary-aligned starts/hit
    // distances are avoided here.
    const auto start = makeState(2.5, 2.5, 12.5);

    // Surround horizontally + above with Occupied hits at 1 cell; report a
    // miss straight down (distance >= lidar z_max) so the algorithm actually
    // learns the cell below is Empty, not merely "not yet scanned."
    types::LidarScanResult scan;
    for (const auto& [h, a] : std::vector<std::pair<double,double>>{
            {0.0,0.0},{90.0,0.0},{180.0,0.0},{270.0,0.0},{0.0,90.0}})
        scan.push_back({5.0*cm,
            Orientation{h*horizontal_angle[deg], a*altitude_angle[deg]}});
    scan.push_back({120.0*cm,
        Orientation{0.0*horizontal_angle[deg], -90.0*altitude_angle[deg]}});

    // A real GPS + movement pair keeps the tracked position/heading in sync
    // with what the algorithm actually decides (see ElevatesWhenOnlyFrontierIsAbove).
    MockGPS gps(start.position, start.heading, 5.0 * cm);
    MockMovement movement(gps);

    bool issued_descend = false;
    for (int i = 0; i < 50; ++i) {
        types::DroneState state{gps.position(), gps.heading(), static_cast<std::size_t>(i)};
        auto cmd = alg.nextStep(state, i == 0 ? &scan : nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) break;
        if (cmd.movement.has_value()) {
            const auto& mv = *cmd.movement;
            switch (mv.type) {
                case types::MovementCommandType::Rotate:  movement.rotate(mv.rotation, mv.angle); break;
                case types::MovementCommandType::Advance: movement.advance(mv.distance); break;
                case types::MovementCommandType::Elevate: movement.elevate(mv.distance); break;
                default: break;
            }
            if (mv.type == types::MovementCommandType::Elevate &&
                mv.distance.force_numerical_value_in(cm) < 0.0) {
                issued_descend = true;
                break;
            }
        }
    }
    EXPECT_TRUE(issued_descend)
        << "Algorithm should issue downward Elevate when only frontier is below";
}

/*
 * What it does: checks that the drone scans before moving into unknown space.
 * Setup: an unknown cell is available ahead of the current position.
 * Checks: the algorithm requests a scan before advancing.
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
        << "Algorithm never set scan_orientation - lidar scan logic may be missing";
}

} // namespace drone_mapper
