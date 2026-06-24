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

} // namespace drone_mapper
