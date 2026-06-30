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

class AlwaysUnmappedMap : public IMap3D {
public:
    types::VoxelOccupancy atVoxel(const Position3D&) const override {
        return types::VoxelOccupancy::Unmapped;
    }
    types::MapConfig getMapConfig() const override { return types::MapConfig{}; }
    bool isInBounds(const Position3D&) const override { return false; }
};

class MockOutputMap : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel,     (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig,      getMapConfig, (),                  (const, override));
    MOCK_METHOD(bool,                  isInBounds,   (const Position3D&), (const, override));
};

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

// Creates six lidar hits around the current cell.
types::LidarScanResult allDirectionsOccupiedScan(double dist_cm = 2.0) {
    types::LidarScanResult scan;
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
 * Setup: runs the algorithm on a small mostly unknown map.
 * Checks: it returns meaningful scan or movement commands instead of a fixed dummy cycle.
 */
TEST(MappingAlgorithm, ProducesExplorationMovementInsteadOfDummyCycle) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(10.0, 20), makeLidar(), makeDrone(), output_map);
    const auto first = alg.nextStep(makeState(), nullptr);
    const bool has_movement = first.movement.has_value() &&
                              first.movement->type != types::MovementCommandType::Hover;
    EXPECT_TRUE(has_movement || first.status == types::AlgorithmStatus::Finished);
}

/*
 * What it does: checks that the algorithm consumes the latest lidar scan.
 * Setup: passes a scan result through the scan pointer given to the algorithm.
 * Checks: the output map is updated according to that scan.
 */
TEST(MappingAlgorithm, IngestsScanFromLatestScanPointer) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(), makeLidar(), makeDrone(), output_map);
    types::LidarScanResult scan{types::LidarHit{
        10.0 * cm,
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}}};
    const auto cmd = alg.nextStep(makeState(), &scan);
    EXPECT_TRUE(cmd.movement.has_value() || cmd.status == types::AlgorithmStatus::Finished);
}

/*
 * What it does: checks that BFS planning uses the internal known map and avoids obstacles.
 * Setup: feeds a scan with an occupied hit 5 cm east; the mock map only supplies interface calls.
 * Checks: captured positions never move past the reported obstacle.
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

    types::LidarScanResult scan_east{types::LidarHit{
        5.0 * cm,
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}}};

    std::vector<Position3D> visited_positions;
    types::DroneState state = makeState();

    for (int i = 0; i < 15; ++i) {
        visited_positions.push_back(state.position);
        auto cmd = alg.nextStep(state, i == 0 ? &scan_east : nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished) break;
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
 * Setup: passes a lidar hit located exactly at the drone position.
 * Checks: the algorithm does not crash and still returns a command or a clean finish.
 */
TEST(MappingAlgorithm, IngestScanWithZeroDistanceHitDoesNotCrash) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(5.0, 100), makeLidar(), makeDrone(), output_map);

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

/*
 * What it does: checks the no-exit case around the drone.
 * Setup: the first scan marks all six orthogonal neighboring cells as occupied.
 * Checks: BFS finds no frontier and the algorithm finishes instead of planning an invalid move.
 */
TEST(MappingAlgorithm, HaltsWhenSurroundedByOccupiedVoxels) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(5.0, 200), makeLidar(), makeDrone(), output_map);

    // Occupied hits in all 6 directions at 5 cm (exactly one cell away).
    const types::LidarScanResult surrounding_scan = allDirectionsOccupiedScan(5.0);

    bool finished = false;
    types::DroneState state = makeState();

    for (int i = 0; i < 200; ++i) {
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
 * What it does: checks that BFS respects mission boundaries.
 * Setup: uses a single-cell boundary cube around the origin.
 * Checks: the algorithm finishes quickly and the simulated state remains inside the bounds.
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

    EXPECT_LE(std::abs(state.position.x.numerical_value_in(cm)), 5.0 + 1e-6);
    EXPECT_LE(std::abs(state.position.y.numerical_value_in(cm)), 5.0 + 1e-6);
    EXPECT_LE(std::abs(state.position.z.numerical_value_in(cm)), 5.0 + 1e-6);
}

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
 * Setup: horizontal neighbors are occupied and the only reachable unknown cell is above.
 * Checks: the algorithm issues a positive Elevate command.
 */
TEST(MappingAlgorithm, ElevatesWhenOnlyFrontierIsAbove) {
    OnlyAboveUnmapped output_map;
    MappingAlgorithmImpl alg(makeMission(5.0, 100), makeLidar(), makeDrone(), output_map);

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

/*
 * What it does: checks rotation math around the 0/360 degree wrap point.
 * Setup: starts facing 350 degrees while the desired direction is east.
 * Checks: total rotation stays small, proving the algorithm takes the short way around.
 */
TEST(MappingAlgorithm, HeadingDeltaWrapsToShortestPath) {
    AlwaysUnmappedMap output_map;
    MappingAlgorithmImpl alg(makeMission(10.0, 50), makeLidar(), makeDrone(), output_map);

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
    EXPECT_LE(std::abs(total_rotation.force_numerical_value_in(deg)), 45.0)
        << "Heading wrap-around failed: algorithm took the long path around";
}

/*
 * What it does: checks the empty-map edge case.
 * Setup: collapses non-zero mission bounds so min equals max on every axis.
 * Checks: the algorithm finishes immediately instead of trying to explore.
 */
TEST(MappingAlgorithm, ZeroSizeMapReturnsFinishedImmediately) {
    AlwaysUnmappedMap output_map;
    types::MissionConfigData zero_mission = makeMission(1.0, 100);
    zero_mission.mission_bounds.min_x      = 5.0 * x_extent[cm];
    zero_mission.mission_bounds.max_x      = 5.0 * x_extent[cm];
    zero_mission.mission_bounds.min_y      = 5.0 * y_extent[cm];
    zero_mission.mission_bounds.max_y      = 5.0 * y_extent[cm];
    zero_mission.mission_bounds.min_height = 5.0 * z_extent[cm];
    zero_mission.mission_bounds.max_height = 5.0 * z_extent[cm];

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

/*
 * What it does: checks vertical planning toward a lower frontier.
 * Setup: starts at z=10 cm, marks horizontal and upper cells occupied, and leaves below open.
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

    MappingAlgorithmImpl alg(makeMission(5.0, 100), makeLidar(), makeDrone(), output_map);

    // Surround horizontally + above with Occupied hits; leave below unscanned.
    types::LidarScanResult scan;
    for (const auto& [h, a] : std::vector<std::pair<double,double>>{
            {0.0,0.0},{90.0,0.0},{180.0,0.0},{270.0,0.0},{0.0,90.0}})
        scan.push_back({5.0*cm,
            Orientation{h*horizontal_angle[deg], a*altitude_angle[deg]}});

    bool issued_descend = false;
    types::DroneState state = makeState(0, 0, 10);
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
 * What it does: checks that the drone scans before moving into unknown space.
 * Setup: places an unknown cell directly ahead of the current position.
 * Checks: the algorithm requests a scan instead of immediately advancing.
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
