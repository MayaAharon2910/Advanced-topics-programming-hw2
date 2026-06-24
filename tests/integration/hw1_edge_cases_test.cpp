// Integration tests derived from the 4 edge-case scenarios used in Assignment 1.
// Each hw1 scenario was converted to hw2 format:
//   - map_input.txt  → data_maps/hw1_caseN.npy  (map_resolution = 1 cm/voxel)
//   - mission_config.txt → inline MissionConfigData (boundaries + start position)
//   - drone_config.txt   → inline DroneConfigData + LidarConfigData
//
// All 4 original scenarios achieved score ≥ 86/100 in hw1.
// Here we assert score > 70 to give the hw2 algorithm reasonable slack
// (different discretization, gps_resolution ≥ 1 cm, fewer max_steps) while
// still catching regressions.

#include <gtest/gtest.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

namespace {

using namespace drone_mapper;
using namespace drone_mapper::types;

// ── Config builders ───────────────────────────────────────────────────────────

SimulationConfigData makeHw1SimConfig(const char* map_file,
                                      double start_x, double start_y, double start_z) {
    SimulationConfigData s;
    s.map_filename   = map_file;
    s.map_resolution = 1.0 * cm;      // hw1 maps are 1 cm/voxel
    s.map_offset     = Position3D{};
    // Start position must be a cell centre = multiple of gps_resolution (1 cm).
    s.initial_drone_position = Position3D{
        start_x * x_extent[cm],
        start_y * y_extent[cm],
        start_z * z_extent[cm]};
    s.initial_angle = 0.0 * horizontal_angle[deg];
    return s;
}

MissionConfigData makeHw1Mission(double min_x, double max_x,
                                 double min_y, double max_y,
                                 double min_h, double max_h,
                                 std::size_t max_steps = 5000) {
    MissionConfigData m;
    m.max_steps                        = max_steps;
    m.gps_resolution                   = 1.0 * cm;
    m.output_mapping_resolution_factor = 1;
    m.boundaries.min_x      = XLength{min_x * cm};
    m.boundaries.max_x      = XLength{max_x * cm};
    m.boundaries.min_y      = YLength{min_y * cm};
    m.boundaries.max_y      = YLength{max_y * cm};
    m.boundaries.min_height = ZLength{min_h * cm};
    m.boundaries.max_height = ZLength{max_h * cm};
    return m;
}

DroneConfigData makeHw1Drone(double radius_cm, double max_adv_cm, double max_elev_cm) {
    return {radius_cm * cm,
            45.0 * horizontal_angle[deg],
            max_adv_cm * cm,
            max_elev_cm * cm};
}

LidarConfigData makeHw1Lidar(double z_min, double z_max, double d, int fovc) {
    return {z_min * cm, z_max * cm, d * cm, static_cast<std::size_t>(fovc)};
}

void runHw1Case(const char* name,
                const SimulationConfigData& sim_cfg,
                const MissionConfigData& mission_cfg,
                const DroneConfigData& drone_cfg,
                const LidarConfigData& lidar_cfg,
                double min_expected_score) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(sim_cfg, std::vector{mission_cfg});
    comp.drones.push_back(drone_cfg);
    comp.lidars.push_back(lidar_cfg);

    const auto report = manager.run(comp, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 1U) << name << ": expected exactly 1 run";
    const auto& result = report.runs.front();
    ASSERT_FALSE(result.mission_results.empty()) << name << ": no mission results";

    EXPECT_NE(result.mission_results.front().status, MissionRunStatus::Error)
        << name << ": run ended with error: "
        << (result.mission_results.front().errors.empty()
                ? "unknown" : result.mission_results.front().errors.front().message);

    EXPECT_GT(result.mission_score, min_expected_score)
        << name << ": score " << result.mission_score
        << " below threshold " << min_expected_score;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Case 1: 20×20×10 office-like map with interior wall segments.
// hw1 result: 86/100.  Threshold: 60 (narrower lidar z_min=10 makes it harder).
// ─────────────────────────────────────────────────────────────────────────────
TEST(Integration, Hw1EdgeCase1_OfficeWithWallSegments) {
    runHw1Case(
        "Case1",
        makeHw1SimConfig("data_maps/hw1_case1.npy", 3.0, 3.0, 4.0),
        makeHw1Mission(1.0, 18.0, 1.0, 18.0, 1.0, 8.0, 8000),
        makeHw1Drone(0.5, 8.0, 8.0),
        makeHw1Lidar(10.0, 200.0, 5.0, 3),
        60.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Case 2: 30×8×8 narrow corridor map.
// hw1 result: 100/100.  Threshold: 70.
// ─────────────────────────────────────────────────────────────────────────────
TEST(Integration, Hw1EdgeCase2_NarrowCorridor) {
    runHw1Case(
        "Case2",
        makeHw1SimConfig("data_maps/hw1_case2.npy", 2.0, 3.0, 3.0),
        makeHw1Mission(1.0, 28.0, 1.0, 6.0, 1.0, 6.0, 5000),
        makeHw1Drone(0.75, 5.0, 5.0),
        makeHw1Lidar(1.0, 500.0, 5.0, 3),
        70.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Case 3: 35×20×16 multi-room map with internal wall gaps.
// hw1 result: 98/100.  Threshold: 70.
// ─────────────────────────────────────────────────────────────────────────────
TEST(Integration, Hw1EdgeCase3_MultiRoomWithGaps) {
    runHw1Case(
        "Case3",
        makeHw1SimConfig("data_maps/hw1_case3.npy", 5.0, 4.0, 2.0),
        makeHw1Mission(1.0, 33.0, 1.0, 18.0, 1.0, 14.0, 8000),
        makeHw1Drone(0.5, 8.0, 8.0),
        makeHw1Lidar(1.0, 200.0, 5.0, 3),
        70.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Case 4: 50×30×12 large open-plan with periodic wall shelves.
// hw1 result: 98/100.  Threshold: 70.
// ─────────────────────────────────────────────────────────────────────────────
TEST(Integration, Hw1EdgeCase4_LargeOpenPlanWithShelves) {
    runHw1Case(
        "Case4",
        makeHw1SimConfig("data_maps/hw1_case4.npy", 5.0, 3.0, 2.0),
        makeHw1Mission(1.0, 48.0, 1.0, 28.0, 1.0, 10.0, 10000),
        makeHw1Drone(1.5, 5.0, 5.0),
        makeHw1Lidar(1.0, 200.0, 5.0, 3),
        70.0);
}
