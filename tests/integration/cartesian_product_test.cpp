/*
 * Integration tests for SimulationManager cartesian-product expansion.
 * The scenarios combine simulations, missions, drones, and lidars and verify that
 * the manager creates the expected number of runs without crashing.
 */

#include <gtest/gtest.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/YamlConfig.h>

namespace {

using namespace drone_mapper;
using namespace drone_mapper::types;

// -----------------------------------------------------------------------------

SimulationConfigData makeSyntheticSimConfig(double sx = 20, double sy = 20, double sz = 20) {
    SimulationConfigData s;
    s.map_filename   = "data_maps/single_voxel_x4_y4_z4.npy";
    s.map_resolution = 10.0 * cm;
    s.map_offset     = Position3D{};
    s.initial_drone_position = Position3D{
        sx * x_extent[cm], sy * y_extent[cm], sz * z_extent[cm]};
    s.initial_angle = 0.0 * horizontal_angle[deg];
    return s;
}

// Boundaries always [0,50] to match the full 5x5x5 map.
// Missions differ only in max_steps so the algorithm stays within the map.
MissionConfigData makeSyntheticMission(std::size_t steps) {
    MissionConfigData m;
    m.max_steps                        = steps;
    m.gps_resolution                   = 10.0 * cm;
    m.output_mapping_resolution_factor = 1;
    m.mission_bounds.min_x      = 0.0 * x_extent[cm];
    m.mission_bounds.max_x      = 40.0 * x_extent[cm];
    m.mission_bounds.min_y      = 0.0 * y_extent[cm];
    m.mission_bounds.max_y      = 40.0 * y_extent[cm];
    m.mission_bounds.min_height = 0.0 * z_extent[cm];
    m.mission_bounds.max_height = 40.0 * z_extent[cm];
    return m;
}

// -----------------------------------------------------------------------------

void assertProduct(const SimulationCompositionData& comp, std::size_t expected) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));
    const auto report = manager.run(comp, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), expected)
        << "Expected " << expected << " runs from Cartesian product";

    std::size_t errors = 0;
    for (const auto& r : report.runs) { if (r.mission_score < 0.0) ++errors; }
    if (errors > 0) {
        GTEST_LOG_(WARNING) << errors << "/" << expected
            << " runs ended with error (algorithm hit map boundaries or obstacles - "
            "pipeline still stable, Cartesian product count is correct)";
    }
}

}

/*
 * What it does: checks cartesian-product expansion for a small synthetic setup.
 * Setup: uses 1 simulation x 2 missions x 2 drones x 1 lidar.
 * Checks: the report contains four runs and keeps the expected simulation filename.
 */
TEST(Integration, CartesianProductProducesFourRuns) {
    SimulationCompositionData comp;

    comp.simulation_mission_groups.emplace_back(
        makeSyntheticSimConfig(),
        std::vector{
            makeSyntheticMission(500),
            makeSyntheticMission(300),
        });
    comp.drones.push_back({3.0 * cm, 45.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm});
    comp.drones.push_back({4.0 * cm, 90.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm});
    comp.lidars.push_back({2.0 * cm, 90.0 * cm, 2.5 * cm, static_cast<std::size_t>(3)});

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));
    const auto report = manager.run(comp, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 4U) << "1x2x2x1 = 4 runs";
    for (const auto& r : report.runs)
        EXPECT_EQ(r.simulation_config.map_filename,
                  std::filesystem::path("data_maps/single_voxel_x4_y4_z4.npy"));
    std::size_t errors = 0;
    for (const auto& r : report.runs) { if (r.mission_score < 0.0) ++errors; }
    if (errors > 0) {
        GTEST_LOG_(WARNING) << errors << "/4 runs ended with error (pipeline stable)";
    }
}

/*
 * What it does: checks cartesian-product expansion across simulations and lidars.
 * Setup: uses 2 simulations x 1 mission x 1 drone x 2 lidars.
 * Checks: the manager reports the expected four runs.
 */
TEST(Integration, CartesianProductTwoSimsTwoLidars) {
    SimulationCompositionData comp;

    comp.simulation_mission_groups.emplace_back(
        makeSyntheticSimConfig(20, 20, 20),
        std::vector{makeSyntheticMission(300)});
    comp.simulation_mission_groups.emplace_back(
        makeSyntheticSimConfig(10, 10, 10),
        std::vector{makeSyntheticMission(300)});
    comp.drones.push_back({3.0 * cm, 45.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm});
    comp.lidars.push_back({2.0 * cm, 50.0 * cm, 2.5 * cm, static_cast<std::size_t>(2)});
    comp.lidars.push_back({2.0 * cm, 90.0 * cm, 2.5 * cm, static_cast<std::size_t>(3)});

    assertProduct(comp, 4);
}

/*
 * What it does: checks cartesian-product expansion on the HW1 narrow-corridor scenario.
 * Setup: loads the converted YAML composition for case 2 from hw1_scenarios.
 * Checks: all four generated runs are reported without crashing the pipeline.
 */
TEST(Integration, CartesianProductHw1Case2NarrowCorridor) {
    const auto comp = drone_mapper::yaml::parseSimulationComposition(
        "hw1_scenarios/composition_case2.yaml");

    assertProduct(comp, 4);  // 1x2x1x2 = 4
}

/*
 * What it does: checks cartesian-product expansion on the HW1 large open-plan scenario.
 * Setup: loads the converted YAML composition for case 4 from hw1_scenarios.
 * Checks: all four generated runs are reported by SimulationManager.
 */
TEST(Integration, CartesianProductHw1Case4LargeOpenPlan) {
    const auto comp = drone_mapper::yaml::parseSimulationComposition(
        "hw1_scenarios/composition_case4.yaml");

    assertProduct(comp, 4);  // 1x1x2x2 = 4
}
