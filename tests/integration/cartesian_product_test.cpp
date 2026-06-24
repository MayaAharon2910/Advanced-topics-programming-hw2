// Integration tests: SimulationManager Cartesian product correctness.
//
// Tests 1-2: synthetic maps, built in-memory.
// Tests 3-4: hw1 edge cases loaded from real YAML files in hw1_scenarios/.
//            These can also be run manually:
//              ./drone_mapper_simulation hw1_scenarios/composition_case2.yaml
//              ./drone_mapper_simulation hw1_scenarios/composition_case4.yaml

#include <gtest/gtest.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/YamlConfig.h>

namespace {

using namespace drone_mapper;
using namespace drone_mapper::types;

// ── Synthetic helpers ─────────────────────────────────────────────────────────

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

MissionConfigData makeSyntheticMission(double x_max, double y_max, double z_max,
                                       std::size_t steps) {
    MissionConfigData m;
    m.max_steps                        = steps;
    m.gps_resolution                   = 10.0 * cm;
    m.output_mapping_resolution_factor = 1;
    m.boundaries.min_x      = XLength{0.0 * cm};
    m.boundaries.max_x      = XLength{x_max * cm};
    m.boundaries.min_y      = YLength{0.0 * cm};
    m.boundaries.max_y      = YLength{y_max * cm};
    m.boundaries.min_height = ZLength{0.0 * cm};
    m.boundaries.max_height = ZLength{z_max * cm};
    return m;
}

// ── Shared assertion ──────────────────────────────────────────────────────────

void assertProduct(const SimulationCompositionData& comp, std::size_t expected) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));
    const auto report = manager.run(comp, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), expected)
        << "Expected " << expected << " runs from Cartesian product";

    std::size_t errors = 0;
    for (const auto& r : report.runs) { if (r.mission_score < 0.0) ++errors; }
    EXPECT_EQ(errors, 0U) << errors << "/" << expected << " runs ended with error";
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: synthetic, 1 sim × 2 missions × 2 drones × 1 lidar = 4 runs
// ─────────────────────────────────────────────────────────────────────────────
TEST(Integration, CartesianProductProducesFourRuns) {
    SimulationCompositionData comp;

    comp.simulation_mission_groups.emplace_back(
        makeSyntheticSimConfig(),
        std::vector{
            makeSyntheticMission(50, 50, 50, 500),
            makeSyntheticMission(30, 30, 30, 300),
        });
    comp.drones.push_back({3.0 * cm, 45.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm});
    comp.drones.push_back({4.0 * cm, 90.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm});
    comp.lidars.push_back({2.0 * cm, 90.0 * cm, 2.5 * cm, static_cast<std::size_t>(3)});

    // Also verify each run references the correct map
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));
    const auto report = manager.run(comp, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 4U) << "1×2×2×1 = 4 runs";
    for (const auto& r : report.runs)
        EXPECT_EQ(r.simulation_config.map_filename,
                  std::filesystem::path("data_maps/single_voxel_x4_y4_z4.npy"));
    std::size_t errors = 0;
    for (const auto& r : report.runs) { if (r.mission_score < 0.0) ++errors; }
    EXPECT_EQ(errors, 0U);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: synthetic, 2 sims × 1 mission × 1 drone × 2 lidars = 4 runs
// ─────────────────────────────────────────────────────────────────────────────
TEST(Integration, CartesianProductTwoSimsTwoLidars) {
    SimulationCompositionData comp;

    comp.simulation_mission_groups.emplace_back(
        makeSyntheticSimConfig(20, 20, 20),
        std::vector{makeSyntheticMission(50, 50, 50, 300)});
    comp.simulation_mission_groups.emplace_back(
        makeSyntheticSimConfig(10, 10, 10),
        std::vector{makeSyntheticMission(50, 50, 50, 300)});
    comp.drones.push_back({3.0 * cm, 45.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm});
    comp.lidars.push_back({2.0 * cm, 50.0 * cm, 2.5 * cm, static_cast<std::size_t>(2)});
    comp.lidars.push_back({2.0 * cm, 90.0 * cm, 2.5 * cm, static_cast<std::size_t>(3)});

    assertProduct(comp, 4);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: hw1 Case 2 — Narrow Corridor (30×8×8 cm) loaded from YAML files.
// 1 sim × 2 missions × 1 drone × 2 lidars = 4 runs.
//
// The same scenario can be run manually from the project root:
//   ./drone_mapper_simulation hw1_scenarios/composition_case2.yaml
// ─────────────────────────────────────────────────────────────────────────────
TEST(Integration, CartesianProductHw1Case2NarrowCorridor) {
    const auto comp = drone_mapper::yaml::parseSimulationComposition(
        "hw1_scenarios/composition_case2.yaml");

    assertProduct(comp, 4);  // 1×2×1×2 = 4
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: hw1 Case 4 — Large Open-Plan (50×30×12 cm) loaded from YAML files.
// 1 sim × 1 mission × 2 drones × 2 lidars = 4 runs.
//
// The same scenario can be run manually from the project root:
//   ./drone_mapper_simulation hw1_scenarios/composition_case4.yaml
// ─────────────────────────────────────────────────────────────────────────────
TEST(Integration, CartesianProductHw1Case4LargeOpenPlan) {
    const auto comp = drone_mapper::yaml::parseSimulationComposition(
        "hw1_scenarios/composition_case4.yaml");

    assertProduct(comp, 4);  // 1×1×2×2 = 4
}
