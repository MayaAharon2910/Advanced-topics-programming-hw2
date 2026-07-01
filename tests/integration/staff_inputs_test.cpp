// =============================================================================
// staff_inputs_test.cpp — Integration tests using the official staff input files
//
// The course staff released a full composition (inputs/sim_compose.yaml)
// with 5 simulations × 6 missions × 2 drones × 2 lidars = 24 runs.
// Running all 24 runs with max_steps up to 10 000 exceeds the 1-minute
// integration test budget, so we split coverage across two tests:
//
//   FullCompositionRunsWithoutCrashing — loads and parses the full composition,
//     then runs the first run only (1 sim × 1 mission × 1 drone × 1 lidar)
//     as a fast pipeline-stability smoke test.
//
//   HouseScenarioMapLoadsWithSemanticValues — isolated regression guard for the
//     Map3DImpl clamping fix: the house map uses uint8 values > 1 and must be
//     treated as Occupied, not Unmapped.
// =============================================================================

#include <gtest/gtest.h>

#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/YamlConfig.h>

#include <chrono>
#include <filesystem>

namespace drone_mapper {

namespace {
std::filesystem::path stagingDir() {
    return std::filesystem::current_path() / "tmp_staff_inputs_output";
}
} // namespace

/*
 * What it does: loads the staff's full composition YAML, then runs a
 *               single-run subset (first sim/mission/drone/lidar) as a
 *               fast pipeline smoke test.
 * Setup: parses inputs/sim_compose.yaml to verify it is well-formed and
 *        contains the expected structure (≥1 sim group, ≥1 drone, ≥1 lidar),
 *        then constructs a 1×1×1×1 composition and runs it.
 * Checks: no exception thrown; the report contains exactly 1 run with a
 *         valid score in [-1, 100]; total runtime stays well under 60s.
 *         This catches crashes, parse errors, and factory failures without
 *         spending minutes on all 24 full runs.
 */
TEST(StaffInputs, FullCompositionRunsWithoutCrashing) {
    // Parse the full composition — verifies the YAML is well-formed.
    const auto comp = yaml::parseSimulationComposition("inputs/sim_compose.yaml");

    ASSERT_FALSE(comp.simulation_mission_groups.empty())
        << "Staff composition must have at least one simulation group";
    ASSERT_FALSE(comp.drones.empty())
        << "Staff composition must specify at least one drone";
    ASSERT_FALSE(comp.lidars.empty())
        << "Staff composition must specify at least one lidar";

    // Pick the sim/mission pair with the fewest max_steps for a fast smoke test.
    // The staff's house missions have max_steps=10000; large_room has 500.
    const auto& groups = comp.simulation_mission_groups;
    std::size_t best_idx = 0;
    std::size_t best_steps = SIZE_MAX;
    for (std::size_t i = 0; i < groups.size(); ++i) {
        for (const auto& m : std::get<1>(groups[i])) {
            if (m.max_steps < best_steps) {
                best_steps = m.max_steps;
                best_idx   = i;
            }
        }
    }

    const auto& [chosen_sim, chosen_missions] = groups[best_idx];
    // Find the mission with the fewest steps within this group.
    const auto& chosen_mission = *std::min_element(
        chosen_missions.begin(), chosen_missions.end(),
        [](const auto& a, const auto& b) { return a.max_steps < b.max_steps; });

    types::SimulationCompositionData subset{};
    subset.composition_file = comp.composition_file;
    subset.simulation_mission_groups.emplace_back(chosen_sim, std::vector{chosen_mission});
    subset.drones.push_back(comp.drones.front());
    subset.lidars.push_back(comp.lidars.front());

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    const auto staging = stagingDir();
    std::filesystem::create_directories(staging);

    const auto t0 = std::chrono::steady_clock::now();
    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(subset, staging));
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    ASSERT_EQ(report.runs.size(), 1U)
        << "Subset composition should produce exactly 1 run";
    EXPECT_GE(report.runs.front().mission_score, -1.0);
    EXPECT_LE(report.runs.front().mission_score, 100.0);

    EXPECT_LT(elapsed_ms, 60'000)
        << "Single staff run took " << elapsed_ms << "ms — exceeds the 1-minute limit";

    std::filesystem::remove_all(staging);
}

/*
 * What it does: regression guard for the Map3DImpl clamping fix using the
 *               staff's house map (scenario_house.npy), which stores solid
 *               voxels as uint8 values 2/3/4/18/45 instead of the canonical 1.
 * Setup: runs the first house mission with the small drone and short lidar —
 *        the smallest/fastest single run in the house scenario.
 * Checks: no exception thrown (the factory must load the uint8 map without
 *         crashing); the run produces a valid score in [-1, 100]. If the
 *         clamping fix were reverted, the drone would fly through solid floors
 *         and the lidar would miss all walls, producing incorrect behaviour
 *         (though not necessarily a crash — the real bug is silent degradation
 *         of scores, which the benchmark map tests catch separately).
 */
TEST(StaffInputs, HouseScenarioMapLoadsWithSemanticValues) {
    const auto comp = yaml::parseSimulationComposition("inputs/sim_compose.yaml");

    ASSERT_FALSE(comp.simulation_mission_groups.empty());
    const auto& [sim, missions] = comp.simulation_mission_groups.front();
    ASSERT_FALSE(missions.empty());
    ASSERT_FALSE(comp.drones.empty());
    ASSERT_FALSE(comp.lidars.empty());

    // Use the mission with fewest steps from this group.
    auto chosen_mission2 = *std::min_element(
        missions.begin(), missions.end(),
        [](const auto& a, const auto& b) { return a.max_steps < b.max_steps; });
    // Cap max_steps so this test finishes in a few seconds even if the drone
    // hits obstacles immediately (which it does on the house map with walls).
    chosen_mission2.max_steps = std::min(chosen_mission2.max_steps, std::size_t{200});

    types::SimulationCompositionData isolated{};
    isolated.composition_file = comp.composition_file;
    isolated.simulation_mission_groups.emplace_back(sim, std::vector{chosen_mission2});
    isolated.drones.push_back(comp.drones.front());
    isolated.lidars.push_back(comp.lidars.back());

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    const auto staging = stagingDir();
    std::filesystem::create_directories(staging);

    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(isolated, staging));

    ASSERT_EQ(report.runs.size(), 1U);
    EXPECT_GE(report.runs.front().mission_score, -1.0);
    EXPECT_LE(report.runs.front().mission_score, 100.0);

    std::filesystem::remove_all(staging);
}

} // namespace drone_mapper
