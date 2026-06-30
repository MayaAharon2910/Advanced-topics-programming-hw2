/*
 * Integration tests for the official staff input files.
 * These tests run the released composition and map files through the normal parser
 * and SimulationManager path to catch path, YAML, and pipeline issues.
 */

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
 * What it does: runs the official staff composition as an end-to-end smoke test.
 * Setup: loads inputs/sim_compose.yaml and executes it through SimulationManager.
 * Checks: the report is well formed, scores stay in range, and the run set finishes under 60 seconds.
 */
TEST(StaffInputs, FullCompositionRunsWithoutCrashing) {
    const auto comp = yaml::parseSimulationComposition("inputs/sim_compose.yaml");

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    const auto staging = stagingDir();
    std::filesystem::create_directories(staging);

    const auto t0 = std::chrono::steady_clock::now();
    types::SimulationManagerReport report;
    ASSERT_NO_THROW(report = manager.run(comp, staging));
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    // 5 sims × (2+1+1+1+1 missions) × 2 drones × 2 lidars = 24 runs
    EXPECT_GT(report.runs.size(), 0U);

    for (const auto& run : report.runs) {
        EXPECT_GE(run.mission_score, -1.0)
            << "Score below valid range - pipeline produced a malformed result";
        EXPECT_LE(run.mission_score, 100.0)
            << "Score above valid range - pipeline produced a malformed result";
        ASSERT_FALSE(run.mission_results.empty())
            << "Every run must report at least one mission result";
    }

    EXPECT_LT(elapsed_ms, 60'000)
        << "Staff composition took " << elapsed_ms << "ms - exceeds the 1-minute limit";

    std::filesystem::remove_all(staging);
}

/*
 * What it does: checks the staff house scenario with semantic voxel values.
 * Setup: isolates the first staff simulation/mission and runs it through the normal loader path.
 * Checks: the produced run has a valid score range instead of failing due to non-binary map values.
 */
TEST(StaffInputs, HouseScenarioMapLoadsWithSemanticValues) {
    const auto comp = yaml::parseSimulationComposition("inputs/sim_compose.yaml");

    // Isolate just the house scenario (first sim/mission group).
    types::SimulationCompositionData isolated{};
    isolated.composition_file = comp.composition_file;
    ASSERT_FALSE(comp.simulation_mission_groups.empty());
    const auto& [sim, missions] = comp.simulation_mission_groups.front();
    ASSERT_FALSE(missions.empty());
    isolated.simulation_mission_groups.emplace_back(sim, std::vector{missions.front()});
    ASSERT_FALSE(comp.drones.empty());
    ASSERT_FALSE(comp.lidars.empty());
    isolated.drones.push_back(comp.drones.front());
    isolated.lidars.push_back(comp.lidars.front());

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
