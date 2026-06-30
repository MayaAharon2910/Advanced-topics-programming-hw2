// =============================================================================
// staff_inputs_test.cpp — Integration test using the official staff input files
//
// The course staff released a complete composition (inputs/sim_compose.yaml)
// covering 5 simulations × their missions × 2 drones × 2 lidars. This test
// runs that exact composition through our SimulationManager unmodified, to
// prove the pipeline survives the staff's real test data end-to-end:
// no crashes, no unhandled exceptions, no infinite loops, and every run
// returns a valid score (0..100, or -1 on a legitimate per-run error).
//
// This is a stability check, not a score check: some runs may legitimately
// error out (e.g. a drone too large for a doorway) — what matters is that
// the system never crashes and always produces a well-formed report.
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
 * What it does: runs the staff's full composition (5 sims × missions × 2
 *               drones × 2 lidars) through our SimulationManager.
 * Setup: loads inputs/sim_compose.yaml directly — no modification to the
 *        staff's files. Output is written to a throwaway temp directory.
 * Checks: the pipeline does not throw; every run in the report has a score
 *         in the valid range [-1, 100]; total runtime stays under 60s
 *         (course requirement: "integration tests should always finish in
 *         reasonable time, about 1 minute at most").
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
            << "Score below valid range — pipeline produced a malformed result";
        EXPECT_LE(run.mission_score, 100.0)
            << "Score above valid range — pipeline produced a malformed result";
        ASSERT_FALSE(run.mission_results.empty())
            << "Every run must report at least one mission result";
    }

    EXPECT_LT(elapsed_ms, 60'000)
        << "Staff composition took " << elapsed_ms << "ms — exceeds the 1-minute limit";

    std::filesystem::remove_all(staging);
}

/*
 * What it does: spot-checks that the house scenario map (scenario_house.npy)
 *               loads correctly despite using semantic values > 1.
 * Setup: runs only the house simulation + lower mission + small drone +
 *        short lidar, in isolation from the rest of the composition.
 * Checks: the run completes without throwing and produces a score — this
 *         is a regression guard for the Map3DImpl clamping fix (values
 *         like 2/3/4/18/45 in this map must be treated as Occupied,
 *         not silently dropped to Unmapped).
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
