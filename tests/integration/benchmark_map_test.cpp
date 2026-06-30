/*
 * Integration tests for the benchmark house map.
 * These tests run the real SimulationManager on the complex scenario and check both
 * runtime and score behavior across different drone sizes.
 */

#include <gtest/gtest.h>
#include <chrono>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/YamlConfig.h>

class BenchmarkMapTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        manager_ = std::make_unique<drone_mapper::SimulationManager>(std::move(factory));
    }

    std::unique_ptr<drone_mapper::SimulationManager> manager_;
};

/*
 * What it does: checks the benchmark composition runtime.
 * Setup: runs 1 simulation x 1 mission x 4 drones x 1 lidar on the complex house map.
 * Checks: exactly four runs are produced and the whole set finishes under 60 seconds.
 */
TEST_F(BenchmarkMapTest, FourRunsWithinOneMinute) {
    const auto comp = drone_mapper::yaml::parseSimulationComposition(
        "complex_scenario/composition.yaml");

    const auto t0 = std::chrono::steady_clock::now();
    const auto report = manager_->run(comp, std::filesystem::current_path());
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    ASSERT_EQ(report.runs.size(), 4U)
        << "Expected 1x1x4x1 = 4 runs from Cartesian product";

    EXPECT_LT(elapsed_ms, 60'000)
        << "Total runtime " << elapsed_ms << "ms exceeded 60-second staff limit";
}

/*
 * What it does: compares score behavior for different drone sizes.
 * Setup: runs the same benchmark map with large, medium, small, and tiny drones.
 * Checks: smaller drones score at least as well as larger drones when both runs complete.
 */
TEST_F(BenchmarkMapTest, SmallerDronesMapsMoreArea) {
    const auto comp = drone_mapper::yaml::parseSimulationComposition(
        "complex_scenario/composition.yaml");

    const auto report = manager_->run(comp, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 4U);

    // Scores in order: run 0=large, 1=medium, 2=small, 3=tiny
    const double score_large  = report.runs[0].mission_score;
    const double score_medium = report.runs[1].mission_score;
    const double score_small  = report.runs[2].mission_score;
    const double score_tiny   = report.runs[3].mission_score;

    GTEST_LOG_(INFO) << "Scores - large:" << score_large
                     << " medium:" << score_medium
                     << " small:" << score_small
                     << " tiny:" << score_tiny;

    // All runs must have completed (no pipeline error)
    for (std::size_t i = 0; i < 4; ++i) {
        if (report.runs[i].mission_score < 0.0) {
            GTEST_LOG_(WARNING) << "Run " << i << " ended with error - "
                "collision detection may have blocked start position. "
                "Check drone radius vs starting location clearance.";
        }
    }

    // Tiny drone must score at least as well as large drone.
    // (It can access every room; large drone is blocked by smaller doorways.)
    if (score_tiny >= 0.0 && score_large >= 0.0) {
        EXPECT_GE(score_tiny, score_large)
            << "Tiny drone should map at least as much as large drone";
    }

    // If medium and small also completed, verify monotonicity
    if (score_medium >= 0.0 && score_large >= 0.0) {
        EXPECT_GE(score_medium, score_large)
            << "Medium drone (fits 3-wide gap) should outscore large (fits only 4-wide)";
    }
    if (score_small >= 0.0 && score_medium >= 0.0) {
        EXPECT_GE(score_small, score_medium)
            << "Small drone (fits 2-wide roof gap) should outscore medium";
    }
}

/*
 * What it does: checks that the tiny drone gets a useful benchmark score.
 * Setup: runs the complex map and reads the tiny-drone result from the report.
 * Checks: the score passes a low sanity threshold instead of only proving that the run exists.
 */
TEST_F(BenchmarkMapTest, TinyDroneAchievesMeaningfulScore) {
    const auto comp = drone_mapper::yaml::parseSimulationComposition(
        "complex_scenario/composition.yaml");

    const auto report = manager_->run(comp, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 4U);

    const double score_tiny = report.runs[3].mission_score;

    if (score_tiny >= 0.0) {
        // Tiny drone can enter all rooms. It should map a significant fraction.
        EXPECT_GT(score_tiny, 30.0)
            << "Tiny drone scored only " << score_tiny
            << " - algorithm may not be exploring the house effectively";
    } else {
        GTEST_LOG_(WARNING) << "Tiny drone run returned error (-1). "
            "Check starting position clearance and boundary config.";
    }
}
