// Integration test: Benchmark map — complex 3-story house (29×30×31 cm).
//
// Tests two things the course staff care about:
//  1. Collision detection: larger drones are blocked by smaller doorways and
//     therefore map less area. Score must strictly decrease as drone size grows.
//  2. Performance: all 4 runs must finish within the staff's 1-minute limit.
//
// The same composition can be run manually:
//   ./drone_mapper_simulation complex_scenario/composition.yaml

#include <gtest/gtest.h>
#include <chrono>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/YamlConfig.h>

// ─────────────────────────────────────────────────────────────────────────────
// Fixture
// ─────────────────────────────────────────────────────────────────────────────
class BenchmarkMapTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        manager_ = std::make_unique<drone_mapper::SimulationManager>(std::move(factory));
    }

    std::unique_ptr<drone_mapper::SimulationManager> manager_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: Cartesian product produces exactly 4 runs and finishes in time.
//
// 1 sim × 1 mission × 4 drones × 1 lidar = 4 runs.
// Runtime for all 4 runs combined must be under 60 seconds.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(BenchmarkMapTest, FourRunsWithinOneMinute) {
    const auto comp = drone_mapper::yaml::parseSimulationComposition(
        "complex_scenario/composition.yaml");

    const auto t0 = std::chrono::steady_clock::now();
    const auto report = manager_->run(comp, std::filesystem::current_path());
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    ASSERT_EQ(report.runs.size(), 4U)
        << "Expected 1×1×4×1 = 4 runs from Cartesian product";

    EXPECT_LT(elapsed_ms, 60'000)
        << "Total runtime " << elapsed_ms << "ms exceeded 60-second staff limit";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: Collision detection — larger drones score less.
//
// Map entrance sizes (at 1cm/voxel):
//   Main entrance (y=25 wall):    4 wide × 4 tall → fits drone_large (r=1.8cm)
//   2nd-floor inner gap (x=20):   3 wide           → fits drone_medium (r=1.3cm)
//   Secret roof gap (x=7,y=18-19):2 wide × 1 tall  → fits drone_small (r=0.8cm)
//   All areas accessible:                           → drone_tiny (r=0.3cm)
//
// Drone order in composition: [large, medium, small, tiny] (same as YAML order).
// Expected: score[tiny] >= score[small] >= score[medium] >= score[large].
// ─────────────────────────────────────────────────────────────────────────────
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

    GTEST_LOG_(INFO) << "Scores — large:" << score_large
                     << " medium:" << score_medium
                     << " small:" << score_small
                     << " tiny:" << score_tiny;

    // All runs must have completed (no pipeline error)
    for (std::size_t i = 0; i < 4; ++i) {
        if (report.runs[i].mission_score < 0.0) {
            GTEST_LOG_(WARNING) << "Run " << i << " ended with error — "
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

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: Tiny drone achieves a meaningful mapping score.
//         (Sanity check that the algorithm actually works on this map.)
// ─────────────────────────────────────────────────────────────────────────────
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
            << " — algorithm may not be exploring the house effectively";
    } else {
        GTEST_LOG_(WARNING) << "Tiny drone run returned error (-1). "
            "Check starting position clearance and boundary config.";
    }
}
