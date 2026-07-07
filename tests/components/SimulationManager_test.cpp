// =============================================================================
// SimulationManager_test.cpp - Component tests for SimulationManager
// SimulationManager receives a SimulationCompositionData, expands the Cartesian
// product of (simulation x mission) x drone x lidar, and calls the factory
// once per combination. These tests replace the factory with a GMock so no
// real maps, sensors, or algorithms are exercised - only the manager's
// product-expansion logic is tested.
// =============================================================================

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationManager.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>

namespace drone_mapper {
namespace {

class MockSimulationRun : public ISimulationRun {
public:
    MOCK_METHOD(types::SimulationResult, run, (), (override));
};

class MockSimulationRunFactory : public ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<ISimulationRun>,
                create,
                (const types::SimulationConfigData& simulation,
                 const types::MissionConfigData& mission,
                 const types::DroneConfigData& drone,
                 const types::LidarConfigData& lidar,
                 const std::filesystem::path& output_path),
                (override));
};

// Minimal 1-sim x 1-mission x 1-drone x 1-lidar composition.
[[nodiscard]] types::SimulationCompositionData singleRunComposition() {
    types::SimulationCompositionData composition{};
    composition.composition_file = "simulation.yaml";
    composition.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{
            "map.npy", 10.0 * cm, Position3D{}, Position3D{},
            0.0 * horizontal_angle[deg]},
        std::vector{types::MissionConfigData{.max_steps=5, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0}});
    composition.drones.push_back(types::DroneConfigData{
        30.0 * cm, 45.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm});
    composition.lidars.push_back(types::LidarConfigData{
        20.0 * cm, 120.0 * cm, 2.5 * cm, 5});
    return composition;
}

} // namespace

/*
 * What it does: verifies the manager calls the factory exactly once for a
 *               single-combination composition and returns the result correctly.
 * Setup: 1 sim × 1 mission × 1 drone × 1 lidar → 1 expected run.
 *        MockRunFactory returns a fake run with score=100 and Completed status.
 * Checks: factory.create() is called Times(1); the report contains 1 run
 *         with Completed status and score 100.0.
 */
TEST(SimulationManager, RunsSingleCartesianCombinationWithFactory) {
    auto factory = std::make_unique<testing::StrictMock<MockSimulationRunFactory>>();
    auto* factory_raw = factory.get();

    types::SimulationResult expected_result{};
    expected_result.mission_config = types::MissionConfigData{.max_steps=5, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0};
    expected_result.resolution_request_status = types::ResolutionRequestStatus::Accepted;
    expected_result.mission_score = 100.0;
    expected_result.mission_results.push_back(types::MissionRunResult{
        types::MissionRunStatus::Completed, 1, {}});

    EXPECT_CALL(*factory_raw, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .Times(1)
        .WillOnce([&expected_result](const types::SimulationConfigData&,
                                      const types::MissionConfigData&,
                                      const types::DroneConfigData&,
                                      const types::LidarConfigData&,
                                      const std::filesystem::path&) {
            auto run = std::make_unique<testing::StrictMock<MockSimulationRun>>();
            EXPECT_CALL(*run, run()).WillOnce(testing::Return(expected_result));
            return run;
        });

    SimulationManager manager(std::move(factory));
    const auto report = manager.run(singleRunComposition(), std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 1U);
    ASSERT_FALSE(report.runs.front().mission_results.empty());
    EXPECT_EQ(report.runs.front().mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_DOUBLE_EQ(report.runs.front().mission_score, 100.0);
}

/*
 * What it does: verifies that a factory exception does not crash the manager
 *               and is converted to a score of -1 with Error status.
 * Setup: factory.create() throws std::runtime_error for the single run.
 * Checks: the manager catches the exception; the report contains 1 run
 *         with score=-1.0 and Error status — other runs (if any) continue.
 */
TEST(SimulationManager, ConvertsFactoryExceptionToErrorRun) {
    auto factory = std::make_unique<testing::StrictMock<MockSimulationRunFactory>>();
    auto* factory_raw = factory.get();

    EXPECT_CALL(*factory_raw, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .Times(1)
        .WillOnce(testing::Throw(std::runtime_error("factory failure")));

    SimulationManager manager(std::move(factory));
    const auto report = manager.run(singleRunComposition(), std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 1U);
    EXPECT_DOUBLE_EQ(report.runs.front().mission_score, -1.0);
    ASSERT_FALSE(report.runs.front().mission_results.empty());
    EXPECT_EQ(report.runs.front().mission_results.front().status, types::MissionRunStatus::Error);
}

/*
 * What it does: verifies the Cartesian product is computed correctly for a
 *               multi-mission, multi-drone composition.
 * Setup: 1 sim × 2 missions × 2 drones × 1 lidar = 4 expected runs.
 *        All dependencies are replaced by MockRunFactory (strict GMock isolation).
 * Checks: factory.create() is called exactly Times(4); the report contains
 *         4 runs. Catches bugs where the manager skips a config axis or
 *         misorders the iteration.
 */
TEST(SimulationManager, GeneratesMultipleRunsForCartesianProduct) {
    types::SimulationCompositionData composition{};
    composition.composition_file = "simulation.yaml";
    composition.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{
            "map.npy", 10.0 * cm, Position3D{}, Position3D{},
            0.0 * horizontal_angle[deg]},
        std::vector{
            types::MissionConfigData{.max_steps=5, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0},
            types::MissionConfigData{.max_steps=8, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0}});
    composition.drones.push_back({30.0 * cm, 45.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm});
    composition.drones.push_back({20.0 * cm, 45.0 * horizontal_angle[deg], 30.0 * cm, 25.0 * cm});
    composition.lidars.push_back({20.0 * cm, 120.0 * cm, 2.5 * cm, 5});

    auto factory = std::make_unique<testing::StrictMock<MockSimulationRunFactory>>();
    auto* factory_raw = factory.get();

    types::SimulationResult dummy{};
    dummy.resolution_request_status = types::ResolutionRequestStatus::Accepted;
    dummy.mission_score = 50.0;
    dummy.mission_results.push_back({types::MissionRunStatus::Completed, 1, {}});

    EXPECT_CALL(*factory_raw,
                create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .Times(4)
        .WillRepeatedly([&dummy](const types::SimulationConfigData&,
                                  const types::MissionConfigData&,
                                  const types::DroneConfigData&,
                                  const types::LidarConfigData&,
                                  const std::filesystem::path&) {
            auto run = std::make_unique<testing::NiceMock<MockSimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(testing::Return(dummy));
            return run;
        });

    SimulationManager manager(std::move(factory));
    const auto report = manager.run(composition, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 4U)
        << "SimulationManager must produce 1x2x2x1 = 4 runs from the Cartesian product";
}

/*
 * What it does: verifies the manager populates report metadata correctly.
 * Setup: single run that succeeds with score 50.
 * Checks: metric == "output_map_accuracy", score_range is [0,100], error_score
 *         is -1, and generated_at_utc is non-empty.
 */
TEST(SimulationManager, ReportMetadataPopulated) {
    auto factory = std::make_unique<testing::NiceMock<MockSimulationRunFactory>>();

    types::SimulationResult res{};
    res.mission_score = 50.0;
    res.mission_results.push_back(
        types::MissionRunResult{types::MissionRunStatus::Completed, 1, {}});

    ON_CALL(*factory, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillByDefault([&res](const types::SimulationConfigData&,
                              const types::MissionConfigData&,
                              const types::DroneConfigData&,
                              const types::LidarConfigData&,
                              const std::filesystem::path&) {
            auto run = std::make_unique<testing::NiceMock<MockSimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(testing::Return(res));
            return run;
        });

    SimulationManager manager(std::move(factory));
    const auto tmp = std::filesystem::temp_directory_path() / "dm_sm_meta";
    std::filesystem::remove_all(tmp);
    const auto report = manager.run(singleRunComposition(), tmp);
    std::filesystem::remove_all(tmp);

    EXPECT_EQ(report.metric, "output_map_accuracy");
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
    EXPECT_EQ(report.error_score, -1);
    EXPECT_FALSE(report.generated_at_utc.empty());
}

/*
 * What it does: verifies the manager writes simulation_output.yaml to disk.
 * Setup: single successful run with score 42.
 * Checks: simulation_output.yaml exists; it contains "score_report", "runs:",
 *         and the score value "42" — matching the required report structure.
 */
TEST(SimulationManager, WritesSimulationOutputYaml) {
    auto factory = std::make_unique<testing::NiceMock<MockSimulationRunFactory>>();

    types::SimulationResult res{};
    res.mission_score = 42.0;
    res.mission_results.push_back(
        types::MissionRunResult{types::MissionRunStatus::Completed, 3, {}});

    ON_CALL(*factory, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillByDefault([&res](const types::SimulationConfigData&,
                              const types::MissionConfigData&,
                              const types::DroneConfigData&,
                              const types::LidarConfigData&,
                              const std::filesystem::path&) {
            auto run = std::make_unique<testing::NiceMock<MockSimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(testing::Return(res));
            return run;
        });

    SimulationManager manager(std::move(factory));
    const auto tmp = std::filesystem::temp_directory_path() / "dm_sm_yaml";
    std::filesystem::remove_all(tmp);
    std::ignore = manager.run(singleRunComposition(), tmp);

    const auto yaml_path = tmp / "simulation_output.yaml";
    ASSERT_TRUE(std::filesystem::exists(yaml_path)) << "simulation_output.yaml not created";

    std::ifstream f(yaml_path);
    const std::string content{std::istreambuf_iterator<char>(f),
                              std::istreambuf_iterator<char>()};
    EXPECT_NE(content.find("score_report"), std::string::npos);
    EXPECT_NE(content.find("42"), std::string::npos);

    std::filesystem::remove_all(tmp);
}

/*
 * What it does: verifies summary fields when runs have mixed scores.
 * Setup: 2 missions with scores 30 and 60. average should be 45.
 * Checks: the YAML file contains "scored_runs: 2" and "average_score: 45".
 */
TEST(SimulationManager, SummaryAverageReflectsScores) {
    types::SimulationCompositionData comp{};
    comp.composition_file = "simulation.yaml";
    double scores[] = {30.0, 60.0};
    int idx = 0;
    comp.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{
            "map.npy", 10.0 * cm, Position3D{}, Position3D{},
            0.0 * horizontal_angle[deg]},
        std::vector{
            types::MissionConfigData{.max_steps=5, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0},
            types::MissionConfigData{.max_steps=5, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0}});
    comp.drones.push_back({30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm});
    comp.lidars.push_back({20.0*cm, 120.0*cm, 2.5*cm, 5});

    auto factory = std::make_unique<testing::NiceMock<MockSimulationRunFactory>>();
    ON_CALL(*factory, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillByDefault([&scores, &idx](const types::SimulationConfigData&,
                                       const types::MissionConfigData&,
                                       const types::DroneConfigData&,
                                       const types::LidarConfigData&,
                                       const std::filesystem::path&) {
            types::SimulationResult r{};
            r.mission_score = scores[idx++ % 2];
            r.mission_results.push_back(
                {types::MissionRunStatus::Completed, 1, {}});
            auto run = std::make_unique<testing::NiceMock<MockSimulationRun>>();
            ON_CALL(*run, run()).WillByDefault(testing::Return(r));
            return run;
        });

    SimulationManager manager(std::move(factory));
    const auto tmp = std::filesystem::temp_directory_path() / "dm_sm_avg";
    std::filesystem::remove_all(tmp);
    std::ignore = manager.run(comp, tmp);

    std::ifstream f(tmp / "simulation_output.yaml");
    const std::string content{std::istreambuf_iterator<char>(f),
                              std::istreambuf_iterator<char>()};
    EXPECT_NE(content.find("scored_runs: 2"), std::string::npos)
        << "Expected 'scored_runs: 2' in:\n" << content;
    EXPECT_NE(content.find("average_score: 45"), std::string::npos)
        << "Expected 'average_score: 45' in:\n" << content;

    std::filesystem::remove_all(tmp);
}

/*
 * What it does: verifies summary fields when all runs are errors.
 * Setup: factory always throws, so both runs get score -1.
 * Checks: YAML contains "error_runs: 2" and "scored_runs: 0".
 */
TEST(SimulationManager, AllErrorRunsReportedInSummary) {
    types::SimulationCompositionData comp{};
    comp.composition_file = "simulation.yaml";
    comp.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{
            "map.npy", 10.0 * cm, Position3D{}, Position3D{},
            0.0 * horizontal_angle[deg]},
        std::vector{
            types::MissionConfigData{.max_steps=5, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0},
            types::MissionConfigData{.max_steps=5, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0}});
    comp.drones.push_back({30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm});
    comp.lidars.push_back({20.0*cm, 120.0*cm, 2.5*cm, 5});

    auto factory = std::make_unique<testing::NiceMock<MockSimulationRunFactory>>();
    EXPECT_CALL(*factory, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillRepeatedly(testing::Throw(std::runtime_error("nope")));

    SimulationManager manager(std::move(factory));
    const auto tmp = std::filesystem::temp_directory_path() / "dm_sm_allerr";
    std::filesystem::remove_all(tmp);
    std::ignore = manager.run(comp, tmp);

    std::ifstream f(tmp / "simulation_output.yaml");
    const std::string content{std::istreambuf_iterator<char>(f),
                              std::istreambuf_iterator<char>()};
    EXPECT_NE(content.find("error_runs: 2"), std::string::npos)
        << "Expected 'error_runs: 2' in:\n" << content;
    EXPECT_NE(content.find("scored_runs: 0"), std::string::npos)
        << "Expected 'scored_runs: 0' in:\n" << content;

    std::filesystem::remove_all(tmp);
}

/*
 * What it does: verifies a single scenario error does not stop the batch.
 * Setup: 1 sim x 2 missions x 1 drone x 1 lidar = 2 runs; the factory throws
 *        for the FIRST call and succeeds for the SECOND.
 * Checks: both runs appear in the report (run 0 = Error/-1, run 1 =
 *         Completed/scored) - proving the manager continues to the next
 *         scenario after an error, per the assignment's explicit
 *         requirement that one failed scenario must not stop the batch.
 */
TEST(SimulationManager, ContinuesToNextScenarioAfterOneErrors) {
    types::SimulationCompositionData comp{};
    comp.composition_file = "simulation.yaml";
    comp.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{
            "map.npy", 10.0 * cm, Position3D{}, Position3D{},
            0.0 * horizontal_angle[deg]},
        std::vector{
            types::MissionConfigData{.max_steps=5, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0},
            types::MissionConfigData{.max_steps=8, .gps_resolution=10.0*cm, .output_mapping_resolution_factor=1.0}});
    comp.drones.push_back({30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm});
    comp.lidars.push_back({20.0*cm, 120.0*cm, 2.5*cm, 5});

    auto factory = std::make_unique<testing::StrictMock<MockSimulationRunFactory>>();
    auto* factory_raw = factory.get();

    types::SimulationResult success{};
    success.resolution_request_status = types::ResolutionRequestStatus::Accepted;
    success.mission_score = 77.0;
    success.mission_results.push_back({types::MissionRunStatus::Completed, 4, {}});

    testing::InSequence seq;
    EXPECT_CALL(*factory_raw, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillOnce(testing::Throw(std::runtime_error("scenario 1 failed")));
    EXPECT_CALL(*factory_raw, create(testing::_, testing::_, testing::_, testing::_, testing::_))
        .WillOnce([&success](const types::SimulationConfigData&,
                              const types::MissionConfigData&,
                              const types::DroneConfigData&,
                              const types::LidarConfigData&,
                              const std::filesystem::path&) {
            auto run = std::make_unique<testing::StrictMock<MockSimulationRun>>();
            EXPECT_CALL(*run, run()).WillOnce(testing::Return(success));
            return run;
        });

    SimulationManager manager(std::move(factory));
    const auto report = manager.run(comp, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 2U)
        << "Manager must still produce both runs even though the first errored";
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results.front().status, types::MissionRunStatus::Error);

    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, 77.0);
    ASSERT_FALSE(report.runs[1].mission_results.empty());
    EXPECT_EQ(report.runs[1].mission_results.front().status, types::MissionRunStatus::Completed);
}

} // namespace drone_mapper
