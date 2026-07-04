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

} // namespace drone_mapper
