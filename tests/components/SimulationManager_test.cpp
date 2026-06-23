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

[[nodiscard]] types::SimulationCompositionData singleRunComposition() {
    types::SimulationCompositionData composition{};
    composition.composition_file = "simulation.yaml";
    composition.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{
            "map.npy",
            10.0 * cm,
            Position3D{},
            Position3D{},
            0.0 * horizontal_angle[deg]},
        std::vector{types::MissionConfigData{5, 10.0 * cm, {}, {}, 1}});
    composition.drones.push_back(types::DroneConfigData{
        30.0 * cm,
        45.0 * horizontal_angle[deg],
        50.0 * cm,
        40.0 * cm});
    composition.lidars.push_back(types::LidarConfigData{
        20.0 * cm,
        120.0 * cm,
        2.5 * cm,
        5});
    return composition;
}

} // namespace

TEST(SimulationManager, RunsSingleCartesianCombinationWithFactory) {
    auto factory = std::make_unique<testing::StrictMock<MockSimulationRunFactory>>();
    auto* factory_raw = factory.get();

    types::SimulationResult expected_result{};
    expected_result.mission_config = types::MissionConfigData{5, 10.0 * cm, {}, {}, 1};
    expected_result.resolution_request_status = types::ResolutionRequestStatus::Accepted;
    expected_result.mission_score = 100.0;
    expected_result.mission_results.push_back(types::MissionRunResult{
        types::MissionRunStatus::Completed,
        1,
        {}});

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

} // namespace drone_mapper
