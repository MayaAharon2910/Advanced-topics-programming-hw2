#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace {

class MockMappingAlgorithm : public drone_mapper::IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    MOCK_METHOD(drone_mapper::types::MappingStepCommand,
                nextStep,
                (const drone_mapper::types::DroneState& state,
                 const drone_mapper::types::LidarScanResult* latest_scan),
                (override));
};

class MockMappingFactory final : public drone_mapper::ISimulationRunFactory {
public:
    std::unique_ptr<drone_mapper::ISimulationRun> create(
        const drone_mapper::types::SimulationConfigData& simulation,
        const drone_mapper::types::MissionConfigData& mission,
        const drone_mapper::types::DroneConfigData& drone,
        const drone_mapper::types::LidarConfigData& lidar,
        const std::filesystem::path& output_path) override {
        auto map_ptr = std::make_shared<NpyArray>();
        const char* err = map_ptr->LoadNPY(simulation.map_filename.string().c_str());
        if (err != nullptr) {
            throw std::runtime_error(std::string("Failed to load NPY: ") + err);
        }

        drone_mapper::types::MapConfig hidden_map_config{
            drone_mapper::types::MappingBounds{},
            simulation.map_offset,
            simulation.map_resolution,
        };
        auto hidden_map = std::make_unique<drone_mapper::Map3DImpl>(map_ptr, hidden_map_config);

        const auto& shape = map_ptr->Shape();
        drone_mapper::types::MapConfig output_map_config{
            hidden_map->getMapConfig().boundaries,
            simulation.map_offset,
            mission.gps_resolution,
        };
        auto output_map = std::make_unique<drone_mapper::Map3DImpl>(
            shape[0],
            shape[1],
            shape[2],
            output_map_config);

        auto gps = std::make_unique<drone_mapper::MockGPS>(
            simulation.initial_drone_position,
            drone_mapper::Orientation{simulation.initial_angle, 0.0 * drone_mapper::altitude_angle[drone_mapper::deg]},
            mission.gps_resolution);
        auto movement = std::make_unique<drone_mapper::MockMovement>(*gps);
        auto lidar_impl = std::make_unique<drone_mapper::MockLidar>(lidar, *hidden_map, *gps);

        auto mapping_algorithm = std::make_unique<testing::StrictMock<MockMappingAlgorithm>>(
            mission, lidar, drone, *output_map);
        MockMappingAlgorithm* mapping_raw = mapping_algorithm.get();
        // Expect nextStep to be called once and return Finished
        EXPECT_CALL(*mapping_raw, nextStep(testing::_, testing::_))
            .Times(1)
            .WillOnce(testing::Return(drone_mapper::types::MappingStepCommand{
                std::nullopt, // no movement
                std::nullopt, // no scan
                drone_mapper::types::AlgorithmStatus::Finished,
            }));

        auto drone_control = std::make_unique<drone_mapper::DroneControlImpl>(
            drone,
            mission,
            *lidar_impl,
            *gps,
            *movement,
            *output_map,
            *mapping_algorithm);
        drone_control->setLidarConfig(lidar);

        const std::filesystem::path output_results = output_path / "output_results";
        std::filesystem::create_directories(output_results);
        const std::filesystem::path output_map_file = output_results / "output_map_mock.npy";
        auto mission_control = std::make_unique<drone_mapper::MissionControlImpl>(
            mission,
            drone,
            *hidden_map,
            *output_map,
            *drone_control,
            output_map_file);

        return std::make_unique<drone_mapper::SimulationRunImpl>(
            std::move(hidden_map),
            std::move(output_map),
            std::move(gps),
            std::move(movement),
            std::move(lidar_impl),
            std::move(mapping_algorithm),
            std::move(drone_control),
            std::move(mission_control),
            simulation,
            mission,
            drone,
            lidar,
            output_map_file);
    }
};

} // namespace

/*
 * What it does: checks the full pipeline with a mock mapping algorithm.
 * Setup: all components are wired together while the mapping decisions are controlled.
 * Checks: the run completes and produces the expected output structure.
 */
TEST(Integration, FullFlowWithMockMapping) {
    auto factory = std::make_unique<MockMappingFactory>();
    drone_mapper::SimulationManager manager(std::move(factory));

    drone_mapper::types::SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(
        drone_mapper::types::SimulationConfigData{
            "data_maps/single_voxel_x2_y4_z2.npy",
            10.0 * drone_mapper::cm,
            drone_mapper::Position3D{},
            drone_mapper::Position3D{},
            0.0 * drone_mapper::horizontal_angle[drone_mapper::deg]},
        std::vector{drone_mapper::types::MissionConfigData{1, 10.0 * drone_mapper::cm, {}, 1}});
    comp.drones.push_back(drone_mapper::types::DroneConfigData{
        30.0 * drone_mapper::cm,
        45.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
        50.0 * drone_mapper::cm,
        40.0 * drone_mapper::cm});
    comp.lidars.push_back(drone_mapper::types::LidarConfigData{
        20.0 * drone_mapper::cm,
        120.0 * drone_mapper::cm,
        2.5 * drone_mapper::cm,
        5});

    const auto report = manager.run(comp, std::filesystem::current_path());
    ASSERT_EQ(report.runs.size(), 1U);
    ASSERT_FALSE(report.runs.front().mission_results.empty());
    EXPECT_EQ(report.runs.front().mission_results.front().status,
              drone_mapper::types::MissionRunStatus::Completed);
}
