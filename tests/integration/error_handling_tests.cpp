#include <gtest/gtest.h>

#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

// Test factories to simulate failures without modifying production code.
class FailingMovement : public drone_mapper::IDroneMovement {
public:
    explicit FailingMovement(drone_mapper::MockGPS& gps) : gps_(gps) {}
    drone_mapper::types::MovementResult rotate(drone_mapper::types::RotationDirection dir, drone_mapper::HorizontalAngle a) override { (void)dir; (void)a; return drone_mapper::types::MovementResult{true, ""}; }
    drone_mapper::types::MovementResult advance(drone_mapper::PhysicalLength dist) override { (void)dist; return drone_mapper::types::MovementResult{false, "collision"}; }
    drone_mapper::types::MovementResult elevate(drone_mapper::PhysicalLength dist) override { (void)dist; return drone_mapper::types::MovementResult{false, "collision"}; }
private:
    drone_mapper::MockGPS& gps_;
};

// A factory that creates runs which will fail during movement (single scenario failure).
class SingleFailureFactory : public drone_mapper::ISimulationRunFactory {
public:
    std::unique_ptr<drone_mapper::ISimulationRun> create(const drone_mapper::types::SimulationConfigData& simulation,
                                                         const drone_mapper::types::MissionConfigData& mission,
                                                         const drone_mapper::types::DroneConfigData& drone,
                                                         const drone_mapper::types::LidarConfigData& lidar,
                                                         const std::filesystem::path& output_path) override {
        auto map_ptr = std::make_shared<NpyArray>();
        const char* err = map_ptr->LoadNPY(simulation.map_filename.string().c_str());
        if (err != nullptr) throw std::runtime_error(std::string("Failed to load NPY: ") + err);

        const drone_mapper::types::MapConfig hidden_map_config{drone_mapper::types::MappingBounds{}, simulation.map_offset, simulation.map_resolution};
        auto hidden_map = std::make_unique<drone_mapper::Map3DImpl>(map_ptr, hidden_map_config);

        const drone_mapper::types::MapConfig output_map_config{hidden_map_config.boundaries, hidden_map_config.offset, mission.gps_resolution};
        auto output_map = std::make_unique<drone_mapper::Map3DImpl>(std::make_shared<NpyArray>(), output_map_config);

        auto gps = std::make_unique<drone_mapper::MockGPS>(
            simulation.initial_drone_position,
            drone_mapper::Orientation{simulation.initial_angle, 0.0 * drone_mapper::altitude_angle[drone_mapper::deg]},
            mission.gps_resolution);
        auto movement = std::make_unique<FailingMovement>(*gps);
        auto lidar_impl = std::make_unique<drone_mapper::MockLidar>(lidar, *hidden_map, *gps);
        auto mapping_algorithm = std::make_unique<drone_mapper::MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

        auto drone_control = std::make_unique<drone_mapper::DroneControlImpl>(drone, mission, *lidar_impl, *gps, *movement, *output_map, *mapping_algorithm);
        drone_control->setLidarConfig(lidar);

        const std::filesystem::path output_results = output_path / "output_results";
        std::filesystem::create_directories(output_results);
        static int counter = 0;
        const std::filesystem::path output_map_file = output_results / ("output_map_fail_" + std::to_string(counter++) + ".npy");

        auto mission_control = std::make_unique<drone_mapper::MissionControlImpl>(mission, drone, *hidden_map, *output_map, *drone_control, output_map_file);

        return std::make_unique<drone_mapper::SimulationRunImpl>(std::move(hidden_map),
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

// A factory that returns a dummy run reporting a map-load failure (group failure simulation).
class MissingMapFactory : public drone_mapper::ISimulationRunFactory {
public:
    std::unique_ptr<drone_mapper::ISimulationRun> create(const drone_mapper::types::SimulationConfigData& simulation,
                                                         const drone_mapper::types::MissionConfigData& mission,
                                                         const drone_mapper::types::DroneConfigData& drone,
                                                         const drone_mapper::types::LidarConfigData& lidar,
                                                         const std::filesystem::path& output_path) override {
        if (!std::filesystem::exists(simulation.map_filename)) {
            struct DummyRun : public drone_mapper::ISimulationRun {
                drone_mapper::types::SimulationResult run() override {
                    drone_mapper::types::SimulationResult r;
                    drone_mapper::types::MissionRunResult mr;
                    mr.status = drone_mapper::types::MissionRunStatus::Error;
                    mr.errors.push_back(drone_mapper::types::ErrorRef{"MAP_LOAD_FAILED", "missing map"});
                    r.mission_results.push_back(mr);
                    r.mission_score = -1.0;
                    return r;
                }
            };
            return std::make_unique<DummyRun>();
        }

        auto realFactory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        return realFactory->create(simulation, mission, drone, lidar, output_path);
    }
};

/*
 * What it does: checks single-run error recovery.
 * Setup: a test factory creates a run that fails during movement.
 * Checks: the failed run gets score -1 and the manager does not crash.
 */
TEST(Integration, SingleScenarioFailureDoesNotCrashAndReturnsMinusOne) {
    auto factory = std::make_unique<SingleFailureFactory>();
    drone_mapper::SimulationManager manager(std::move(factory));

    drone_mapper::types::SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(
        drone_mapper::types::SimulationConfigData{"data_maps/single_voxel_x2_y4_z2.npy", 10.0 * drone_mapper::cm, drone_mapper::Position3D{}, drone_mapper::Position3D{}, 0.0 * drone_mapper::horizontal_angle[drone_mapper::deg]},
        std::vector{drone_mapper::types::MissionConfigData{.max_steps=10, .gps_resolution=10.0*drone_mapper::cm, .output_mapping_resolution_factor=1.0}});
    comp.drones.push_back(drone_mapper::types::DroneConfigData{30.0 * drone_mapper::cm, 45.0 * drone_mapper::horizontal_angle[drone_mapper::deg], 50.0 * drone_mapper::cm, 40.0 * drone_mapper::cm});
    comp.lidars.push_back(drone_mapper::types::LidarConfigData{20.0 * drone_mapper::cm, 120.0 * drone_mapper::cm, 2.5 * drone_mapper::cm, 5});

    auto report = manager.run(comp, std::filesystem::current_path());
    ASSERT_FALSE(report.runs.empty());
    EXPECT_EQ(report.runs.front().mission_score, -1.0);
    ASSERT_FALSE(report.runs.front().mission_results.empty());
    EXPECT_EQ(report.runs.front().mission_results.front().status, drone_mapper::types::MissionRunStatus::Error);
}

/*
 * What it does: checks group-level failure handling.
 * Setup: a test factory reports map-load failure for all runs in the group.
 * Checks: each affected run receives score -1 with an error code.
 */
TEST(Integration, GroupScenarioMissingMapAssignsMinusOneToAll) {
    auto factory = std::make_unique<MissingMapFactory>();
    drone_mapper::SimulationManager manager(std::move(factory));

    drone_mapper::types::SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(
        drone_mapper::types::SimulationConfigData{"data_maps/this_file_does_not_exist.npy", 10.0 * drone_mapper::cm, drone_mapper::Position3D{}, drone_mapper::Position3D{}, 0.0 * drone_mapper::horizontal_angle[drone_mapper::deg]},
        std::vector{drone_mapper::types::MissionConfigData{.max_steps=1, .gps_resolution=10.0*drone_mapper::cm, .output_mapping_resolution_factor=1.0}});
    comp.simulation_mission_groups.emplace_back(
        drone_mapper::types::SimulationConfigData{"data_maps/this_file_does_not_exist.npy", 10.0 * drone_mapper::cm, drone_mapper::Position3D{}, drone_mapper::Position3D{}, 0.0 * drone_mapper::horizontal_angle[drone_mapper::deg]},
        std::vector{drone_mapper::types::MissionConfigData{.max_steps=1, .gps_resolution=10.0*drone_mapper::cm, .output_mapping_resolution_factor=1.0}});
    comp.drones.push_back(drone_mapper::types::DroneConfigData{30.0 * drone_mapper::cm, 45.0 * drone_mapper::horizontal_angle[drone_mapper::deg], 50.0 * drone_mapper::cm, 40.0 * drone_mapper::cm});
    comp.lidars.push_back(drone_mapper::types::LidarConfigData{20.0 * drone_mapper::cm, 120.0 * drone_mapper::cm, 2.5 * drone_mapper::cm, 5});

    auto report = manager.run(comp, std::filesystem::current_path());
    ASSERT_EQ(report.runs.size(), 2U);
    for (const auto& r : report.runs) {
        EXPECT_EQ(r.mission_score, -1.0);
        ASSERT_FALSE(r.mission_results.empty());
        EXPECT_EQ(r.mission_results.front().status, drone_mapper::types::MissionRunStatus::Error);
        ASSERT_FALSE(r.mission_results.front().errors.empty());
        EXPECT_EQ(r.mission_results.front().errors.front().code, "MAP_LOAD_FAILED");
    }
}
