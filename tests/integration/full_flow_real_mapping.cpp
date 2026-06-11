#include <gtest/gtest.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

TEST(SimulationManager, FullFlowRealMapping) {
    auto factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager manager(std::move(factory));

    drone_mapper::types::SimulationCompositionData comp;
    // Minimal composition: reuse default sample in main
    comp.simulations.push_back(drone_mapper::types::SimulationConfigData{
        "data_maps/single_voxel_x2_y4_z2.npy",
        10.0 * drone_mapper::cm,
        drone_mapper::Position3D{},
        drone_mapper::Position3D{},
        0.0 * drone_mapper::horizontal_angle[drone_mapper::deg]
    });
    comp.missions.push_back(drone_mapper::types::MissionConfigData{1, 10.0 * drone_mapper::cm, 1});
    comp.drones.push_back(drone_mapper::types::DroneConfigData{30.0 * drone_mapper::cm, 45.0 * drone_mapper::horizontal_angle[drone_mapper::deg], 50.0 * drone_mapper::cm, 40.0 * drone_mapper::cm});
    comp.lidars.push_back(drone_mapper::types::LidarConfigData{20.0 * drone_mapper::cm, 120.0 * drone_mapper::cm, 2.5 * drone_mapper::cm, 5});

    auto report = manager.run(comp, std::filesystem::current_path());
    EXPECT_GT(report.runs.size(), 0);
}
