#include <gtest/gtest.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

TEST(Integration, FullFlowRealMapping) {
    auto factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager manager(std::move(factory));

    drone_mapper::types::SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(
        drone_mapper::types::SimulationConfigData{
            "data_maps/single_voxel_x2_y4_z2.npy",
            10.0 * drone_mapper::cm,
            drone_mapper::Position3D{},
            drone_mapper::Position3D{},
            0.0 * drone_mapper::horizontal_angle[drone_mapper::deg]
        },
        std::vector{drone_mapper::types::MissionConfigData{1, 10.0 * drone_mapper::cm, {}, {}, 1}});
    comp.drones.push_back(drone_mapper::types::DroneConfigData{30.0 * drone_mapper::cm, 45.0 * drone_mapper::horizontal_angle[drone_mapper::deg], 50.0 * drone_mapper::cm, 40.0 * drone_mapper::cm});
    comp.lidars.push_back(drone_mapper::types::LidarConfigData{20.0 * drone_mapper::cm, 120.0 * drone_mapper::cm, 2.5 * drone_mapper::cm, 5});

    auto report = manager.run(comp, std::filesystem::current_path());
    EXPECT_GT(report.runs.size(), 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// FullFlowRealisticScenario
//
// Runs the real algorithm end-to-end on an existing map with enough steps
// to fully explore it, then asserts a high mapping score.
// Uses the same factory pattern as FullFlowRealMapping but with proper
// config so the algorithm can complete successfully.
// ─────────────────────────────────────────────────────────────────────────────
TEST(Integration, FullFlowRealisticScenario) {
    auto factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager manager(std::move(factory));

    drone_mapper::types::SimulationCompositionData comp;

    // Map: 5×5×5 at 10cm/voxel → world [0,50]³ cm, one obstacle at (40,40,40).
    // Drone starts at cell centre (20,20,20) — a multiple of gps_resolution —
    // so toGrid() and the planned movement both agree on the cell boundary.
    drone_mapper::types::SimulationConfigData sim_cfg;
    sim_cfg.map_filename             = "data_maps/single_voxel_x4_y4_z4.npy";
    sim_cfg.map_resolution           = 10.0 * drone_mapper::cm;
    sim_cfg.map_offset               = drone_mapper::Position3D{};
    sim_cfg.initial_drone_position   = drone_mapper::Position3D{
        20.0 * drone_mapper::x_extent[drone_mapper::cm],
        20.0 * drone_mapper::y_extent[drone_mapper::cm],
        20.0 * drone_mapper::z_extent[drone_mapper::cm]};
    sim_cfg.initial_angle            = 0.0 * drone_mapper::horizontal_angle[drone_mapper::deg];

    drone_mapper::types::MissionConfigData mission_cfg;
    mission_cfg.max_steps                        = 2000;
    mission_cfg.gps_resolution                   = 10.0 * drone_mapper::cm;
    mission_cfg.output_mapping_resolution_factor = 1;
    // Boundaries must match the map so the algorithm and MockMovement
    // agree on what is "in-bounds".
    mission_cfg.boundaries.min_x      =  5.0 * drone_mapper::x_extent[drone_mapper::cm];
    mission_cfg.boundaries.max_x      = 35.0 * drone_mapper::x_extent[drone_mapper::cm];
    mission_cfg.boundaries.min_y      =  5.0 * drone_mapper::y_extent[drone_mapper::cm];
    mission_cfg.boundaries.max_y      = 35.0 * drone_mapper::y_extent[drone_mapper::cm];
    mission_cfg.boundaries.min_height =  5.0 * drone_mapper::z_extent[drone_mapper::cm];
    mission_cfg.boundaries.max_height = 35.0 * drone_mapper::z_extent[drone_mapper::cm];

    comp.simulation_mission_groups.emplace_back(sim_cfg, std::vector{mission_cfg});
    comp.drones.push_back(drone_mapper::types::DroneConfigData{
        3.0 * drone_mapper::cm,
        45.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
        10.0 * drone_mapper::cm,
        10.0 * drone_mapper::cm});
    comp.lidars.push_back(drone_mapper::types::LidarConfigData{
        2.0 * drone_mapper::cm,
        90.0 * drone_mapper::cm,
        2.5 * drone_mapper::cm,
        3});

    const auto report = manager.run(comp, std::filesystem::current_path());

    ASSERT_EQ(report.runs.size(), 1U);
    const auto& result = report.runs.front();
    ASSERT_FALSE(result.mission_results.empty());
    EXPECT_NE(result.mission_results.front().status,
              drone_mapper::types::MissionRunStatus::Error)
        << (!result.mission_results.front().errors.empty()
                ? result.mission_results.front().errors.front().message
                : "unknown error");
    EXPECT_GT(result.mission_score, 80.0)
        << "Score: " << result.mission_score;
}
