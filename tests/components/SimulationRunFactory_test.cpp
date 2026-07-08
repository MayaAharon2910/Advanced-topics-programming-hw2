// Component tests: SimulationRunFactoryImpl resolution-request handling.

#include <gtest/gtest.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <filesystem>

namespace {

using namespace drone_mapper;
using namespace drone_mapper::types;

SimulationConfigData makeSimConfig() {
    SimulationConfigData s;
    s.map_filename   = "data_maps/single_voxel_x4_y4_z4.npy";
    s.map_resolution = 10.0 * cm;
    s.map_offset     = Position3D{};
    s.initial_drone_position = Position3D{
        20.0 * x_extent[cm], 20.0 * y_extent[cm], 20.0 * z_extent[cm]};
    s.initial_angle = 0.0 * horizontal_angle[deg];
    return s;
}

} // namespace

/*
 * What it does: requests an output mapping resolution factor of 2 on a
 * mission whose gps_resolution is 10cm - the bonus feature, actively
 * honoring a coarser output resolution instead of always ignoring it.
 * Setup: a real SimulationRunFactoryImpl builds a SimulationRunImpl against
 * the tiny synthetic single_voxel_x4_y4_z4 map (10cm native resolution),
 * with gps_resolution_cm=10 and output_mapping_resolution_factor=2.
 * Checks: the resulting output map's resolution is exactly 20cm
 * (gps_resolution * factor), and resolution_request_status is Accepted,
 * not Ignored/IgnoredTooSmall.
 */
TEST(SimulationRunFactory, SupportsDifferentResolutionsBonus) {
    const SimulationConfigData sim_cfg = makeSimConfig();

    MissionConfigData mission;
    mission.max_steps                        = 10;
    mission.gps_resolution                   = 10.0 * cm;
    mission.output_mapping_resolution_factor = 2;
    mission.mission_bounds.min_x      = 0.0 * x_extent[cm];
    mission.mission_bounds.max_x      = 40.0 * x_extent[cm];
    mission.mission_bounds.min_y      = 0.0 * y_extent[cm];
    mission.mission_bounds.max_y      = 40.0 * y_extent[cm];
    mission.mission_bounds.min_height = 0.0 * z_extent[cm];
    mission.mission_bounds.max_height = 40.0 * z_extent[cm];

    const DroneConfigData drone_cfg{3.0 * cm, 45.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm};
    const LidarConfigData lidar_cfg{2.0 * cm, 90.0 * cm, 2.5 * cm, static_cast<std::size_t>(3)};

    const std::filesystem::path out_dir =
        std::filesystem::temp_directory_path() / "simulation_run_factory_resolution_bonus_test";
    std::filesystem::create_directories(out_dir);

    SimulationRunFactoryImpl factory;
    auto run = factory.create(sim_cfg, mission, drone_cfg, lidar_cfg, out_dir);
    const SimulationResult result = run->run();

    EXPECT_DOUBLE_EQ(result.output_map_config.resolution.force_numerical_value_in(cm), 20.0);
    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted);

    std::filesystem::remove_all(out_dir);
}
