/*
 * Small integration tests for movement and YAML parsing.
 * These tests cover simple multi-component behavior that is larger than a single
 * class unit test but still small enough to debug quickly.
 */

#include <gtest/gtest.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/YamlConfig.h>

#include <filesystem>
#include <fstream>

/*
 * What it does: checks a simple rotation path using MockMovement.
 * Setup: creates MockGPS and asks movement to rotate once.
 * Checks: the GPS heading changes by the requested angle.
 */
TEST(Integration, MockMovementRotateHappyPath) {
    drone_mapper::Position3D pos{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    drone_mapper::Orientation heading{0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg};
    drone_mapper::MockGPS gps(pos, heading, 10.0 * drone_mapper::cm);
    drone_mapper::MockMovement movement(gps);
    auto res = movement.rotate(drone_mapper::types::RotationDirection::Left, 45.0 * drone_mapper::deg);
    EXPECT_TRUE(res);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.numerical_value_in(drone_mapper::deg), 45.0);
}

/*
 * What it does: checks advance and elevation movement together.
 * Setup: starts from a known GPS state and applies advance followed by elevate.
 * Checks: the final position matches the expected x/y/z coordinates.
 */
TEST(Integration, MockMovementAdvanceAndElevateHappyPath) {
    drone_mapper::Position3D pos{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    drone_mapper::Orientation heading{90.0 * drone_mapper::deg, 30.0 * drone_mapper::altitude_angle[drone_mapper::deg]};
    drone_mapper::MockGPS gps(pos, heading, 10.0 * drone_mapper::cm);
    drone_mapper::MockMovement movement(gps);

    ASSERT_TRUE(movement.advance(20.0 * drone_mapper::cm));
    EXPECT_NEAR(gps.position().x.numerical_value_in(drone_mapper::cm), 0.0, 1e-9);
    EXPECT_NEAR(gps.position().y.numerical_value_in(drone_mapper::cm), 17.3205080757, 1e-9);
    EXPECT_NEAR(gps.position().z.numerical_value_in(drone_mapper::cm), 10.0, 1e-9);

    ASSERT_TRUE(movement.elevate(5.0 * drone_mapper::cm));
    EXPECT_NEAR(gps.position().z.numerical_value_in(drone_mapper::cm), 15.0, 1e-9);
}

/*
 * What it does: checks loading of a composition that references other YAML files.
 * Setup: writes temporary simulation, mission, drone, and lidar YAML files.
 * Checks: the parser resolves the references and loads all config objects.
 */
TEST(YamlConfig, LoadsReferencedCompositionFiles) {
    const std::filesystem::path dir = std::filesystem::current_path() / "tmp_yaml_reference_test";
    std::filesystem::create_directories(dir);

    {
        std::ofstream(dir / "sim.yaml")
            << "map_filename: data_maps/single_voxel_x2_y4_z2.npy\n"
            << "map_resolution: 10\n"
            << "initial_angle: 0\n";
        std::ofstream(dir / "mission.yaml")
            << "max_steps: 7\n"
            << "gps_resolution: 5\n"
            << "output_mapping_resolution_factor: 1\n";
        std::ofstream(dir / "drone.yaml")
            << "dimensions: 30\n"
            << "max_rotate: 45\n"
            << "max_advance: 50\n"
            << "max_elevate: 40\n";
        std::ofstream(dir / "lidar.yaml")
            << "z_min: 20\n"
            << "z_max: 120\n"
            << "d: 2.5\n"
            << "fov_circles: 2\n";
        std::ofstream(dir / "composition.yaml")
            << "simulations:\n"
            << "  - simulation_config: sim.yaml\n"
            << "mission_configs:\n"
            << "  - mission_config: mission.yaml\n"
            << "drones:\n"
            << "  - drone_config: drone.yaml\n"
            << "lidars:\n"
            << "  - lidar_config: lidar.yaml\n";
    }

    const auto comp = drone_mapper::yaml::parseSimulationComposition(dir / "composition.yaml");
    ASSERT_EQ(comp.simulation_mission_groups.size(), 1U);
    ASSERT_EQ(std::get<1>(comp.simulation_mission_groups.front()).size(), 1U);
    ASSERT_EQ(comp.drones.size(), 1U);
    ASSERT_EQ(comp.lidars.size(), 1U);
    EXPECT_EQ(std::get<1>(comp.simulation_mission_groups.front()).front().max_steps, 7U);
    EXPECT_EQ(comp.lidars.front().fov_circles, 2U);

    std::filesystem::remove_all(dir);
}

/*
 * What it does: checks the default value for a missing resolution factor.
 * Setup: writes a mission YAML without output_mapping_resolution_factor.
 * Checks: the parsed mission uses factor 1.
 */
TEST(YamlConfig, MissingResolutionFactorDefaultsToOne) {
    const std::filesystem::path dir = std::filesystem::current_path() / "tmp_yaml_missing_factor_test";
    std::filesystem::create_directories(dir);

    std::ofstream(dir / "sim.yaml")
        << "map_filename: data_maps/single_voxel_x2_y4_z2.npy\n"
        << "map_resolution: 10\n"
        << "initial_angle: 0\n";
    std::ofstream(dir / "mission.yaml")
        << "max_steps: 5\n"
        << "gps_resolution: 5\n";  // no output_mapping_resolution_factor key
    std::ofstream(dir / "drone.yaml")
        << "dimensions: 30\nmax_rotate: 45\nmax_advance: 50\nmax_elevate: 40\n";
    std::ofstream(dir / "lidar.yaml")
        << "z_min: 20\nz_max: 120\nd: 2.5\nfov_circles: 2\n";
    std::ofstream(dir / "composition.yaml")
        << "simulations:\n  - simulation_config: sim.yaml\n"
        << "mission_configs:\n  - mission_config: mission.yaml\n"
        << "drones:\n  - drone_config: drone.yaml\n"
        << "lidars:\n  - lidar_config: lidar.yaml\n";

    const auto comp = drone_mapper::yaml::parseSimulationComposition(dir / "composition.yaml");
    const auto& mission = std::get<1>(comp.simulation_mission_groups.front()).front();
    EXPECT_DOUBLE_EQ(mission.output_mapping_resolution_factor, 1.0);

    std::filesystem::remove_all(dir);
}

/*
 * What it does: checks that an explicit zero resolution factor is preserved.
 * Setup: writes a mission YAML with output_mapping_resolution_factor set to 0.
 * Checks: the parser keeps the zero value for later validation/error handling.
 */
TEST(YamlConfig, ExplicitZeroResolutionFactorIsPreserved) {
    const std::filesystem::path dir = std::filesystem::current_path() / "tmp_yaml_zero_factor_test";
    std::filesystem::create_directories(dir);

    std::ofstream(dir / "sim.yaml")
        << "map_filename: data_maps/single_voxel_x2_y4_z2.npy\n"
        << "map_resolution: 10\n"
        << "initial_angle: 0\n";
    std::ofstream(dir / "mission.yaml")
        << "max_steps: 5\n"
        << "gps_resolution: 5\n"
        << "output_mapping_resolution_factor: 0\n";
    std::ofstream(dir / "drone.yaml")
        << "dimensions: 30\nmax_rotate: 45\nmax_advance: 50\nmax_elevate: 40\n";
    std::ofstream(dir / "lidar.yaml")
        << "z_min: 20\nz_max: 120\nd: 2.5\nfov_circles: 2\n";
    std::ofstream(dir / "composition.yaml")
        << "simulations:\n  - simulation_config: sim.yaml\n"
        << "mission_configs:\n  - mission_config: mission.yaml\n"
        << "drones:\n  - drone_config: drone.yaml\n"
        << "lidars:\n  - lidar_config: lidar.yaml\n";

    const auto comp = drone_mapper::yaml::parseSimulationComposition(dir / "composition.yaml");
    const auto& mission = std::get<1>(comp.simulation_mission_groups.front()).front();
    EXPECT_DOUBLE_EQ(mission.output_mapping_resolution_factor, 0.0);

    std::filesystem::remove_all(dir);
}
