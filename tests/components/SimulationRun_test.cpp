#include <gtest/gtest.h>

#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/ScanResultToVoxels.h>

TEST(SimulationRun, MockGPSAndMovement) {
    drone_mapper::Position3D pos{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    drone_mapper::Orientation head{
        0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
        0.0 * drone_mapper::altitude_angle[drone_mapper::deg]};
    drone_mapper::MockGPS gps(pos, head, 10.0 * drone_mapper::cm);
    drone_mapper::MockMovement movement(gps);

    // Rotate left by 90 degrees
    auto res = movement.rotate(
        drone_mapper::types::RotationDirection::Left,
        90.0 * drone_mapper::horizontal_angle[drone_mapper::deg]);
    EXPECT_TRUE(res);
    auto new_heading = gps.heading();
    EXPECT_NEAR(new_heading.horizontal.numerical_value_in(drone_mapper::deg), 90.0, 1e-6);
}

TEST(SimulationRun, ScanResultToVoxelsApplyToMapMarksFreeRayAndHit) {
    // Create a small output map to apply voxels to
    drone_mapper::types::MapConfig cfg;
    cfg.resolution = 1.0 * drone_mapper::cm;
    cfg.boundaries.min_x = drone_mapper::XLength{-50.0 * drone_mapper::cm};
    cfg.boundaries.max_x = drone_mapper::XLength{50.0 * drone_mapper::cm};
    cfg.boundaries.min_y = drone_mapper::YLength{-50.0 * drone_mapper::cm};
    cfg.boundaries.max_y = drone_mapper::YLength{50.0 * drone_mapper::cm};
    cfg.boundaries.min_height = drone_mapper::ZLength{-50.0 * drone_mapper::cm};
    cfg.boundaries.max_height = drone_mapper::ZLength{50.0 * drone_mapper::cm};

    drone_mapper::Map3DImpl output_map(100, 100, 100, cfg);

    drone_mapper::types::LidarConfigData lidar_cfg{0.5 * drone_mapper::cm, 50.0 * drone_mapper::cm, 2.5 * drone_mapper::cm, 1};

    // Scan from origin along x-axis, hit at 3cm
    drone_mapper::types::LidarScanResult scan{drone_mapper::types::LidarHit{
        3.0 * drone_mapper::cm,
        drone_mapper::Orientation{
            0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
            0.0 * drone_mapper::altitude_angle[drone_mapper::deg]}}};

    drone_mapper::ScanResultToVoxels::applyToMap(
        output_map,
        drone_mapper::Position3D{},  // scan origin at (0,0,0)
        drone_mapper::Orientation{
            0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
            0.0 * drone_mapper::altitude_angle[drone_mapper::deg]},
        scan,
        lidar_cfg);

    // Check that 3cm along x is occupied
    const auto at_3cm = output_map.atVoxel(drone_mapper::Position3D{
        3.0 * drone_mapper::x_extent[drone_mapper::cm],
        0.0 * drone_mapper::y_extent[drone_mapper::cm],
        0.0 * drone_mapper::z_extent[drone_mapper::cm]});
    EXPECT_EQ(at_3cm, drone_mapper::types::VoxelOccupancy::Occupied);

    // Check that 1cm along x is empty (free path before the hit)
    const auto at_1cm = output_map.atVoxel(drone_mapper::Position3D{
        1.0 * drone_mapper::x_extent[drone_mapper::cm],
        0.0 * drone_mapper::y_extent[drone_mapper::cm],
        0.0 * drone_mapper::z_extent[drone_mapper::cm]});
    EXPECT_EQ(at_1cm, drone_mapper::types::VoxelOccupancy::Empty);
}

TEST(SimulationRun, MappingAlgorithmDoesNotHoverForever) {
    drone_mapper::types::MissionConfigData mission{4, 1.0 * drone_mapper::cm, {}, 1};
    drone_mapper::types::LidarConfigData lidar{0.1 * drone_mapper::cm, 5.0 * drone_mapper::cm, 1.0 * drone_mapper::cm, 1};
    drone_mapper::types::DroneConfigData drone{15.0 * drone_mapper::cm, 90.0 * drone_mapper::horizontal_angle[drone_mapper::deg], 5.0 * drone_mapper::cm, 5.0 * drone_mapper::cm};

    class DummyMap : public drone_mapper::IMap3D {
    public:
        drone_mapper::types::VoxelOccupancy atVoxel(const drone_mapper::Position3D&) const override { return drone_mapper::types::VoxelOccupancy::Unmapped; }
        drone_mapper::types::MapConfig getMapConfig() const override { return drone_mapper::types::MapConfig{}; }
        bool isInBounds(const drone_mapper::Position3D&) const override { return false; }
    } output_map;

    drone_mapper::MappingAlgorithmImpl algorithm(mission, lidar, drone, output_map);

    bool emitted_movement = false;
    for (std::size_t i = 0; i < 4; ++i) {
        const auto command = algorithm.nextStep(
            drone_mapper::types::DroneState{
                drone_mapper::Position3D{},
                drone_mapper::Orientation{
                    0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
                    0.0 * drone_mapper::altitude_angle[drone_mapper::deg]},
                i},
            nullptr);
        emitted_movement = emitted_movement ||
            (command.movement.has_value() &&
             command.movement->type != drone_mapper::types::MovementCommandType::Hover);
    }

    EXPECT_TRUE(emitted_movement);
}
