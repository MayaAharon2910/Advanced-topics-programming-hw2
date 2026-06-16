#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>

namespace drone_mapper {

// ── Shared test doubles ───────────────────────────────────────────────────────
class DummyMap : public IMutableMap3D {
public:
    types::VoxelOccupancy atVoxel(const Position3D&)   const override { return types::VoxelOccupancy::Empty; }
    types::MapConfig      getMapConfig()                const override { return types::MapConfig{}; }
    bool                  isInBounds(const Position3D&) const override { return true; }
    MOCK_METHOD(void, set,  (const Position3D&, types::VoxelOccupancy), (override));
    void save(const std::filesystem::path&) const override {}
};

class DummyLidar : public ILidar {
public:
    types::LidarScanResult scan(Orientation) const override { return {}; }
};

class DummyGPS : public IGPS {
public:
    Position3D position() const override { return Position3D{}; }
    Orientation heading()  const override { return Orientation{}; }
};

class MockAlgorithm : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    MOCK_METHOD(types::MappingStepCommand, nextStep,
                (const types::DroneState&, const types::LidarScanResult*), (override));
};

// Helpers to build common config objects
types::DroneConfigData   droneConfig()   { return {30.0*cm, 45.0*horizontal_angle[deg], 50.0*cm, 40.0*cm}; }
types::MissionConfigData missionConfig() { return {10, 10.0*cm, {}, 1}; }
types::LidarConfigData   lidarConfig()   { return {20.0*cm, 120.0*cm, 2.5*cm, 5}; }

// ── Test 1: happy path advance ───────────────────────────────────────────────
TEST(DroneControl, ExecutesAdvanceCommand) {
    DummyMap output_map;
    DummyLidar lidar;
    DummyGPS   gps;

    class MockMove : public IDroneMovement {
    public:
        MOCK_METHOD(types::MovementResult, rotate,  (types::RotationDirection, HorizontalAngle), (override));
        MOCK_METHOD(types::MovementResult, advance, (PhysicalLength), (override));
        types::MovementResult elevate(PhysicalLength) override { return {true, {}}; }
    } movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);
    EXPECT_CALL(alg, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            types::MovementCommand{types::MovementCommandType::Advance,
                                   types::RotationDirection::Left,
                                   0.0*horizontal_angle[deg], 5.0*cm},
            std::nullopt,
            types::AlgorithmStatus::Working}));
    EXPECT_CALL(movement, advance(testing::_))
        .WillOnce(testing::Return(types::MovementResult{true, ""}));

    DroneControlImpl ctrl(droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    EXPECT_EQ(ctrl.step().status, types::DroneStepStatus::Continue);
}

// ── Test 2: movement returns false → Error status ────────────────────────────
TEST(DroneControl, MovementFailureReturnsError) {
    DummyMap output_map;
    DummyLidar lidar;
    DummyGPS   gps;

    class FailMovement : public IDroneMovement {
    public:
        types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override { return {true, {}}; }
        types::MovementResult advance(PhysicalLength) override { return {false, "obstacle"}; }
        types::MovementResult elevate(PhysicalLength) override { return {true, {}}; }
    } movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);
    EXPECT_CALL(alg, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            types::MovementCommand{types::MovementCommandType::Advance,
                                   types::RotationDirection::Left,
                                   0.0*horizontal_angle[deg], 5.0*cm},
            std::nullopt,
            types::AlgorithmStatus::Working}));

    DroneControlImpl ctrl(droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    EXPECT_EQ(ctrl.step().status, types::DroneStepStatus::Error);
}

// ── Test 3: algorithm returns Finished → Completed status ────────────────────
TEST(DroneControl, AlgorithmFinishedReturnsCompleted) {
    DummyMap output_map;
    DummyLidar lidar;
    DummyGPS gps;

    class NoOpMovement : public IDroneMovement {
    public:
        types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override { return {true, {}}; }
        types::MovementResult advance(PhysicalLength) override { return {true, {}}; }
        types::MovementResult elevate(PhysicalLength) override { return {true, {}}; }
    } movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);
    EXPECT_CALL(alg, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));

    DroneControlImpl ctrl(droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    EXPECT_EQ(ctrl.step().status, types::DroneStepStatus::Completed);
}

// ── Test 4: scan result from previous step is passed to nextStep ─────────────
TEST(DroneControl, ScanResultPassedToAlgorithmOnNextStep) {
    DummyMap output_map;
    DummyGPS gps;

    // Lidar that returns one hit
    class HitLidar : public ILidar {
    public:
        types::LidarScanResult scan(Orientation) const override {
            return {types::LidarHit{5.0*cm, Orientation{0.0*deg, 0.0*deg}}};
        }
    } lidar;

    class NoOpMovement : public IDroneMovement {
    public:
        types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override { return {true, {}}; }
        types::MovementResult advance(PhysicalLength) override { return {true, {}}; }
        types::MovementResult elevate(PhysicalLength) override { return {true, {}}; }
    } movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);

    // Step 1: return a scan command → lidar fires, stores result
    EXPECT_CALL(alg, nextStep(testing::_, testing::IsNull()))
        .WillOnce(testing::Return(types::MappingStepCommand{
            std::nullopt,
            Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
            types::AlgorithmStatus::Working}));

    // Step 2: scan pointer must be non-null (previous result forwarded)
    EXPECT_CALL(alg, nextStep(testing::_, testing::Not(testing::IsNull())))
        .WillOnce(testing::Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));

    DroneControlImpl ctrl(droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    (void)ctrl.step(); // step 1
    (void)ctrl.step(); // step 2 — verifies non-null scan pointer
}

} // namespace drone_mapper
