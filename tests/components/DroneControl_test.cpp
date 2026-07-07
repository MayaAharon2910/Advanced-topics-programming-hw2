// =============================================================================
// DroneControl_test.cpp - Component tests for DroneControlImpl
// DroneControlImpl is the step executor: it asks the algorithm for the next
// command, calls MockMovement to carry it out, fires the lidar if a scan
// was requested, applies voxels to the output map, and returns a step result.
// All tests replace the algorithm with MockAlgorithm (GMock) and the movement
// with a local mock, so bugs in those components never affect these tests.
// DummyGPS, DummyLidar, and DummyMap provide silent stubs for everything else.
// =============================================================================
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
    types::LidarConfigData config() const override { return {}; }
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
types::MissionConfigData missionConfig() { return {10, 10.0*cm, 1, {}}; }
types::LidarConfigData   lidarConfig()   { return {20.0*cm, 120.0*cm, 2.5*cm, 5}; }

/*
 * What it does: executes an Advance command returned by the mapping algorithm.
 * Setup: the algorithm is mocked to return Advance and movement reports success.
 * Checks: DroneControl calls advance() and continues the mission.
 */
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

/*
 * What it does: handles a movement failure from the drone movement layer.
 * Setup: the algorithm returns Advance, but the movement object returns failure.
 * Checks: step() returns Error instead of continuing.
 */
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

/*
 * What it does: handles an algorithm-finished status.
 * Setup: the mocked algorithm returns Finished without a movement command.
 * Checks: DroneControl reports Completed.
 */
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

/*
 * What it does: forwards the latest lidar scan to the algorithm on the following step.
 * Setup: the first step requests a scan and the second step expects a non-null scan pointer.
 * Checks: the scan result is preserved between steps.
 */
TEST(DroneControl, ScanResultPassedToAlgorithmOnNextStep) {
    DummyMap output_map;
    DummyGPS gps;

    // Lidar that returns one hit
    class HitLidar : public ILidar {
    public:
        types::LidarScanResult scan(Orientation) const override {
            return {types::LidarHit{5.0*cm, Orientation{0.0*deg, 0.0*deg}}};
        }
        types::LidarConfigData config() const override { return {}; }
    } lidar;

    class NoOpMovement : public IDroneMovement {
    public:
        types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override { return {true, {}}; }
        types::MovementResult advance(PhysicalLength) override { return {true, {}}; }
        types::MovementResult elevate(PhysicalLength) override { return {true, {}}; }
    } movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);

    // Step 1: return a scan command -> lidar fires, stores result
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
    (void)ctrl.step(); // step 2 - verifies non-null scan pointer
}


class MockMoveGMock : public drone_mapper::IDroneMovement {
public:
    MOCK_METHOD(drone_mapper::types::MovementResult, rotate,
                (drone_mapper::types::RotationDirection, drone_mapper::HorizontalAngle), (override));
    MOCK_METHOD(drone_mapper::types::MovementResult, advance,
                (drone_mapper::PhysicalLength), (override));
    MOCK_METHOD(drone_mapper::types::MovementResult, elevate,
                (drone_mapper::PhysicalLength), (override));
};

/*
 * What it does: executes a Rotate command returned by the algorithm.
 * Setup: the mocked algorithm returns a left rotation command.
 * Checks: DroneControl calls rotate() and continues.
 */
TEST(DroneControl, RotateCommandCallsRotate) {
    DummyMap output_map;
    DummyLidar lidar;
    DummyGPS gps;
    MockMoveGMock movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);
    ON_CALL(alg, nextStep(testing::_, testing::_))
        .WillByDefault(testing::Return(drone_mapper::types::MappingStepCommand{
            drone_mapper::types::MovementCommand{
                drone_mapper::types::MovementCommandType::Rotate,
                drone_mapper::types::RotationDirection::Left,
                30.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
                0.0 * drone_mapper::cm},
            std::nullopt,
            drone_mapper::types::AlgorithmStatus::Working}));
    EXPECT_CALL(movement, rotate(testing::_, testing::_))
        .WillOnce(testing::Return(drone_mapper::types::MovementResult{true, ""}));

    drone_mapper::DroneControlImpl ctrl(
        droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    EXPECT_EQ(ctrl.step().status, drone_mapper::types::DroneStepStatus::Continue);
}

/*
 * What it does: executes an Elevate command returned by the algorithm.
 * Setup: the mocked algorithm returns an elevation command.
 * Checks: DroneControl calls elevate() and continues.
 */
TEST(DroneControl, ElevateCommandCallsElevate) {
    DummyMap output_map;
    DummyLidar lidar;
    DummyGPS gps;
    MockMoveGMock movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);
    ON_CALL(alg, nextStep(testing::_, testing::_))
        .WillByDefault(testing::Return(drone_mapper::types::MappingStepCommand{
            drone_mapper::types::MovementCommand{
                drone_mapper::types::MovementCommandType::Elevate,
                drone_mapper::types::RotationDirection::Left,
                0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
                10.0 * drone_mapper::cm},
            std::nullopt,
            drone_mapper::types::AlgorithmStatus::Working}));
    EXPECT_CALL(movement, elevate(testing::_))
        .WillOnce(testing::Return(drone_mapper::types::MovementResult{true, ""}));

    drone_mapper::DroneControlImpl ctrl(
        droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    EXPECT_EQ(ctrl.step().status, drone_mapper::types::DroneStepStatus::Continue);
}

/*
 * What it does: handles Hover without moving the drone.
 * Setup: the mocked algorithm returns a hover command.
 * Checks: advance(), rotate(), and elevate() are not called.
 */
TEST(DroneControl, HoverCommandCallsNeitherAdvanceNorRotate) {
    DummyMap output_map;
    DummyLidar lidar;
    DummyGPS gps;
    MockMoveGMock movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);
    ON_CALL(alg, nextStep(testing::_, testing::_))
        .WillByDefault(testing::Return(drone_mapper::types::MappingStepCommand{
            drone_mapper::types::MovementCommand{
                drone_mapper::types::MovementCommandType::Hover,
                drone_mapper::types::RotationDirection::Left,
                0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
                0.0 * drone_mapper::cm},
            std::nullopt,
            drone_mapper::types::AlgorithmStatus::Working}));
    EXPECT_CALL(movement, advance(testing::_)).Times(0);
    EXPECT_CALL(movement, rotate(testing::_, testing::_)).Times(0);
    EXPECT_CALL(movement, elevate(testing::_)).Times(0);

    drone_mapper::DroneControlImpl ctrl(
        droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    std::ignore = ctrl.step();
}

/*
 * What it does: treats FinishedWithUnmappableVoxels as a clean mission end.
 * Setup: the mocked algorithm returns FinishedWithUnmappableVoxels.
 * Checks: DroneControl reports Completed.
 */
TEST(DroneControl, UnmappableStatusReturnsCompleted) {
    DummyMap output_map;
    DummyLidar lidar;
    DummyGPS gps;
    class NoOpMove : public drone_mapper::IDroneMovement {
    public:
        drone_mapper::types::MovementResult rotate(
            drone_mapper::types::RotationDirection, drone_mapper::HorizontalAngle) override { return {true, {}}; }
        drone_mapper::types::MovementResult advance(drone_mapper::PhysicalLength) override { return {true, {}}; }
        drone_mapper::types::MovementResult elevate(drone_mapper::PhysicalLength) override { return {true, {}}; }
    } movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);
    EXPECT_CALL(alg, nextStep(testing::_, testing::_))
        .WillOnce(testing::Return(drone_mapper::types::MappingStepCommand{
            std::nullopt, std::nullopt,
            drone_mapper::types::AlgorithmStatus::FinishedWithUnmappableVoxels}));

    drone_mapper::DroneControlImpl ctrl(
        droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    EXPECT_EQ(ctrl.step().status, drone_mapper::types::DroneStepStatus::Completed);
}

/*
 * What it does: keeps step state across multiple calls.
 * Setup: step() is called repeatedly with mocked algorithm responses.
 * Checks: the component progresses instead of reusing the first step forever.
 */
TEST(DroneControl, StepIndexIncrementsAcrossCalls) {
    DummyMap output_map;
    DummyLidar lidar;
    DummyGPS gps;
    class NoOpMove : public drone_mapper::IDroneMovement {
    public:
        drone_mapper::types::MovementResult rotate(
            drone_mapper::types::RotationDirection, drone_mapper::HorizontalAngle) override { return {true, {}}; }
        drone_mapper::types::MovementResult advance(drone_mapper::PhysicalLength) override { return {true, {}}; }
        drone_mapper::types::MovementResult elevate(drone_mapper::PhysicalLength) override { return {true, {}}; }
    } movement;

    std::vector<std::size_t> step_indices;
    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);
    EXPECT_CALL(alg, nextStep(testing::_, testing::_))
        .Times(3)
        .WillRepeatedly([&](const drone_mapper::types::DroneState& state,
                            const drone_mapper::types::LidarScanResult*) {
            step_indices.push_back(state.step_index);
            return drone_mapper::types::MappingStepCommand{
                std::nullopt, std::nullopt, drone_mapper::types::AlgorithmStatus::Working};
        });

    drone_mapper::DroneControlImpl ctrl(
        droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    std::ignore = ctrl.step(); std::ignore = ctrl.step(); std::ignore = ctrl.step();

    ASSERT_EQ(step_indices.size(), 3U);
    EXPECT_LT(step_indices[0], step_indices[1]);
    EXPECT_LT(step_indices[1], step_indices[2]);
}

/*
 * What it does: verifies the move+scan fusion optimization is wired
 * correctly at the DroneControl execution level (MappingStepCommand
 * explicitly allows carrying both fields at once - "movement must be
 * performed before scan").
 * Setup: the mocked algorithm returns ONE command with both an Advance
 * movement AND a scan_orientation set. Movement and lidar are both GMock
 * objects placed in a single InSequence so call order is enforced.
 * Checks: advance() is called before scan(); the fused scan's result is
 * forwarded to the algorithm's next nextStep() call, exactly like a
 * standalone scan would be.
 */
TEST(DroneControl, FusedMoveAndScanExecutesMovementBeforeScanAndForwardsResult) {
    DummyMap output_map;
    DummyGPS gps;

    class MockLidarGMock : public ILidar {
    public:
        MOCK_METHOD(types::LidarScanResult, scan, (Orientation), (const, override));
        types::LidarConfigData config() const override { return {}; }
    } lidar;
    MockMoveGMock movement;

    MockAlgorithm alg(missionConfig(), lidarConfig(), droneConfig(), output_map);

    {
        testing::InSequence seq;
        EXPECT_CALL(movement, advance(testing::_))
            .WillOnce(testing::Return(types::MovementResult{true, ""}));
        EXPECT_CALL(lidar, scan(testing::_))
            .WillOnce(testing::Return(types::LidarScanResult{
                types::LidarHit{5.0*cm, Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]}}}));
    }

    // Step 1: command carries BOTH movement and scan_orientation - the
    // fused case idea #1 introduced.
    EXPECT_CALL(alg, nextStep(testing::_, testing::IsNull()))
        .WillOnce(testing::Return(types::MappingStepCommand{
            types::MovementCommand{types::MovementCommandType::Advance,
                                   types::RotationDirection::Left,
                                   0.0*horizontal_angle[deg], 5.0*cm},
            Orientation{0.0*horizontal_angle[deg], 0.0*altitude_angle[deg]},
            types::AlgorithmStatus::Working}));

    // Step 2: the fused step's scan result must be forwarded here, exactly
    // like a standalone scan (see ScanResultPassedToAlgorithmOnNextStep).
    EXPECT_CALL(alg, nextStep(testing::_, testing::Not(testing::IsNull())))
        .WillOnce(testing::Return(types::MappingStepCommand{
            std::nullopt, std::nullopt, types::AlgorithmStatus::Finished}));

    DroneControlImpl ctrl(droneConfig(), missionConfig(), lidar, gps, movement, output_map, alg);
    ctrl.setLidarConfig(lidarConfig());
    EXPECT_EQ(ctrl.step().status, types::DroneStepStatus::Continue);  // step 1: fused move+scan
    EXPECT_EQ(ctrl.step().status, types::DroneStepStatus::Completed); // step 2: verifies forwarding
}

} // namespace drone_mapper
