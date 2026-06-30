// =============================================================================
// MissionControl_test.cpp - Component tests for MissionControlImpl
//
// MissionControlImpl drives the per-mission step loop: it calls DroneControl
// step-by-step until max_steps, Completed, or Error; then saves the output map.
//
// All tests replace DroneControl with a lightweight fake (DummyDroneControl or
// CountingDroneControl) so that DroneControl bugs never affect these tests.
// The output map is a partial mock: only save() is mocked; all other methods
// use simple stub implementations.
// =============================================================================

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/MissionControlImpl.h>

namespace drone_mapper {
namespace {

// Output map stub - only save() is a GMock method so we can assert it
// is (or is not) called the expected number of times.
class DummyMap final : public IMutableMap3D {
public:
    explicit DummyMap(types::MapConfig cfg) : cfg_(cfg) {}
    types::VoxelOccupancy atVoxel(const Position3D&) const override { return types::VoxelOccupancy::Empty; }
    types::MapConfig getMapConfig() const override { return cfg_; }
    bool isInBounds(const Position3D&) const override { return true; }
    void set(const Position3D&, types::VoxelOccupancy) override {}
    MOCK_METHOD(void, save, (const std::filesystem::path& output_path), (const, override));
private:
    types::MapConfig cfg_{};
};

// Simple drone control stub that always returns the same status.
class DummyDroneControl final : public IDroneControl {
public:
    explicit DummyDroneControl(types::DroneStepStatus status) : status_(status) {}
    types::DroneStepResult step() override { return types::DroneStepResult{status_, "step"}; }
    types::DroneState state() const override { return {}; }
private:
    types::DroneStepStatus status_;
};

// Drone control that returns Continue for `continue_steps` calls, then
// returns `final_status`. Used to test exact step counts and completion paths.
class CountingDroneControl final : public drone_mapper::IDroneControl {
public:
    CountingDroneControl(int continue_steps,
                         drone_mapper::types::DroneStepStatus final_status,
                         const std::string& error_msg = "")
        : remain_(continue_steps), final_(final_status), msg_(error_msg) {}

    drone_mapper::types::DroneStepResult step() override {
        if (remain_ > 0) { --remain_; return {drone_mapper::types::DroneStepStatus::Continue, ""}; }
        return {final_, msg_};
    }
    drone_mapper::types::DroneState state() const override { return {}; }
private:
    int remain_;
    drone_mapper::types::DroneStepStatus final_;
    std::string msg_;
};

// Helpers to build configs without repeating boilerplate.
static drone_mapper::types::MissionConfigData makeMission3(std::size_t max_steps) {
    drone_mapper::types::MissionConfigData m{};
    m.max_steps = max_steps;
    m.gps_resolution = 10.0 * drone_mapper::cm;
    return m;
}

static drone_mapper::types::MapConfig makeValidMapConfig() {
    drone_mapper::types::MapConfig cfg{};
    cfg.boundaries.max_x      = 10.0 * drone_mapper::x_extent[drone_mapper::cm];
    cfg.boundaries.max_y      = 10.0 * drone_mapper::y_extent[drone_mapper::cm];
    cfg.boundaries.max_height = 10.0 * drone_mapper::z_extent[drone_mapper::cm];
    cfg.resolution = 10.0 * drone_mapper::cm;
    return cfg;
}

} // namespace

/*
 * What it does: checks the max_steps stopping condition.
 * Setup: uses a drone control mock that keeps returning running.
 * Checks: MissionControl stops at the step limit and saves the output map.
 */
TEST(MissionControl, StopsAtMaxStepsAndSavesOutputMap) {
    types::MissionConfigData mission{};
    mission.max_steps = 2;
    mission.gps_resolution = 10.0 * cm;
    types::DroneConfigData drone{};
    types::MapConfig cfg{};
    cfg.boundaries.max_x      = 10.0 * x_extent[cm];
    cfg.boundaries.max_y      = 10.0 * y_extent[cm];
    cfg.boundaries.max_height = 10.0 * z_extent[cm];
    cfg.resolution = 10.0 * cm;
    DummyMap hidden(cfg);
    DummyMap output(cfg);
    DummyDroneControl control(types::DroneStepStatus::Continue);
    EXPECT_CALL(output, save(testing::_)).Times(1);

    MissionControlImpl mission_control(mission, drone, hidden, output, control, "out.npy");
    const auto result = mission_control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 2U);
}

/*
 * What it does: checks validation of invalid mission boundaries.
 * Setup: passes bounds where min and max values are inconsistent.
 * Checks: MissionControl returns an error before running drone steps.
 */
TEST(MissionControl, InvalidBoundariesReturnErrorImmediately) {
    types::MissionConfigData mission{};
    mission.max_steps = 2;
    types::DroneConfigData drone{};
    types::MapConfig cfg{};
    cfg.boundaries.min_x = 5.0 * x_extent[cm];
    cfg.boundaries.max_x = 5.0 * x_extent[cm];
    cfg.resolution = 10.0 * cm;
    DummyMap hidden(cfg);
    DummyMap output(cfg);
    DummyDroneControl control(types::DroneStepStatus::Continue);
    EXPECT_CALL(output, save(testing::_)).Times(0);

    MissionControlImpl mission_control(mission, drone, hidden, output, control, "out.npy");
    const auto result = mission_control.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors.front().code, "MISSION_BOUNDARY_INVALID");
}

/*
 * What it does: checks the normal completion path.
 * Setup: makes the drone control return completed.
 * Checks: MissionControl reports completed and writes the map output.
 */
TEST(MissionControl, CompletesWhenDroneCompletes) {
    auto cfg = makeValidMapConfig();
    DummyMap hidden(cfg), output(cfg);
    CountingDroneControl control(2, drone_mapper::types::DroneStepStatus::Completed);
    EXPECT_CALL(output, save(testing::_)).Times(1);

    drone_mapper::MissionControlImpl mc(makeMission3(100), {}, hidden, output, control, "out.npy");
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, drone_mapper::types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 3U);
}

/*
 * What it does: checks handling of a drone-step error.
 * Setup: makes the drone control return an error during the mission loop.
 * Checks: MissionControl stops the run and records the error status.
 */
TEST(MissionControl, StepErrorIsRecordedAndStopsRun) {
    auto cfg = makeValidMapConfig();
    DummyMap hidden(cfg), output(cfg);
    CountingDroneControl control(1, drone_mapper::types::DroneStepStatus::Error, "DRONE_STEP_ERROR");
    EXPECT_CALL(output, save(testing::_)).Times(1);

    drone_mapper::MissionControlImpl mc(makeMission3(100), {}, hidden, output, control, "out.npy");
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, drone_mapper::types::MissionRunStatus::Error);
    ASSERT_FALSE(result.errors.empty());
}

/*
 * What it does: checks the off-by-one behavior of the step loop.
 * Setup: runs until the configured max_steps value is reached.
 * Checks: the reported number of steps matches the configured bound.
 */
TEST(MissionControl, ExactStepCountMatchesBound) {
    auto cfg = makeValidMapConfig();
    DummyMap hidden(cfg), output(cfg);
    CountingDroneControl control(1000, drone_mapper::types::DroneStepStatus::Continue);
    EXPECT_CALL(output, save(testing::_)).Times(1);

    drone_mapper::MissionControlImpl mc(makeMission3(5), {}, hidden, output, control, "out.npy");
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, drone_mapper::types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 5U);
}

/*
 * What it does: checks that map output is saved after a failed mission too.
 * Setup: forces an error during the run after output map creation.
 * Checks: the save path is still called before returning the result.
 */
TEST(MissionControl, SaveIsCalledOnCompletedEvenAfterError) {
    auto cfg = makeValidMapConfig();
    DummyMap hidden(cfg), output(cfg);
    CountingDroneControl control(0, drone_mapper::types::DroneStepStatus::Error, "err");
    EXPECT_CALL(output, save(testing::_)).Times(1);

    drone_mapper::MissionControlImpl mc(makeMission3(10), {}, hidden, output, control, "out.npy");
    std::ignore = mc.runMission();
}

} // namespace drone_mapper
