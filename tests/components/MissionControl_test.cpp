// =============================================================================
// MissionControl_test.cpp — Component tests for MissionControlImpl
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

// Output map stub — only save() is a GMock method so we can assert it
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
 * What it does: runs a mission where the drone always says Continue.
 * Setup: max_steps=2, drone returns Continue on every step.
 * Checks: status is MaxSteps, step count is exactly 2, and save() is called
 *         once to persist the partial output map.
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
 * What it does: provides invalid boundaries (min_x == max_x) and checks
 *               that the mission is rejected before a single step runs.
 * Setup: boundaries where min_x equals max_x (zero-width space).
 * Checks: status is Error with code MISSION_BOUNDARY_INVALID, and save()
 *         is never called because no output was produced.
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
 * What it does: runs a mission where the drone finishes after 2 Continue steps.
 * Setup: drone returns Continue twice, then Completed.
 * Checks: status is Completed, step count is 3 (2 Continue + 1 Completed),
 *         and save() is called once.
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
 * What it does: simulates a mid-run drone error and checks the error is recorded.
 * Setup: drone returns Continue once, then Error.
 * Checks: status is Error, the errors list is non-empty, and save() is called
 *         to preserve whatever partial map was built before the failure.
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
 * What it does: verifies that max_steps is honoured exactly, not approximately.
 * Setup: drone always returns Continue; max_steps=5.
 * Checks: the mission stops at exactly 5 steps — not 4, not 6.
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
 * What it does: verifies save() is called even when the first step errors.
 * Setup: drone errors immediately (0 Continue steps before Error).
 * Checks: save() is called exactly once — the partial (empty) output map
 *         must still be written to disk so the score report can reference it.
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
