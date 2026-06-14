#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/MissionControlImpl.h>

namespace drone_mapper {
namespace {

class DummyMap final : public IMutableMap3D {
public:
    explicit DummyMap(types::MapConfig cfg) : cfg_(cfg) {}
    types::VoxelOccupancy atVoxel(const Position3D&) const override { return types::VoxelOccupancy::Empty; }
    types::MapConfig getMapConfig() const override { return cfg_; }
    void set(const Position3D&, types::VoxelOccupancy) override {}
    MOCK_METHOD(void, save, (const std::filesystem::path& output_path), (const, override));
private:
    types::MapConfig cfg_{};
};

class DummyDroneControl final : public IDroneControl {
public:
    explicit DummyDroneControl(types::DroneStepStatus status) : status_(status) {}
    types::DroneStepResult step() override { return types::DroneStepResult{status_, "step"}; }
    types::DroneState state() const override { return {}; }
private:
    types::DroneStepStatus status_;
};

} // namespace

TEST(MissionControl, StopsAtMaxStepsAndSavesOutputMap) {
    types::MissionConfigData mission{};
    mission.max_steps = 2;
    mission.gps_resolution = 10.0 * cm;
    types::DroneConfigData drone{};
    types::MapConfig cfg{};
    cfg.boundaries.max_x = 10.0 * x_extent[cm];
    cfg.boundaries.max_y = 10.0 * y_extent[cm];
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

} // namespace drone_mapper
