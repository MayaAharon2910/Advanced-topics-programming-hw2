#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <deque>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    explicit MappingAlgorithmImpl(types::MissionConfigData mission);
    MappingAlgorithmImpl(types::MissionConfigData mission, types::DroneConfigData drone);

    [[nodiscard]] types::MovementCommand nextMove(const types::DroneState& state,
                                                  const types::LidarScanResult& latest_scan) override;
    void applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) override;

private:
    enum class ExplorationState { Planning, Moving, Finished };
    enum class BfsGoalMode { Frontier6, Frontier26 };

    struct GridKey {
        int x = 0;
        int y = 0;
        int z = 0;
        [[nodiscard]] bool operator<(const GridKey& other) const noexcept {
            return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
        }
        [[nodiscard]] bool operator==(const GridKey& other) const noexcept {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    types::MissionConfigData mission_;
    types::DroneConfigData drone_;
    bool has_drone_config_ = false;

    std::map<GridKey, types::VoxelOccupancy> known_voxels_{};
    std::set<std::tuple<int, int, int>> visited_positions_{};
    std::deque<types::MovementCommand> pending_commands_{};
    std::vector<GridKey> current_path_{};
    Position3D current_position_{};
    Orientation orientation_{};
    ExplorationState state_ = ExplorationState::Planning;

    [[nodiscard]] double cellSizeCm() const;
    [[nodiscard]] double maxTraceDistanceCm() const;
    [[nodiscard]] GridKey toGrid(const Position3D& position) const;
    [[nodiscard]] Position3D toPosition(const GridKey& key) const;
    [[nodiscard]] types::VoxelOccupancy at(const GridKey& key) const;
    void setKnown(const GridKey& key, types::VoxelOccupancy value);
    void ingestScan(const types::DroneState& state, const types::LidarScanResult& scan);
    void markCurrentVisited();

    [[nodiscard]] bool isInsideMissionBounds(const GridKey& key) const;
    [[nodiscard]] bool isNavigable(const GridKey& key) const;
    [[nodiscard]] bool isBfsGoal(const GridKey& key, BfsGoalMode mode) const;
    [[nodiscard]] bool hasUnknownOrthogonalNeighbor(const GridKey& key) const;
    [[nodiscard]] bool hasUnknownMooreNeighbor(const GridKey& key) const;
    [[nodiscard]] std::vector<GridKey> bfsToGoal(BfsGoalMode mode) const;

    bool enqueueSweepMove();
    void enqueueCommandsForStep(const GridKey& target);
    [[nodiscard]] types::MovementCommand nextPlanningCommand();
    [[nodiscard]] types::MovementCommand nextMovingCommand();
};

} // namespace drone_mapper
