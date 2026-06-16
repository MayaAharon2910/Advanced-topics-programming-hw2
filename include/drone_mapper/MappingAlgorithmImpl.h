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
    using IMappingAlgorithm::IMappingAlgorithm;

    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

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
    [[nodiscard]] types::MappingStepCommand nextPlanningStep();
    [[nodiscard]] types::MappingStepCommand nextMovingStep();
};

} // namespace drone_mapper
