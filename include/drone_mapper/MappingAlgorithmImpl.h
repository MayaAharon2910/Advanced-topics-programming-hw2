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
    enum class BfsGoalMode      { Frontier6, Frontier26 };

    struct GridKey {
        int x = 0, y = 0, z = 0;
        [[nodiscard]] bool operator<(const GridKey& o) const noexcept {
            return std::tie(x,y,z) < std::tie(o.x,o.y,o.z);
        }
        [[nodiscard]] bool operator==(const GridKey& o) const noexcept {
            return x==o.x && y==o.y && z==o.z;
        }
    };

    // State
    std::map<GridKey, types::VoxelOccupancy>     known_voxels_{};
    std::set<std::tuple<int,int,int>>            visited_positions_{};
    std::deque<types::MovementCommand>           pending_commands_{};
    std::vector<GridKey>                         current_path_{};
    Position3D                                   current_position_{};
    Orientation                                  orientation_{};
    ExplorationState                             state_ = ExplorationState::Planning;

    // World-coordinate helpers: keep unit conversions in one place.
    [[nodiscard]] PhysicalLength  cellSize()          const;
    [[nodiscard]] PhysicalLength  maxTraceDistance()  const;
    [[nodiscard]] GridKey         toGrid(const Position3D&) const;
    [[nodiscard]] Position3D      toPosition(const GridKey&) const;

    // Voxel map helpers
    [[nodiscard]] types::VoxelOccupancy at(const GridKey&) const;
    void setKnown(const GridKey&, types::VoxelOccupancy);
    void markCurrentVisited();

    // Scan ingestion helpers: convert the latest scan into internal map knowledge.
    void processHit(const types::DroneState&, const types::LidarHit&);
    void markFreeRay(const Position3D& origin,
                     double dx, double dy, double dz,
                     PhysicalLength distance);
    void ingestScan(const types::DroneState&, const types::LidarScanResult&);

    // Bounds / navigability
    [[nodiscard]] bool isInsideMissionBounds(const GridKey&) const;
    [[nodiscard]] bool isNavigable(const GridKey&) const;

    // BFS and frontier helpers: find the next unexplored reachable area.
    [[nodiscard]] bool hasUnknownOrthogonalNeighbor(const GridKey&) const;
    [[nodiscard]] bool hasUnknownMooreNeighbor(const GridKey&)      const;
    [[nodiscard]] bool isBfsGoal(const GridKey&, BfsGoalMode)       const;
    [[nodiscard]] std::vector<GridKey> bfsToGoal(BfsGoalMode)       const;
    [[nodiscard]] std::vector<GridKey> reconstructPath(const GridKey& goal,
                                                        const GridKey& start,
                                                        const std::map<GridKey,GridKey>& parent) const;

    // Movement planning
    bool enqueueSweepMove();
    void enqueueCommandsForStep(const GridKey& target);

    // State machine
    [[nodiscard]] types::MappingStepCommand nextMovingStep();
    [[nodiscard]] types::MappingStepCommand nextPlanningStep();
    [[nodiscard]] types::MappingStepCommand finishedCommand();
};

} // namespace drone_mapper
