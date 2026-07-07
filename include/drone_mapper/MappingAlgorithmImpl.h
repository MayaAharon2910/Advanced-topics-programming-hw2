#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <deque>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

namespace drone_mapper {

// Faithful port of the HW1 Drone exploration logic (hw1/src/Drone.cpp) into the
// fixed IMappingAlgorithm API of HW2.
//
// HW1 state machine and order of operations, preserved here:
//   Scanning : at every new position, run a fixed batch of 6 scans
//              (forward / right / back / left at elevation 0, then up +90, down -90).
//              When the batch completes, mark the position visited -> Planning.
//   Planning : 1) continue an existing BFS path if the next step is still
//                 sphere-safe; if it is blocked by UNKNOWN cells, emit a
//                 targeted scan toward one of those cells,
//              2) cheap local sweep to an adjacent unvisited navigable cell,
//              3) BFS to the nearest Frontier6 goal, fallback Frontier26,
//              4) targeted scan at an unknown cell adjacent to the drone,
//              5) Finished.
//   Moving   : replay the queued Rotate/Advance/Elevate chunks; when the queue
//              empties -> Scanning (HW1 always re-scans after arriving).
//
// Scan ingestion is HW1's markScanRay: a DDA voxel traversal that marks the
// crossed cells Empty and the hit cell Occupied, with the two HW1 special
// cases: distance 0 (too close to measure) marks the first cell outside the
// current one as Occupied, and a miss marks free space up to max range without
// placing a wall at the end and without overwriting a known Occupied cell.
class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    // HW1 had four states; HW2's previous implementation dropped Scanning,
    // which removed the systematic 6-direction coverage at each position.
    enum class ExplorationState { Scanning, Planning, Moving, Finished };
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
    std::deque<Orientation>                      pending_scans_{};      // HW1 pending scan batch / targeted scans
    std::set<std::tuple<int,int,int,int,int,int>> targeted_scans_done_{}; // (from, target) pairs already scanned
    std::vector<GridKey>                         current_path_{};
    Position3D                                   current_position_{};
    Orientation                                  orientation_{};
    ExplorationState                             state_ = ExplorationState::Scanning; // HW1 starts by scanning

    // World-coordinate helpers
    [[nodiscard]] PhysicalLength  cellSize()          const;
    [[nodiscard]] PhysicalLength  lidarMaxDistance()  const;
    [[nodiscard]] GridKey         toGrid(const Position3D&) const;
    [[nodiscard]] Position3D      toPosition(const GridKey&) const; // voxel center

    // Voxel map helpers
    [[nodiscard]] types::VoxelOccupancy at(const GridKey&) const;
    void setKnown(const GridKey&, types::VoxelOccupancy);
    void markCurrentVisited();
    void preserveVisitedPositionsAsFree(); // HW1: cells we stood in stay Empty

    // Scan ingestion (HW1 markScanRay, DDA over grid cells)
    void markScanRay(const Position3D& origin,
                     double dx, double dy, double dz,
                     double reported_distance_cm);
    void ingestScan(const types::DroneState&, const types::LidarScanResult&);

    // Bounds / navigability
    [[nodiscard]] bool isInsideMissionBounds(const GridKey&) const;
    [[nodiscard]] bool isNavigable(const GridKey&) const;      // HW1 isNavigableVoxel
    [[nodiscard]] bool sphereAreaIsFree(const GridKey&) const; // HW1 sphereAreaIsFree

    // Frontier helpers (HW1 names preserved)
    [[nodiscard]] bool hasUnknownOrthogonalNeighbor(const GridKey&) const;
    [[nodiscard]] bool hasLineOfSightToUnknown(const GridKey& from, int dx, int dy, int dz) const;
    [[nodiscard]] bool hasUnknownMooreNeighbor(const GridKey&) const;
    [[nodiscard]] bool isBfsGoal(const GridKey&, BfsGoalMode) const;

    // Targeted scans (HW1 makeTargetedScanCommand / enqueueTargetedScan*)
    [[nodiscard]] std::optional<Orientation> targetedScanOrientation(const GridKey& target) const;
    bool enqueueTargetedScanForTarget(const GridKey& target);
    bool enqueueTargetedScanAroundCurrentPosition();

    // BFS
    [[nodiscard]] std::vector<GridKey> bfsToGoal(BfsGoalMode) const;
    [[nodiscard]] std::vector<GridKey> reconstructPath(const GridKey& goal,
                                                        const GridKey& start,
                                                        const std::map<GridKey,GridKey>& parent) const;

    // Movement planning (HW1 buildCommandsForStep / enqueueSweepMove)
    bool enqueueSweepMove();
    void enqueueCommandsForStep(const GridKey& target);
    void enqueueScanBatch(); // HW1 createScanCommands: 4 horizontal + up + down

    // State machine
    [[nodiscard]] types::MappingStepCommand nextScanningStep();
    [[nodiscard]] types::MappingStepCommand nextMovingStep();
    [[nodiscard]] types::MappingStepCommand nextPlanningStep();
    [[nodiscard]] types::MappingStepCommand finishedCommand();
};

} // namespace drone_mapper
