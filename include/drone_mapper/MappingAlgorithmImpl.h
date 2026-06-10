#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    explicit MappingAlgorithmImpl(types::MissionConfigData mission);

    [[nodiscard]] types::MovementCommand nextMove(const types::DroneState& state,
                                                  const types::LidarScanResult& latest_scan) override;
    void applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) override;

private:
    types::MissionConfigData mission_;
    // Internal exploration state adapted from hw1 Drone.cpp
    std::unique_ptr<Map3DImpl> internal_map_ = nullptr;
    types::Position3D current_position_{};
    types::Orientation orientation_{};
    std::vector<types::MovementCommand> pending_commands_{};
    bool scan_batch_completion_pending_ = false;
    std::set<std::tuple<int,int,int>> visited_positions_{};
    std::vector<types::Position3D> current_path_{};
    enum class ExplorationState { Scanning, Planning, Moving, Finished };
    ExplorationState state_ = ExplorationState::Scanning;
};

} // namespace drone_mapper
