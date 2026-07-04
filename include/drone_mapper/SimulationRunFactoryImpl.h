#pragma once

#include <drone_mapper/ISimulationRunFactory.h>

#include <cstddef>

namespace drone_mapper {

class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const types::MissionConfigData& mission,
           const types::DroneConfigData& drone,
           const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override;

private:
    std::size_t next_run_index_ = 0;
};

using SimulationRunFactory = SimulationRunFactoryImpl;

} // namespace drone_mapper
