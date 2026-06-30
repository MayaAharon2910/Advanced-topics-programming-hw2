#pragma once

#include <drone_mapper/ISimulation.h>
#include <drone_mapper/ISimulationRunFactory.h>

#include <memory>

namespace drone_mapper {

class SimulationManager final : public ISimulation {
public:
    explicit SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory);

    // Execute the full Cartesian product and return the aggregate report.
    [[nodiscard]] types::SimulationManagerReport run(const types::SimulationCompositionData& composition,
                                              const std::filesystem::path& output_path) override; // output - to save the output map for example

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
};

} // namespace drone_mapper
