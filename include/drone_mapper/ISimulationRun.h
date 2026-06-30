#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

// **Do not change this interface.**
class ISimulationRun {
public:
    virtual ~ISimulationRun() = default;

    // Run one concrete scenario and return score/output metadata.
    [[nodiscard]] virtual types::SimulationResult run() = 0;
};

} // namespace drone_mapper
