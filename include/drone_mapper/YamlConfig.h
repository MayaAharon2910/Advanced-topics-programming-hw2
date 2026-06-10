#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>

namespace drone_mapper {

namespace yaml {
    // Parse a simulation composition YAML file into SimulationCompositionData.
    // Throws std::runtime_error on parse failures. Logs recoverable warnings
    // to stderr immediately as required.
    types::SimulationCompositionData parseSimulationComposition(const std::filesystem::path& path);
}

} // namespace drone_mapper
