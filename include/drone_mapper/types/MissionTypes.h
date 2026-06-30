#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/types/MapTypes.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace drone_mapper::types {

// Explicit mission boundaries. All-zero bounds mean "unset" and are derived from the map.
struct MissionConfigData {
    std::size_t max_steps = 0;
    PhysicalLength gps_resolution{};
    double output_mapping_resolution_factor = 0;
    MappingBounds mission_bounds{};
    std::filesystem::path source_file{};
};

enum class MissionRunStatus {
    Completed,
    MaxSteps,
    Error,
};

struct ErrorRef {
    std::string code{};
    std::string message{};
};

struct MissionRunResult {
    MissionRunStatus status = MissionRunStatus::Completed;
    std::size_t steps = 0;
    // A mission can collect multiple recoverable errors before it stops.
    std::vector<ErrorRef> errors{};
};

} // namespace drone_mapper::types
