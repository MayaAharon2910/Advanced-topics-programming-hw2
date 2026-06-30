#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/types/MapTypes.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace drone_mapper::types {

// Changed: boundaries were removed because map bounds now live on MapConfig/IMap3D.
struct MissionConfigData {
    std::size_t max_steps = 0;
    PhysicalLength gps_resolution{};
    MappingBounds boundaries{};
    MappingBounds mission_bounds{};
    double output_mapping_resolution_factor = 0;
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
    // double score = 0.0; // moved to simulationResult
    // std::filesystem::path output_map_file{}; // moved to simulation Result
    // Changed: a run can report multiple errors instead of a single ErrorRef.
    std::vector<ErrorRef> errors{}; // we may have multiple errors
};

} // namespace drone_mapper::types
