#pragma once

#include <drone_mapper/Units.h>

#include <cstddef>
#include <filesystem>
#include <vector>

namespace drone_mapper::types {

struct LidarConfigData {
    PhysicalLength z_min{};
    PhysicalLength z_max{};
    PhysicalLength d{};
    std::size_t fov_circles = 0;
    std::filesystem::path source_file{};
};

struct LidarHit {
    // Misses use max double centimeters;
    PhysicalLength distance{};
    Orientation angle{};
};

using LidarScanResult = std::vector<LidarHit>;

} // namespace drone_mapper::types
