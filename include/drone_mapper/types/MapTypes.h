#pragma once

#include <drone_mapper/Units.h>

namespace drone_mapper::types {

enum class VoxelOccupancy {
    PotentiallyOccupied = -3,
    OutOfBounds = -2,
    Unmapped = -1,
    Empty = 0,
    Occupied = 1,
};

// Boundaries describe a map volume in world coordinates.
struct MappingBounds {
    XLength min_x{};
    XLength max_x{};
    YLength min_y{};
    YLength max_y{};
    ZLength min_height{};
    ZLength max_height{};
};

struct MappedVoxel {
    Position3D position{};
    VoxelOccupancy value = VoxelOccupancy::Unmapped;
};

// Full geometry description for any 3D map.
// A default MapConfig means unset bounds, zero offset, and zero resolution.
struct MapConfig {
    MappingBounds boundaries{};
    Position3D offset{};
    PhysicalLength resolution{};
};
} // namespace drone_mapper::types
