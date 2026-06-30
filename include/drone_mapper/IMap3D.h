#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

// Read-only 3D occupancy map interface used by LiDAR implementations.
// **Do not change this interface.**
class IMap3D {
public:
    virtual ~IMap3D() = default;

    // Query a voxel by world-space position.
    [[nodiscard]] virtual types::VoxelOccupancy atVoxel(const Position3D& pos) const = 0;
    // Expose map geometry as a single object.
    [[nodiscard]] virtual types::MapConfig getMapConfig() const = 0;
    [[nodiscard]] virtual bool isInBounds(const Position3D& pos) const = 0; 
};

} // namespace drone_mapper
