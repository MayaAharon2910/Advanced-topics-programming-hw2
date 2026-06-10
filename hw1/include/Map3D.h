#pragma once

#include <vector>
#include <cstddef>

class Map3D {
public:
    enum VoxelState {
        FREE = 0,
        OCCUPIED = 1,
        UNKNOWN = -1,
        OUT_OF_BOUNDS = -2
    };

    // Creates a map initialized to UNKNOWN with full-map mission bounds.
    Map3D(size_t width, size_t height, size_t depth);

    // Returns the voxel value at unsigned coordinates or OUT_OF_BOUNDS when invalid.
    int at(size_t x, size_t y, size_t z) const;
    // Returns the voxel value at signed coordinates or OUT_OF_BOUNDS when invalid.
    int at(int x, int y, int z) const;
    // Sets a voxel value at unsigned coordinates when inside the map and mission bounds.
    void set(size_t x, size_t y, size_t z, int value);
    // Sets a voxel value at signed coordinates when inside the map and mission bounds.
    void set(int x, int y, int z, int value);
    // Defines the inclusive mission bounds used by reads, writes, and fill operations.
    void setMissionBounds(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z);
    // Marks every voxel outside the mission bounds as OUT_OF_BOUNDS.
    void fillOutOfBoundsVoxels();
    // Returns true when a voxel coordinate is inside the configured mission bounds.
    bool isWithinMissionBounds(int x, int y, int z) const;

    // Returns the map width in voxels.
    size_t width() const { return width_; }
    // Returns the map height in voxels.
    size_t height() const { return height_; }
    // Returns the map depth in voxels.
    size_t depth() const { return depth_; }

private:
    // Converts 3-D coordinates into the backing vector index.
    size_t indexOf(size_t x, size_t y, size_t z) const;

    size_t width_, height_, depth_;
    std::vector<int> data_;
    int min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
};
