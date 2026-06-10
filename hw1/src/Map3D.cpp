#include "Map3D.h"
#include <algorithm>

Map3D::Map3D(size_t width, size_t height, size_t depth)
    : width_(width), height_(height), depth_(depth),
      data_(width * height * depth, UNKNOWN),
      min_x_(0), max_x_(static_cast<int>(width) - 1),
      min_y_(0), max_y_(static_cast<int>(height) - 1),
      min_z_(0), max_z_(static_cast<int>(depth) - 1) {}

size_t Map3D::indexOf(size_t x, size_t y, size_t z) const {
    // x is the slowest-changing coordinate to match the map file traversal order.
    return x * (height_ * depth_) + y * depth_ + z;
}

bool Map3D::isWithinMissionBounds(int x, int y, int z) const {
    return x >= min_x_ && x <= max_x_ &&
           y >= min_y_ && y <= max_y_ &&
           z >= min_z_ && z <= max_z_;
}

void Map3D::setMissionBounds(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {
    min_x_ = min_x;
    // Clamp upper bounds to the allocated map; lower bounds are handled by at()/set().
    max_x_ = std::min(max_x, static_cast<int>(width_) - 1);
    min_y_ = min_y;
    max_y_ = std::min(max_y, static_cast<int>(height_) - 1);
    min_z_ = min_z;
    max_z_ = std::min(max_z, static_cast<int>(depth_) - 1);
}

void Map3D::fillOutOfBoundsVoxels() {
    for (size_t x = 0; x < width_; ++x) {
        for (size_t y = 0; y < height_; ++y) {
            for (size_t z = 0; z < depth_; ++z) {
                if (!isWithinMissionBounds(static_cast<int>(x),
                                           static_cast<int>(y),
                                           static_cast<int>(z))) {
                    data_[indexOf(x, y, z)] = OUT_OF_BOUNDS;
                }
            }
        }
    }
}

int Map3D::at(size_t x, size_t y, size_t z) const {
    if (x >= width_ || y >= height_ || z >= depth_) {
        return OUT_OF_BOUNDS;
    }
    if (!isWithinMissionBounds(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z))) {
        return OUT_OF_BOUNDS;
    }
    return data_[indexOf(x, y, z)];
}

int Map3D::at(int x, int y, int z) const {
    if (x < 0 || y < 0 || z < 0) {
        return OUT_OF_BOUNDS;
    }
    return at(static_cast<size_t>(x), static_cast<size_t>(y), static_cast<size_t>(z));
}

void Map3D::set(size_t x, size_t y, size_t z, int value) {
    if (x >= width_ || y >= height_ || z >= depth_) {
        return;
    }
    if (!isWithinMissionBounds(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z))) {
        return;
    }
    data_[indexOf(x, y, z)] = value;
}

void Map3D::set(int x, int y, int z, int value) {
    if (x < 0 || y < 0 || z < 0) {
        return;
    }
    set(static_cast<size_t>(x), static_cast<size_t>(y), static_cast<size_t>(z), value);
}
