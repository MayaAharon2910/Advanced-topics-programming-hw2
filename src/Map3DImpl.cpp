#include <drone_mapper/Map3DImpl.h>

#include <fstream>
#include <stdexcept>
#include <utility>
#include <vector>
#include <cstring>
#include <cmath>

#include <algorithm>

namespace drone_mapper {

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
    // Initialize internal storage from the provided NpyArray when possible.
    // Prefer a contiguous 1D vector backing for performance.
    map_width_ = map_height_ = map_depth_ = 0;
    if (!map_->IsEmpty()) {
        const auto& shape = map_->Shape();
        if (shape.size() == 3) {
            map_width_ = shape[0];
            map_height_ = shape[1];
            map_depth_ = shape[2];
            data_.resize(map_width_ * map_height_ * map_depth_, static_cast<int8_t>(-1));
            // Copy raw bytes taking element size into account
            const size_t elem_bytes = map_->SizeValueBytes();
            if (elem_bytes == 1) {
                const auto* src = map_->Data<uint8_t>();
                for (size_t i = 0; i < data_.size(); ++i) data_[i] = static_cast<int8_t>(src[i]);
            } else if (elem_bytes == 2) {
                const auto* src = map_->Data<int16_t>();
                for (size_t i = 0; i < data_.size(); ++i) data_[i] = static_cast<int8_t>(src[i]);
            } else {
                // Fallback: read as bytes
                const auto* src = map_->Data<uint8_t>();
                const size_t bytes = map_->SizeBytes();
                const size_t values = map_->NumValue();
                for (size_t i = 0; i < std::min(values, data_.size()); ++i) data_[i] = static_cast<int8_t>(src[i]);
            }
        }
    }
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    // Convert world position to voxel indices using offset and resolution
    const double rx = (pos.x - config_.offset.x).force_numerical_value_in(cm);
    const double ry = (pos.y - config_.offset.y).force_numerical_value_in(cm);
    const double rz = (pos.z - config_.offset.z).force_numerical_value_in(cm);

    if (config_.resolution.force_numerical_value_in(cm) <= 0.0) {
        return types::VoxelOccupancy::Unmapped;
    }

    const int ix = static_cast<int>(std::floor(rx / config_.resolution.force_numerical_value_in(cm)));
    const int iy = static_cast<int>(std::floor(ry / config_.resolution.force_numerical_value_in(cm)));
    const int iz = static_cast<int>(std::floor(rz / config_.resolution.force_numerical_value_in(cm)));

    if (ix < 0 || iy < 0 || iz < 0) return types::VoxelOccupancy::OutOfBounds;
    if (static_cast<size_t>(ix) >= map_width_ || static_cast<size_t>(iy) >= map_height_ || static_cast<size_t>(iz) >= map_depth_) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    const size_t idx = static_cast<size_t>(ix) * map_height_ * map_depth_ + static_cast<size_t>(iy) * map_depth_ + static_cast<size_t>(iz);
    const int8_t raw = data_.empty() ? static_cast<int8_t>(-1) : data_[idx];
    switch (raw) {
        case 1: return types::VoxelOccupancy::Occupied;
        case 0: return types::VoxelOccupancy::Empty;
        case -2: return types::VoxelOccupancy::OutOfBounds;
        case -1: default: return types::VoxelOccupancy::Unmapped;
    }
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    const double rx = (pos.x - config_.offset.x).force_numerical_value_in(cm);
    const double ry = (pos.y - config_.offset.y).force_numerical_value_in(cm);
    const double rz = (pos.z - config_.offset.z).force_numerical_value_in(cm);
    const int ix = static_cast<int>(std::floor(rx / config_.resolution.force_numerical_value_in(cm)));
    const int iy = static_cast<int>(std::floor(ry / config_.resolution.force_numerical_value_in(cm)));
    const int iz = static_cast<int>(std::floor(rz / config_.resolution.force_numerical_value_in(cm)));
    if (ix < 0 || iy < 0 || iz < 0) return;
    if (static_cast<size_t>(ix) >= map_width_ || static_cast<size_t>(iy) >= map_height_ || static_cast<size_t>(iz) >= map_depth_) return;
    const size_t idx = static_cast<size_t>(ix) * map_height_ * map_depth_ + static_cast<size_t>(iy) * map_depth_ + static_cast<size_t>(iz);
    int8_t raw = -1;
    switch (value) {
        case types::VoxelOccupancy::Occupied: raw = 1; break;
        case types::VoxelOccupancy::Empty: raw = 0; break;
        case types::VoxelOccupancy::OutOfBounds: raw = -2; break;
        case types::VoxelOccupancy::Unmapped: raw = -1; break;
        default: raw = -1; break;
    }
    if (data_.empty()) data_.resize(map_width_ * map_height_ * map_depth_, static_cast<int8_t>(-1));
    data_[idx] = raw;
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    // Save using TinyNPY helper: write our internal contiguous vector as int8 values
    if (map_width_ == 0 || map_height_ == 0 || map_depth_ == 0) {
        throw std::runtime_error("Map3DImpl::save: empty map cannot be saved.");
    }
    std::vector<int8_t> out = data_;
    NpyArray::shape_t shape{map_width_, map_height_, map_depth_};
    const char* err = NpyArray::SaveNPY(path.string(), out, shape);
    if (err != nullptr) {
        throw std::runtime_error(std::string("Failed to save NPY: ") + err);
    }
}

} // namespace drone_mapper
