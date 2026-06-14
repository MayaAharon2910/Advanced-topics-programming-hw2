#include <drone_mapper/Map3DImpl.h>

#include <fstream>
#include <stdexcept>
#include <utility>
#include <vector>
#include <cmath>

#include <algorithm>

namespace drone_mapper {
namespace {

[[nodiscard]] int8_t toRaw(types::VoxelOccupancy value) {
    switch (value) {
        case types::VoxelOccupancy::Occupied: return 1;
        case types::VoxelOccupancy::Empty: return 0;
        case types::VoxelOccupancy::OutOfBounds: return -2;
        case types::VoxelOccupancy::Unmapped:
        default: return -1;
    }
}

[[nodiscard]] bool hasUnsetBounds(const types::MappingBounds& bounds) {
    return bounds.min_x.numerical_value_in(cm) == 0.0 &&
           bounds.max_x.numerical_value_in(cm) == 0.0 &&
           bounds.min_y.numerical_value_in(cm) == 0.0 &&
           bounds.max_y.numerical_value_in(cm) == 0.0 &&
           bounds.min_height.numerical_value_in(cm) == 0.0 &&
           bounds.max_height.numerical_value_in(cm) == 0.0;
}

void deriveBounds(types::MapConfig& config, size_t width, size_t height, size_t depth) {
    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (resolution_cm <= 0.0 || !hasUnsetBounds(config.boundaries)) {
        return;
    }
    config.boundaries.min_x = config.offset.x;
    config.boundaries.min_y = config.offset.y;
    config.boundaries.min_height = config.offset.z;
    config.boundaries.max_x = config.offset.x + static_cast<double>(width) * resolution_cm * x_extent[cm];
    config.boundaries.max_y = config.offset.y + static_cast<double>(height) * resolution_cm * y_extent[cm];
    config.boundaries.max_height = config.offset.z + static_cast<double>(depth) * resolution_cm * z_extent[cm];
}

} // namespace

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
    width_ = height_ = depth_ = 0;
    if (!map_->IsEmpty()) {
        const auto& shape = map_->Shape();
        if (shape.size() == 3) {
            width_ = shape[0];
            height_ = shape[1];
            depth_ = shape[2];
            data_.resize(width_ * height_ * depth_, static_cast<int8_t>(-1));
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
                const size_t values = map_->NumValue();
                for (size_t i = 0; i < std::min(values, data_.size()); ++i) data_[i] = static_cast<int8_t>(src[i]);
            }
        }
    }
    deriveBounds(config_, width_, height_, depth_);
}

Map3DImpl::Map3DImpl(size_t width,
                     size_t height,
                     size_t depth,
                     const types::MapConfig map_config,
                     types::VoxelOccupancy fill_value)
    : map_(std::make_shared<NpyArray>()),
      config_(map_config),
      data_(width * height * depth, toRaw(fill_value)),
      width_(width),
      height_(height),
      depth_(depth) {
    deriveBounds(config_, width_, height_, depth_);
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
    if (static_cast<size_t>(ix) >= width_ || static_cast<size_t>(iy) >= height_ || static_cast<size_t>(iz) >= depth_) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    const size_t idx = static_cast<size_t>(ix) * height_ * depth_ + static_cast<size_t>(iy) * depth_ + static_cast<size_t>(iz);
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
    if (static_cast<size_t>(ix) >= width_ || static_cast<size_t>(iy) >= height_ || static_cast<size_t>(iz) >= depth_) return;
    const size_t idx = static_cast<size_t>(ix) * height_ * depth_ + static_cast<size_t>(iy) * depth_ + static_cast<size_t>(iz);
    if (data_.empty()) data_.resize(width_ * height_ * depth_, static_cast<int8_t>(-1));
    data_[idx] = toRaw(value);
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    // Save using TinyNPY helper: write our internal contiguous vector as int8 values
    if (width_ == 0 || height_ == 0 || depth_ == 0) {
        throw std::runtime_error("Map3DImpl::save: empty map cannot be saved.");
    }
    NpyArray::shape_t shape{width_, height_, depth_};
    const char* err = NpyArray::SaveNPY(path.string(), data_, shape);
    if (err != nullptr) {
        throw std::runtime_error(std::string("Failed to save NPY: ") + err);
    }
}

} // namespace drone_mapper
