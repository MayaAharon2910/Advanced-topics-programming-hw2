#include <drone_mapper/Map3DImpl.h>

#include <fstream>
#include <stdexcept>
#include <typeinfo>
#include <utility>
#include <vector>
#include <cmath>
#include <cstdint>

#include <algorithm>

namespace drone_mapper {
namespace {

[[nodiscard]] int8_t toRaw(types::VoxelOccupancy value) {
    switch (value) {
        case types::VoxelOccupancy::Occupied: return 1;
        case types::VoxelOccupancy::Empty: return 0;
        case types::VoxelOccupancy::OutOfBounds: return -2;
        case types::VoxelOccupancy::PotentiallyOccupied: return -3;
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
    // Map3DImpl translates world coordinates to array indices as:
    //   index = floor((world + offset) / resolution)
    // Therefore index 0 corresponds to world coordinate -offset, not +offset.
    config.boundaries.min_x = -config.offset.x;
    config.boundaries.min_y = -config.offset.y;
    config.boundaries.min_height = -config.offset.z;

    config.boundaries.max_x = -config.offset.x + static_cast<double>(width) * resolution_cm * x_extent[cm];
    config.boundaries.max_y = -config.offset.y + static_cast<double>(height) * resolution_cm * y_extent[cm];
    config.boundaries.max_height = -config.offset.z + static_cast<double>(depth) * resolution_cm * z_extent[cm];
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
    width_ = height_ = depth_ = 0;
    if (!map_->IsEmpty()) {
        const auto& shape = map_->Shape();
        if (shape.size() == 3) {
            width_ = shape[0];
            height_ = shape[1];
            depth_ = shape[2];
            data_.resize(width_ * height_ * depth_, static_cast<int8_t>(-1));
            const size_t elem_bytes = map_->SizeValueBytes();
            // Map files may use semantic values > 1 (e.g. 2=ground, 18=wall,
            // 45=roof in the staff benchmark map) to distinguish material
            // types. Our VoxelOccupancy model only has Occupied=1; any raw
            // value > 1 must be clamped to 1 so collision detection and
            // MapsComparison treat all solid material uniformly as Occupied.
            if (elem_bytes == 1) {
                const auto* src = map_->Data<uint8_t>();
                for (size_t i = 0; i < data_.size(); ++i) {
                    const uint8_t v = src[i];
                    data_[i] = (v > 1) ? int8_t{1} : static_cast<int8_t>(v);
                }
            } else if (elem_bytes == 2) {
                const auto* src = map_->Data<int16_t>();
                for (size_t i = 0; i < data_.size(); ++i) {
                    const int16_t v = src[i];
                    data_[i] = (v > 1) ? int8_t{1} : static_cast<int8_t>(v);
                }
            } else {
                const auto* src = map_->Data<uint8_t>();
                const size_t values = map_->NumValue();
                for (size_t i = 0; i < std::min(values, data_.size()); ++i) {
                    const uint8_t v = src[i];
                    data_[i] = (v > 1) ? int8_t{1} : static_cast<int8_t>(v);
                }
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
    const double rx = (pos.x + config_.offset.x).force_numerical_value_in(cm);
    const double ry = (pos.y + config_.offset.y).force_numerical_value_in(cm);
    const double rz = (pos.z + config_.offset.z).force_numerical_value_in(cm);

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
        case 0: return types::VoxelOccupancy::Empty;
        case -2: return types::VoxelOccupancy::OutOfBounds;
        case -3: return types::VoxelOccupancy::PotentiallyOccupied;
        case -1: return types::VoxelOccupancy::Unmapped;
        // Any positive value is occupied: staff maps use 1 for objects and
        // e.g. 3 for terrain/walls (scenario_house.npy).
        default: return raw > 0 ? types::VoxelOccupancy::Occupied
                                : types::VoxelOccupancy::Unmapped;
    }
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    const auto& b = config_.boundaries;
    const double x = pos.x.force_numerical_value_in(cm);
    const double y = pos.y.force_numerical_value_in(cm);
    const double z = pos.z.force_numerical_value_in(cm);
    return x >= b.min_x.force_numerical_value_in(cm) &&
           x < b.max_x.force_numerical_value_in(cm) &&
           y >= b.min_y.force_numerical_value_in(cm) &&
           y < b.max_y.force_numerical_value_in(cm) &&
           z >= b.min_height.force_numerical_value_in(cm) &&
           z < b.max_height.force_numerical_value_in(cm);
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    const double rx = (pos.x + config_.offset.x).force_numerical_value_in(cm);
    const double ry = (pos.y + config_.offset.y).force_numerical_value_in(cm);
    const double rz = (pos.z + config_.offset.z).force_numerical_value_in(cm);
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
    if (width_ == 0 || height_ == 0 || depth_ == 0) {
        throw std::runtime_error("Map3DImpl::save: empty map cannot be saved.");
    }

    const std::filesystem::path parent = path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }

    NpyArray::shape_t shape{width_, height_, depth_};
    NpyArray output_array(shape, sizeof(int8_t), NpyArray::GetTypeChar(typeid(int8_t)));
    output_array.Allocate();
    std::copy(data_.begin(), data_.end(), output_array.Data<int8_t>());

    const char* err = output_array.SaveNPY(path.string());
    if (err != nullptr) {
        throw std::runtime_error(std::string("Failed to save NPY: ") + err);
    }
}

} // namespace drone_mapper
