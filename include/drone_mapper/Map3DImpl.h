#pragma once

#include <TinyNPY.h>

#include <drone_mapper/IMutableMap3D.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <vector>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    // Constructs a map from an NPY array with an explicit geometry configuration.
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config);
    Map3DImpl(size_t width,
              size_t height,
              size_t depth,
              const types::MapConfig map_config,
              types::VoxelOccupancy fill_value = types::VoxelOccupancy::Unmapped);

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    // Returns the map geometry (boundaries, offset, resolution).
    [[nodiscard]] types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;

    //Mutable map methods
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

private:
    // Shared ownership of the underlying NPY array.
    std::shared_ptr<NpyArray> map_;
    // All map geometry (boundaries, offset, resolution) in one struct.
    types::MapConfig config_;
    // Internal contiguous storage (width * height * depth)
    std::vector<int8_t> data_;
    size_t width_ = 0;
    size_t height_ = 0;
    size_t depth_ = 0;
};

} // namespace drone_mapper
