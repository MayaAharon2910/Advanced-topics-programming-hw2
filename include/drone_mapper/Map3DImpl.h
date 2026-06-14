#pragma once

#include <TinyNPY.h>

#include <drone_mapper/IMutableMap3D.h>

#include <cstddef>
#include <filesystem>
#include <memory>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    // Changed: added offset-aware construction for hidden maps loaded from NPY files.
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config);
    Map3DImpl(size_t width,
              size_t height,
              size_t depth,
              const types::MapConfig map_config,
              types::VoxelOccupancy fill_value = types::VoxelOccupancy::Unmapped);

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    // Changed: exposes boundaries, offset, and resolution as one map-owned configuration.
    [[nodiscard]] types::MapConfig getMapConfig() const override;

    //Mutable map methods
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

private:
    // Changed: shared ownership supports the new pointer-based storage member.
    std::shared_ptr<NpyArray> map_;
    // Changed: replaces standalone resolution_ so all map geometry stays together.
    types::MapConfig config_;
    // Internal contiguous storage (width * height * depth)
    std::vector<int8_t> data_;
    size_t width_ = 0;
    size_t height_ = 0;
    size_t depth_ = 0;
};

} // namespace drone_mapper
