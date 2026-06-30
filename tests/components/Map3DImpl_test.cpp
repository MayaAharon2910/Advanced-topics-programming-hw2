// =============================================================================
// Map3DImpl_test.cpp - Component tests for Map3DImpl's .npy loading
//
// Map3DImpl reads raw voxel bytes from an NpyArray into an internal int8_t
// buffer. Staff-provided maps use semantic values > 1 (e.g. 2=ground, 18=wall,
// 45=roof). These tests verify the loader clamps any raw value > 1 to Occupied
// so collision detection and map scoring treat all solid material consistently.
// =============================================================================

#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <filesystem>
#include <fstream>
#include <cstdint>
#include <string>
#include <vector>

namespace drone_mapper {

namespace {

// Write a minimal uint8 NPY file to disk.
// NPY format: magic (6) + version (2) + header_len (2) + header + data.
void writeUint8Npy(const std::filesystem::path& path,
                   const std::vector<uint8_t>& data,
                   std::size_t x, std::size_t y, std::size_t z) {
    // Header describes a uint8, C-contiguous 3-D array.
    const std::string dict =
        "{'descr': '|u1', 'fortran_order': False, 'shape': ("
        + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z)
        + "), }";
    // Header must be padded to a multiple of 64 bytes (v1 spec).
    const std::size_t header_data_len = dict.size() + 1; // +1 for \n
    const std::size_t padding = (64 - (10 + header_data_len) % 64) % 64;
    const std::string header = dict + std::string(padding, ' ') + '\n';
    const uint16_t header_len = static_cast<uint16_t>(header.size());

    std::ofstream f(path, std::ios::binary);
    // Magic + version 1.0
    f.write("\x93NUMPY\x01\x00", 8);
    f.write(reinterpret_cast<const char*>(&header_len), 2);
    f.write(header.data(), header.size());
    f.write(reinterpret_cast<const char*>(data.data()), data.size());
}

// Load an NPY file using the same pattern as SimulationRunFactoryImpl.
std::shared_ptr<NpyArray> loadNpy(const std::filesystem::path& path) {
    auto arr = std::make_shared<NpyArray>();
    arr->LoadNPY(path.string().c_str());
    return arr;
}

} // namespace

/*
 * What it does: loads a 1x1x3 uint8 NPY where voxels hold semantic values 2, 18, and 45.
 * Setup: writes a valid NPY file to /tmp and loads it through Map3DImpl.
 * Checks: atVoxel() returns Occupied for all three voxels, so values above 1 are clamped to Occupied.
 */
TEST(Map3DImpl, ClampsValuesGreaterThanOneToOccupied) {
    const auto path = std::filesystem::temp_directory_path() / "test_clamp_gt1.npy";
    writeUint8Npy(path, {2, 18, 45}, 1, 1, 3);

    types::MapConfig cfg;
    cfg.resolution = 1.0 * cm;
    cfg.offset = Position3D{};
    Map3DImpl map(loadNpy(path), cfg);

    for (int z = 0; z < 3; ++z) {
        const auto pos = Position3D{
            0.0 * x_extent[cm], 0.0 * y_extent[cm],
            static_cast<double>(z) * z_extent[cm]};
        EXPECT_EQ(map.atVoxel(pos), types::VoxelOccupancy::Occupied)
            << "Value at z=" << z << " was not clamped to Occupied";
    }
    std::filesystem::remove(path);
}

/*
 * What it does: checks loading of the standard voxel values used by the project.
 * Setup: writes a tiny NPY file with normal map values.
 * Checks: Map3DImpl returns the expected voxel state for each value.
 */
TEST(Map3DImpl, StandardValuesPassThroughUnchanged) {
    const auto path = std::filesystem::temp_directory_path() / "test_passthrough.npy";
    writeUint8Npy(path, {0, 1, 2}, 1, 1, 3);

    types::MapConfig cfg;
    cfg.resolution = 1.0 * cm;
    cfg.offset = Position3D{};
    Map3DImpl map(loadNpy(path), cfg);

    EXPECT_EQ(map.atVoxel({0.0*x_extent[cm], 0.0*y_extent[cm], 0.0*z_extent[cm]}),
              types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel({0.0*x_extent[cm], 0.0*y_extent[cm], 1.0*z_extent[cm]}),
              types::VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel({0.0*x_extent[cm], 0.0*y_extent[cm], 2.0*z_extent[cm]}),
              types::VoxelOccupancy::Occupied);
    std::filesystem::remove(path);
}

} // namespace drone_mapper
