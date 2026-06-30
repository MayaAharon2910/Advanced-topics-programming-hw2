// =============================================================================
// Map3DImpl_test.cpp — Component tests for Map3DImpl's .npy loading
//
// Map3DImpl reads raw voxel bytes from an NpyArray (loaded from a .npy file)
// into an internal int8_t buffer. Some staff-provided maps (e.g. the complex
// benchmark house map) use semantic values greater than 1 (2=ground, 18=wall,
// 45=roof, etc.) to distinguish material types in the source data, while our
// VoxelOccupancy model only recognises Occupied=1 as "solid". These tests
// verify the loader clamps any raw value > 1 to Occupied so collision
// detection and map scoring treat all solid material consistently.
// =============================================================================

#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>

namespace drone_mapper {

/*
 * What it does: loads a 1×1×3 uint8 map where the voxels hold semantic
 *               values 2, 18, and 45 instead of the canonical 1=Occupied.
 * Setup: builds an NpyArray directly in memory with raw uint8_t data
 *        (mirrors the staff's benchmark_map.npy material encoding).
 * Checks: atVoxel() returns VoxelOccupancy::Occupied for all three voxels —
 *         proves the loader clamps any value > 1 instead of treating it as
 *         Unmapped (which would let the drone fly straight through solid
 *         material it never scanned).
 */
TEST(Map3DImpl, ClampsValuesGreaterThanOneToOccupied) {
    NpyArray::shape_t shape{1, 1, 3};
    std::vector<uint8_t> raw{2, 18, 45};
    auto npy = std::make_shared<NpyArray>(shape, raw.data());

    types::MapConfig cfg{};
    cfg.resolution = 1.0 * cm;
    cfg.offset = Position3D{};
    Map3DImpl map(npy, cfg);

    for (int z = 0; z < 3; ++z) {
        const auto pos = Position3D{
            0.0 * x_extent[cm], 0.0 * y_extent[cm],
            static_cast<double>(z) * z_extent[cm]};
        EXPECT_EQ(map.atVoxel(pos), types::VoxelOccupancy::Occupied)
            << "Raw value at z=" << z << " was not clamped to Occupied";
    }
}

/*
 * What it does: loads a map containing both standard values (0, 1) and a
 *               semantic value (2) in the same array.
 * Setup: 1×1×3 uint8 map with values {0, 1, 2}.
 * Checks: 0 stays Empty, 1 stays Occupied (both pass through unchanged),
 *         and 2 is clamped to Occupied — confirming standard values are
 *         not accidentally altered by the clamping logic.
 */
TEST(Map3DImpl, StandardValuesPassThroughUnchanged) {
    NpyArray::shape_t shape{1, 1, 3};
    std::vector<uint8_t> raw{0, 1, 2};
    auto npy = std::make_shared<NpyArray>(shape, raw.data());

    types::MapConfig cfg{};
    cfg.resolution = 1.0 * cm;
    cfg.offset = Position3D{};
    Map3DImpl map(npy, cfg);

    EXPECT_EQ(map.atVoxel(Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 0.0*z_extent[cm]}),
              types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 1.0*z_extent[cm]}),
              types::VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(Position3D{0.0*x_extent[cm], 0.0*y_extent[cm], 2.0*z_extent[cm]}),
              types::VoxelOccupancy::Occupied);
}

} // namespace drone_mapper
