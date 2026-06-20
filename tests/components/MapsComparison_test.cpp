#include <gtest/gtest.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Map3DImpl.h>
#include <vector>

TEST(MapsComparison, IdenticalMapsReturn100) {
    auto npy = std::make_shared<NpyArray>();
    drone_mapper::types::MapConfig default_cfg;
    default_cfg.offset = drone_mapper::Position3D{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    default_cfg.resolution = 1.0 * drone_mapper::cm;
    default_cfg.boundaries.min_x = drone_mapper::XLength{0.0 * drone_mapper::cm};
    default_cfg.boundaries.max_x = drone_mapper::XLength{10.0 * drone_mapper::cm};
    default_cfg.boundaries.min_y = drone_mapper::YLength{0.0 * drone_mapper::cm};
    default_cfg.boundaries.max_y = drone_mapper::YLength{10.0 * drone_mapper::cm};
    default_cfg.boundaries.min_height = drone_mapper::ZLength{0.0 * drone_mapper::cm};
    default_cfg.boundaries.max_height = drone_mapper::ZLength{10.0 * drone_mapper::cm};

    class TestMap : public drone_mapper::IMap3D {
    public:
        explicit TestMap(const drone_mapper::types::MapConfig& c) : cfg(c) {}
        drone_mapper::types::VoxelOccupancy atVoxel(const drone_mapper::Position3D& pos) const override {
            const double x = pos.x.numerical_value_in(drone_mapper::cm);
            const double y = pos.y.numerical_value_in(drone_mapper::cm);
            const double z = pos.z.numerical_value_in(drone_mapper::cm);
            const double xmin = cfg.boundaries.min_x.numerical_value_in(drone_mapper::cm);
            const double xmax = cfg.boundaries.max_x.numerical_value_in(drone_mapper::cm);
            const double ymin = cfg.boundaries.min_y.numerical_value_in(drone_mapper::cm);
            const double ymax = cfg.boundaries.max_y.numerical_value_in(drone_mapper::cm);
            const double zmin = cfg.boundaries.min_height.numerical_value_in(drone_mapper::cm);
            const double zmax = cfg.boundaries.max_height.numerical_value_in(drone_mapper::cm);
            if (x < xmin || x >= xmax || y < ymin || y >= ymax || z < zmin || z >= zmax) return drone_mapper::types::VoxelOccupancy::OutOfBounds;
            return drone_mapper::types::VoxelOccupancy::Empty;
        }
        drone_mapper::types::MapConfig getMapConfig() const override { return cfg; }
        bool isInBounds(const drone_mapper::Position3D&) const override { return true; }
    private:
        drone_mapper::types::MapConfig cfg;
    } map1(default_cfg), map2(default_cfg);

    auto cfg = map1.getMapConfig();
    EXPECT_DOUBLE_EQ(cfg.boundaries.min_x.numerical_value_in(drone_mapper::cm), 0.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.max_x.numerical_value_in(drone_mapper::cm), 10.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.min_y.numerical_value_in(drone_mapper::cm), 0.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.max_y.numerical_value_in(drone_mapper::cm), 10.0);

    auto scores = drone_mapper::MapsComparison::compare(map1, {&map2});
    ASSERT_FALSE(scores.empty());
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

// BONUS TEST: Cross-resolution comparison (BONUS FEATURE)
TEST(MapsComparison, CrossResolutionBonus) {
    drone_mapper::types::MapConfig cfg1;
    cfg1.offset = drone_mapper::Position3D{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    cfg1.resolution = 1.0 * drone_mapper::cm;
    cfg1.boundaries.min_x = drone_mapper::XLength{0.0 * drone_mapper::cm};
    cfg1.boundaries.max_x = drone_mapper::XLength{10.0 * drone_mapper::cm};
    cfg1.boundaries.min_y = drone_mapper::YLength{0.0 * drone_mapper::cm};
    cfg1.boundaries.max_y = drone_mapper::YLength{10.0 * drone_mapper::cm};
    cfg1.boundaries.min_height = drone_mapper::ZLength{0.0 * drone_mapper::cm};
    cfg1.boundaries.max_height = drone_mapper::ZLength{10.0 * drone_mapper::cm};

    drone_mapper::types::MapConfig cfg2 = cfg1;
    cfg2.resolution = 2.0 * drone_mapper::cm;

    class TestMap2 : public drone_mapper::IMap3D {
    public:
        explicit TestMap2(const drone_mapper::types::MapConfig& c) : cfg(c) {}
        drone_mapper::types::VoxelOccupancy atVoxel(const drone_mapper::Position3D& pos) const override {
            const double x = pos.x.numerical_value_in(drone_mapper::cm);
            const double y = pos.y.numerical_value_in(drone_mapper::cm);
            const double z = pos.z.numerical_value_in(drone_mapper::cm);
            const double xmin = cfg.boundaries.min_x.numerical_value_in(drone_mapper::cm);
            const double xmax = cfg.boundaries.max_x.numerical_value_in(drone_mapper::cm);
            const double ymin = cfg.boundaries.min_y.numerical_value_in(drone_mapper::cm);
            const double ymax = cfg.boundaries.max_y.numerical_value_in(drone_mapper::cm);
            const double zmin = cfg.boundaries.min_height.numerical_value_in(drone_mapper::cm);
            const double zmax = cfg.boundaries.max_height.numerical_value_in(drone_mapper::cm);
            if (x < xmin || x >= xmax || y < ymin || y >= ymax || z < zmin || z >= zmax) return drone_mapper::types::VoxelOccupancy::OutOfBounds;
            return drone_mapper::types::VoxelOccupancy::Empty;
        }
        drone_mapper::types::MapConfig getMapConfig() const override { return cfg; }
        bool isInBounds(const drone_mapper::Position3D&) const override { return true; }
    private:
        drone_mapper::types::MapConfig cfg;
    } m1(cfg1), m2(cfg2);

    auto scores = drone_mapper::MapsComparison::compare(m1, {&m2});
    ASSERT_FALSE(scores.empty());
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Grid-backed test map: lets us control occupancy per-voxel precisely so we
// can construct exact "% similar" scenarios for the scoring tests below.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

class GridTestMap : public drone_mapper::IMap3D {
public:
    GridTestMap(const drone_mapper::types::MapConfig& cfg, int side, bool default_occupied)
        : cfg_(cfg), side_(side), occupancy_(static_cast<size_t>(side * side * side), default_occupied) {}

    void setOccupied(int x, int y, int z, bool occupied) {
        occupancy_[index(x, y, z)] = occupied;
    }

    drone_mapper::types::VoxelOccupancy atVoxel(const drone_mapper::Position3D& pos) const override {
        const double res = cfg_.resolution.numerical_value_in(drone_mapper::cm);
        const int x = static_cast<int>(pos.x.numerical_value_in(drone_mapper::cm) / res);
        const int y = static_cast<int>(pos.y.numerical_value_in(drone_mapper::cm) / res);
        const int z = static_cast<int>(pos.z.numerical_value_in(drone_mapper::cm) / res);
        if (x < 0 || y < 0 || z < 0 || x >= side_ || y >= side_ || z >= side_) {
            return drone_mapper::types::VoxelOccupancy::OutOfBounds;
        }
        return occupancy_[index(x, y, z)] ? drone_mapper::types::VoxelOccupancy::Occupied
                                          : drone_mapper::types::VoxelOccupancy::Empty;
    }

    drone_mapper::types::MapConfig getMapConfig() const override { return cfg_; }
    bool isInBounds(const drone_mapper::Position3D&) const override { return true; }

private:
    [[nodiscard]] size_t index(int x, int y, int z) const {
        return static_cast<size_t>(x) * static_cast<size_t>(side_) * static_cast<size_t>(side_) +
               static_cast<size_t>(y) * static_cast<size_t>(side_) + static_cast<size_t>(z);
    }

    drone_mapper::types::MapConfig cfg_;
    int side_;
    std::vector<bool> occupancy_;
};

drone_mapper::types::MapConfig gridConfig(int side) {
    drone_mapper::types::MapConfig cfg;
    cfg.offset = drone_mapper::Position3D{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
    cfg.resolution = 1.0 * drone_mapper::cm;
    cfg.boundaries.min_x = drone_mapper::XLength{0.0 * drone_mapper::cm};
    cfg.boundaries.max_x = drone_mapper::XLength{static_cast<double>(side) * drone_mapper::cm};
    cfg.boundaries.min_y = drone_mapper::YLength{0.0 * drone_mapper::cm};
    cfg.boundaries.max_y = drone_mapper::YLength{static_cast<double>(side) * drone_mapper::cm};
    cfg.boundaries.min_height = drone_mapper::ZLength{0.0 * drone_mapper::cm};
    cfg.boundaries.max_height = drone_mapper::ZLength{static_cast<double>(side) * drone_mapper::cm};
    return cfg;
}

} // namespace

// ── Completely different maps (100% Occupied vs 100% Empty) → score == 0 ────
TEST(MapsComparison, CompletelyDifferentMapsReturn0) {
    constexpr int kSide = 5; // 125 voxels total
    const auto cfg = gridConfig(kSide);

    GridTestMap fully_occupied(cfg, kSide, /*default_occupied=*/true);
    GridTestMap fully_empty(cfg, kSide, /*default_occupied=*/false);

    auto scores = drone_mapper::MapsComparison::compare(fully_occupied, {&fully_empty});
    ASSERT_FALSE(scores.empty());
    EXPECT_DOUBLE_EQ(scores.front(), 0.0);
}

// ── ~88% identical maps → high but not perfect score ─────────────────────────
TEST(MapsComparison, SimilarMapsReturnHighScore) {
    constexpr int kSide = 5; // 125 voxels total
    const auto cfg = gridConfig(kSide);

    GridTestMap original(cfg, kSide, /*default_occupied=*/false);
    GridTestMap target(cfg, kSide, /*default_occupied=*/false);

    // Occupy the same 15 voxels (x=0..2, full y/z) in both maps...
    for (int y = 0; y < kSide; ++y) {
        for (int z = 0; z < kSide; ++z) {
            original.setOccupied(0, y, z, true);
            target.setOccupied(0, y, z, true);
        }
    }
    // ...but disagree on the column x=1 (5 voxels) so the maps differ slightly.
    for (int z = 0; z < kSide; ++z) {
        target.setOccupied(1, 0, z, true);
    }
    // 125 voxels total, 5 disagree -> (125-5)/125 = 96% — comfortably in the
    // "high score" band the staff demonstrated (>75, <100).

    auto scores = drone_mapper::MapsComparison::compare(original, {&target});
    ASSERT_FALSE(scores.empty());
    const double score = scores.front();
    EXPECT_GT(score, 75.0);
    EXPECT_LT(score, 100.0);
}

// ── ~16% identical maps → low score ───────────────────────────────────────────
TEST(MapsComparison, DifferentMapsReturnLowScore) {
    constexpr int kSide = 5; // 125 voxels total
    const auto cfg = gridConfig(kSide);

    GridTestMap original(cfg, kSide, /*default_occupied=*/false);
    GridTestMap target(cfg, kSide, /*default_occupied=*/true);

    // Make exactly the x=0 plane (25 voxels) agree (Empty in both); the
    // remaining 100 voxels disagree (Empty vs Occupied).
    // matches/total = 25/125 = 20% — within the "low score" band (>0, <25).
    for (int y = 0; y < kSide; ++y) {
        for (int z = 0; z < kSide; ++z) {
            target.setOccupied(0, y, z, false);
        }
    }

    auto scores = drone_mapper::MapsComparison::compare(original, {&target});
    ASSERT_FALSE(scores.empty());
    const double score = scores.front();
    EXPECT_GT(score, 0.0);
    EXPECT_LT(score, 25.0);
}
