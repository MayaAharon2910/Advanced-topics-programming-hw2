#include <gtest/gtest.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Map3DImpl.h>

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
