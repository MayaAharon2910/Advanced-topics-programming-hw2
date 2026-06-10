#include "Map3D.h"
#include "MapExport.h"

#include <gtest/gtest.h>

namespace {

StrongPosition3D origin() {
    return {
        0.0 * mp_units::si::unit_symbols::cm,
        0.0 * mp_units::si::unit_symbols::cm,
        0.0 * mp_units::si::unit_symbols::cm
    };
}

} // namespace

// ─────────────────────────────────────────────
// Map3D — construction and default state
// ─────────────────────────────────────────────

TEST(Map3D, DefaultValueIsUnknown) {
    Map3D map(3, 3, 3);
    EXPECT_EQ(map.at(1, 1, 1), Map3D::UNKNOWN);
}

TEST(Map3D, DimensionsAreCorrect) {
    Map3D map(4, 5, 6);
    EXPECT_EQ(map.width(),  4u);
    EXPECT_EQ(map.height(), 5u);
    EXPECT_EQ(map.depth(),  6u);
}

// ─────────────────────────────────────────────
// Map3D — set / get round-trips
// ─────────────────────────────────────────────

TEST(Map3D, SetAndGetFree) {
    Map3D map(3, 3, 3);
    map.set(1, 1, 1, Map3D::FREE);
    EXPECT_EQ(map.at(1, 1, 1), Map3D::FREE);
}

TEST(Map3D, SetAndGetOccupied) {
    Map3D map(3, 3, 3);
    map.set(2, 0, 0, Map3D::OCCUPIED);
    EXPECT_EQ(map.at(2, 0, 0), Map3D::OCCUPIED);
}

TEST(Map3D, SetDoesNotAffectNeighbours) {
    Map3D map(3, 3, 3);
    map.set(1, 1, 1, Map3D::FREE);
    EXPECT_EQ(map.at(0, 1, 1), Map3D::UNKNOWN);
    EXPECT_EQ(map.at(2, 1, 1), Map3D::UNKNOWN);
    EXPECT_EQ(map.at(1, 0, 1), Map3D::UNKNOWN);
    EXPECT_EQ(map.at(1, 2, 1), Map3D::UNKNOWN);
}

TEST(Map3D, OverwriteOccupiedWithFree) {
    Map3D map(2, 2, 2);
    map.set(0, 0, 0, Map3D::OCCUPIED);
    map.set(0, 0, 0, Map3D::FREE);
    EXPECT_EQ(map.at(0, 0, 0), Map3D::FREE);
}

// ─────────────────────────────────────────────
// Map3D — out-of-bounds behaviour
// ─────────────────────────────────────────────

TEST(Map3D, NegativeCoordinateReturnsOutOfBounds) {
    Map3D map(3, 3, 3);
    EXPECT_EQ(map.at(-1, 0, 0), Map3D::OUT_OF_BOUNDS);
    EXPECT_EQ(map.at(0, -1, 0), Map3D::OUT_OF_BOUNDS);
    EXPECT_EQ(map.at(0, 0, -1), Map3D::OUT_OF_BOUNDS);
}

TEST(Map3D, PastEndCoordinateReturnsOutOfBounds) {
    Map3D map(3, 3, 3);
    EXPECT_EQ(map.at(3, 0, 0), Map3D::OUT_OF_BOUNDS);
    EXPECT_EQ(map.at(0, 3, 0), Map3D::OUT_OF_BOUNDS);
    EXPECT_EQ(map.at(0, 0, 3), Map3D::OUT_OF_BOUNDS);
}

TEST(Map3D, SetOutOfBoundsIsIgnored) {
    Map3D map(2, 2, 2);
    map.set(-1, 0, 0, Map3D::FREE);  // should not crash
    map.set(5,  0, 0, Map3D::FREE);  // should not crash
    // No assertion needed — just verifying no crash / UB
}

// ─────────────────────────────────────────────
// Map3D — mission bounds and fillOutOfBounds
// ─────────────────────────────────────────────

TEST(Map3D, VoxelsOutsideMissionBoundsReturnOutOfBounds) {
    // 5×5×5 map, mission bounded to x=[1,3], y=[1,3], z=[1,3]
    Map3D map(5, 5, 5);
    map.setMissionBounds(1, 3, 1, 3, 1, 3);
    map.fillOutOfBoundsVoxels();

    EXPECT_EQ(map.at(0, 0, 0), Map3D::OUT_OF_BOUNDS);  // outside
    EXPECT_EQ(map.at(4, 4, 4), Map3D::OUT_OF_BOUNDS);  // outside
    EXPECT_EQ(map.at(2, 2, 2), Map3D::UNKNOWN);          // inside
}

TEST(Map3D, SetInsideMissionBoundsSucceeds) {
    Map3D map(5, 5, 5);
    map.setMissionBounds(1, 3, 1, 3, 1, 3);
    map.set(2, 2, 2, Map3D::FREE);
    EXPECT_EQ(map.at(2, 2, 2), Map3D::FREE);
}

TEST(Map3D, SetOutsideMissionBoundsIsIgnored) {
    Map3D map(5, 5, 5);
    map.setMissionBounds(1, 3, 1, 3, 1, 3);
    map.fillOutOfBoundsVoxels();
    map.set(0, 0, 0, Map3D::FREE);  // outside mission bounds — should be ignored
    EXPECT_EQ(map.at(0, 0, 0), Map3D::OUT_OF_BOUNDS);
}

// ─────────────────────────────────────────────
// calculateScore — correctness
// ─────────────────────────────────────────────

TEST(MapExportScore, CountsFreeAndOccupiedCells) {
    Map3D ground_truth(2, 1, 1);
    ground_truth.set(0, 0, 0, Map3D::FREE);
    ground_truth.set(1, 0, 0, Map3D::OCCUPIED);

    Map3D output(2, 1, 1);
    output.set(0, 0, 0, Map3D::FREE);
    output.set(1, 0, 0, Map3D::OCCUPIED);

    EXPECT_EQ(calculateScore(output, ground_truth, origin()), 100);
}

TEST(MapExportScore, PenalizesMissingWalls) {
    Map3D ground_truth(2, 1, 1);
    ground_truth.set(0, 0, 0, Map3D::FREE);
    ground_truth.set(1, 0, 0, Map3D::OCCUPIED);

    Map3D output(2, 1, 1);
    output.set(0, 0, 0, Map3D::FREE);
    output.set(1, 0, 0, Map3D::UNKNOWN);  // missed wall

    EXPECT_EQ(calculateScore(output, ground_truth, origin()), 50);
}

TEST(MapExportScore, AllUnknownScoresZero) {
    Map3D ground_truth(3, 1, 1);
    ground_truth.set(0, 0, 0, Map3D::FREE);
    ground_truth.set(1, 0, 0, Map3D::FREE);
    ground_truth.set(2, 0, 0, Map3D::OCCUPIED);

    Map3D output(3, 1, 1);
    // output is all UNKNOWN — nothing mapped

    EXPECT_EQ(calculateScore(output, ground_truth, origin()), 0);
}

TEST(MapExportScore, OutOfBoundsInGroundTruthNotCounted) {
    // 3-voxel map: OOB | FREE | OCCUPIED
    Map3D ground_truth(3, 1, 1);
    ground_truth.setMissionBounds(1, 2, 0, 0, 0, 0);
    ground_truth.fillOutOfBoundsVoxels();
    ground_truth.set(1, 0, 0, Map3D::FREE);
    ground_truth.set(2, 0, 0, Map3D::OCCUPIED);

    Map3D output(3, 1, 1);
    output.setMissionBounds(1, 2, 0, 0, 0, 0);
    output.fillOutOfBoundsVoxels();
    output.set(1, 0, 0, Map3D::FREE);
    output.set(2, 0, 0, Map3D::OCCUPIED);

    // Only the 2 in-bounds voxels count; OOB is ignored
    EXPECT_EQ(calculateScore(output, ground_truth, origin()), 100);
}

TEST(MapExportScore, WrongFreeReducesScore) {
    // Ground truth: 4 FREE voxels, 0 OCCUPIED
    Map3D ground_truth(4, 1, 1);
    for (int i = 0; i < 4; ++i) {
        ground_truth.set(i, 0, 0, Map3D::FREE);
    }

    Map3D output(4, 1, 1);
    output.set(0, 0, 0, Map3D::FREE);    // correct
    output.set(1, 0, 0, Map3D::FREE);    // correct
    output.set(2, 0, 0, Map3D::UNKNOWN); // missed
    output.set(3, 0, 0, Map3D::UNKNOWN); // missed

    // 2 correct out of 4 total = 50
    EXPECT_EQ(calculateScore(output, ground_truth, origin()), 50);
}

TEST(MapExportScore, DimensionMismatchScoresZero) {
    Map3D ground_truth(3, 3, 3);
    Map3D output(2, 2, 2);  // different size

    EXPECT_EQ(calculateScore(output, ground_truth, origin()), 0);
}

TEST(MapExportScore, EmptyGroundTruthScoresZero) {
    // All voxels UNKNOWN in ground truth → total=0 → score=0
    Map3D ground_truth(2, 2, 2);
    Map3D output(2, 2, 2);
    output.set(0, 0, 0, Map3D::FREE);

    EXPECT_EQ(calculateScore(output, ground_truth, origin()), 0);
}
