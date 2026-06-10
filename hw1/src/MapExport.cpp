#include "MapExport.h"
#include <algorithm>
#include <fstream>

namespace {

constexpr int MIN_SCORE = 0;
constexpr int MAX_SCORE = 100;

} // namespace

bool exportMapToFile(const Map3D& map, const std::string& filename) {
    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    if (!file) {
        return false;
    }

    // Write dimensions
    file << map.width() << " " << map.height() << " " << map.depth() << "\n";

    // Write voxels in order: z, y, x
    for (size_t z = 0; z < map.depth(); ++z) {
        for (size_t y = 0; y < map.height(); ++y) {
            for (size_t x = 0; x < map.width(); ++x) {
                file << map.at(x, y, z) << " ";
            }
            file << "\n";
        }
    }

    return true;
}

bool getMapDimensions(const std::string& filename, size_t& width, size_t& height, size_t& depth) {
    std::ifstream file(filename);
    if (!file) {
        return false;
    }

    file >> width >> height >> depth;
    return !file.fail();
}

int calculateScore(const Map3D& output_map, const Map3D& ground_truth_map, const StrongPosition3D& start_position) {
    (void)start_position;

    if (ground_truth_map.width() != output_map.width() ||
        ground_truth_map.height() != output_map.height() ||
        ground_truth_map.depth() != output_map.depth()) {
        return MIN_SCORE;
    }

    // Only real ground-truth cells count; UNKNOWN and OUT_OF_BOUNDS are not part of the grade.
    size_t correct = 0;
    size_t total = 0;
    for (size_t x = 0; x < ground_truth_map.width(); ++x) {
        for (size_t y = 0; y < ground_truth_map.height(); ++y) {
            for (size_t z = 0; z < ground_truth_map.depth(); ++z) {
                int expected = ground_truth_map.at(x, y, z);
                if (expected != Map3D::FREE && expected != Map3D::OCCUPIED) {
                    continue;
                }
                ++total;
                if (output_map.at(x, y, z) == expected) {
                    ++correct;
                }
            }
        }
    }
    if (total == 0U) {
        return MIN_SCORE;
    }

    int score = static_cast<int>((correct * MAX_SCORE) / total);
    return std::min(MAX_SCORE, score);
}
