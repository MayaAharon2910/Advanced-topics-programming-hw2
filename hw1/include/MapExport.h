#pragma once

#include "Map3D.h"
#include "StrongPosition3D.h"
#include <string>

// Writes a Map3D to a plain-text map file.
bool exportMapToFile(const Map3D& map, const std::string& filename);

// Calculates a 0-100 score by comparing mapped voxels against ground truth.
int calculateScore(const Map3D& output_map, const Map3D& ground_truth_map, const StrongPosition3D& start_position);

// Reads only the width, height, and depth header from a map file.
bool getMapDimensions(const std::string& filename, size_t& width, size_t& height, size_t& depth);
