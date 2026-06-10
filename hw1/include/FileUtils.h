#pragma once

#include "Config.h"
#include "Map3D.h"
#include <string>
#include <optional>

// Parses a full string as a double and returns nullopt on invalid input.
std::optional<double> parse_double(const std::string& s);
// Parses a full string as an int and returns nullopt on invalid input.
std::optional<int> parse_int(const std::string& s);
// Loads drone_config.txt, applying defaults and appending recoverable errors.
Config parse_drone_config(const std::string& filename, std::string& errors);
// Loads mission_config.txt, applying defaults and appending recoverable errors.
MissionConfig parse_mission_config(const std::string& filename, std::string& errors);
// Loads map_input.txt into a Map3D and appends recoverable parsing errors.
Map3D parse_map_input(const std::string& filename, std::string& errors);
