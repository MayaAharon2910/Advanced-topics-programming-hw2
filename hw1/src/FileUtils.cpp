#include "FileUtils.h"
#include <cctype>
#include <fstream>
#include <sstream>

namespace {

constexpr double DEFAULT_MAX_ROTATION_DEG  = 90.0;
constexpr double DEFAULT_MAX_ELEVATION_CM  = 50.0;
constexpr double DEFAULT_MAX_ADVANCE_CM    = 100.0;
constexpr double DEFAULT_MIN_PASS_WIDTH_CM = 50.0;
constexpr double DEFAULT_MIN_PASS_LENGTH_CM = 50.0;
constexpr double DEFAULT_MIN_PASS_HEIGHT_CM = 80.0;
constexpr double DEFAULT_LIDAR_Z_MIN_CM    = 20.0;
constexpr double DEFAULT_LIDAR_Z_MAX_CM    = 100.0;
constexpr int    DEFAULT_LIDAR_FOVC        = 3;
constexpr double DEFAULT_LIDAR_D_CM        = 5.0;
constexpr size_t DEFAULT_MAP_DIM           = 1;

constexpr size_t DRONE_NUMERIC_TOTAL_COUNT = 10;
constexpr size_t IDX_MAX_ROTATION_DEG      = 0;
constexpr size_t IDX_MAX_ELEVATION_CM      = 1;
constexpr size_t IDX_MAX_ADVANCE_CM        = 2;
constexpr size_t IDX_MIN_PASS_WIDTH_CM     = 3;
constexpr size_t IDX_MIN_PASS_LENGTH_CM    = 4;
constexpr size_t IDX_MIN_PASS_HEIGHT_CM    = 5;
constexpr size_t IDX_LIDAR_Z_MIN_CM        = 6;
constexpr size_t IDX_LIDAR_Z_MAX_CM        = 7;
constexpr size_t IDX_LIDAR_D_CM            = 8;
constexpr size_t IDX_LIDAR_FOVC            = 9;

constexpr size_t NUMERIC_BOUNDS_COUNT      = 6;
constexpr size_t NUMERIC_START_COUNT       = 9;
constexpr size_t NUMERIC_TOTAL_COUNT       = 10;
constexpr size_t IDX_MIN_X   = 0;
constexpr size_t IDX_MAX_X   = 1;
constexpr size_t IDX_MIN_Y   = 2;
constexpr size_t IDX_MAX_Y   = 3;
constexpr size_t IDX_MIN_H   = 4;
constexpr size_t IDX_MAX_H   = 5;
constexpr size_t IDX_START_X     = 6;
constexpr size_t IDX_START_Y     = 7;
constexpr size_t IDX_START_Z     = 8;
constexpr size_t IDX_START_ANGLE = 9;
constexpr size_t RECHARGE_POSITION_COMPONENT_COUNT = 3;
constexpr double MISSION_CENTER_DIVISOR = 2.0;

std::string trim_copy(const std::string& text) {
    auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) { return ""; }
    auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

std::string strip_inline_comment(const std::string& text) {
    return text.substr(0, text.find('#'));
}

void validate_drone_config(Config& config, std::string& errors) {
    // Keep bad numeric input recoverable: the simulator can still run with defaults.
    if (config.max_rotation.numerical_value_in(mp_units::si::unit_symbols::deg) <= 0.0) {
        errors += "Invalid max_rotation_deg range; using default 90.\n";
        config.max_rotation = DEFAULT_MAX_ROTATION_DEG * mp_units::si::unit_symbols::deg;
    }
    if (config.max_elevation.numerical_value_in(mp_units::si::unit_symbols::cm) <= 0.0) {
        errors += "Invalid max_elevation_cm range; using default 50.\n";
        config.max_elevation = DEFAULT_MAX_ELEVATION_CM * mp_units::si::unit_symbols::cm;
    }
    if (config.max_advance.numerical_value_in(mp_units::si::unit_symbols::cm) <= 0.0) {
        errors += "Invalid max_advance_cm range; using default 100.\n";
        config.max_advance = DEFAULT_MAX_ADVANCE_CM * mp_units::si::unit_symbols::cm;
    }
    if (config.min_pass_width_cm.numerical_value_in(mp_units::si::unit_symbols::cm) <= 0.0) {
        errors += "Invalid min_pass_width_cm range; using default 50.\n";
        config.min_pass_width_cm = DEFAULT_MIN_PASS_WIDTH_CM * mp_units::si::unit_symbols::cm;
    }
    if (config.min_pass_length_cm.numerical_value_in(mp_units::si::unit_symbols::cm) <= 0.0) {
        errors += "Invalid min_pass_length_cm range; using default 50.\n";
        config.min_pass_length_cm = DEFAULT_MIN_PASS_LENGTH_CM * mp_units::si::unit_symbols::cm;
    }
    if (config.min_pass_height_cm.numerical_value_in(mp_units::si::unit_symbols::cm) <= 0.0) {
        errors += "Invalid min_pass_height_cm range; using default 80.\n";
        config.min_pass_height_cm = DEFAULT_MIN_PASS_HEIGHT_CM * mp_units::si::unit_symbols::cm;
    }
    if (config.lidar_z_min_cm.numerical_value_in(mp_units::si::unit_symbols::cm) < 0.0) {
        errors += "Invalid lidar_z_min_cm range; using default 20.\n";
        config.lidar_z_min_cm = DEFAULT_LIDAR_Z_MIN_CM * mp_units::si::unit_symbols::cm;
    }
    if (config.lidar_z_max_cm <= config.lidar_z_min_cm) {
        errors += "Invalid lidar_z_max_cm range; using default 100.\n";
        config.lidar_z_max_cm = DEFAULT_LIDAR_Z_MAX_CM * mp_units::si::unit_symbols::cm;
    }
    if (config.lidar_fovc < 1) {
        errors += "Invalid lidar_fovc range; using default 3.\n";
        config.lidar_fovc = DEFAULT_LIDAR_FOVC;
    }
    if (config.lidar_d_cm.numerical_value_in(mp_units::si::unit_symbols::cm) <= 0.0) {
        errors += "Invalid lidar_d_cm range; using default 5.\n";
        config.lidar_d_cm = DEFAULT_LIDAR_D_CM * mp_units::si::unit_symbols::cm;
    }
}

std::vector<std::string> splitList(const std::string& text, char separator) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream stream(text);
    while (std::getline(stream, token, separator)) {
        if (!token.empty()) { tokens.push_back(token); }
    }
    return tokens;
}

} // namespace

std::optional<double> parse_double(const std::string& s) {
    try {
        std::size_t consumed = 0;
        std::string trimmed = trim_copy(s);
        double value = std::stod(trimmed, &consumed);
        while (consumed < trimmed.size() && std::isspace(static_cast<unsigned char>(trimmed[consumed]))) {
            ++consumed;
        }
        if (consumed != trimmed.size()) { return std::nullopt; }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<int> parse_int(const std::string& s) {
    try {
        std::size_t consumed = 0;
        std::string trimmed = trim_copy(s);
        int value = std::stoi(trimmed, &consumed);
        while (consumed < trimmed.size() && std::isspace(static_cast<unsigned char>(trimmed[consumed]))) {
            ++consumed;
        }
        if (consumed != trimmed.size()) { return std::nullopt; }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

Config parse_drone_config(const std::string& filename, std::string& errors) {
    Config config;
    // Defaults are assigned up front so a missing key does not need special handling later.
    config.max_rotation      = DEFAULT_MAX_ROTATION_DEG  * mp_units::si::unit_symbols::deg;
    config.max_elevation     = DEFAULT_MAX_ELEVATION_CM  * mp_units::si::unit_symbols::cm;
    config.max_advance       = DEFAULT_MAX_ADVANCE_CM    * mp_units::si::unit_symbols::cm;
    config.min_pass_width_cm = DEFAULT_MIN_PASS_WIDTH_CM * mp_units::si::unit_symbols::cm;
    config.min_pass_length_cm = DEFAULT_MIN_PASS_LENGTH_CM * mp_units::si::unit_symbols::cm;
    config.min_pass_height_cm = DEFAULT_MIN_PASS_HEIGHT_CM * mp_units::si::unit_symbols::cm;
    config.lidar_z_min_cm    = DEFAULT_LIDAR_Z_MIN_CM    * mp_units::si::unit_symbols::cm;
    config.lidar_z_max_cm    = DEFAULT_LIDAR_Z_MAX_CM    * mp_units::si::unit_symbols::cm;
    config.lidar_fovc        = DEFAULT_LIDAR_FOVC;
    config.lidar_d_cm        = DEFAULT_LIDAR_D_CM        * mp_units::si::unit_symbols::cm;

    std::ifstream file(filename);
    if (!file) {
        errors += "Cannot open " + filename + "\n";
        return config;
    }

    auto parse_value = [&](const std::string& key, const std::string& value) {
        auto val = parse_double(value);
        if (!val) { errors += "Invalid " + key + ": " + value + "\n"; }
        return val;
    };

    auto apply_key_value = [&](const std::string& key, const std::string& value) {
        if (key == "max_rotation_deg" || key == "max_rotation") {
            auto val = parse_value(key, value);
            if (val) { config.max_rotation = *val * mp_units::si::unit_symbols::deg; }
        } else if (key == "max_elevation_cm" || key == "max_elevation") {
            auto val = parse_value(key, value);
            if (val) { config.max_elevation = *val * mp_units::si::unit_symbols::cm; }
        } else if (key == "max_advance" || key == "max_advance_cm") {
            auto val = parse_value(key, value);
            if (val) { config.max_advance = *val * mp_units::si::unit_symbols::cm; }
        } else if (key == "min_pass_width" || key == "min_pass_width_cm") {
            auto val = parse_value(key, value);
            if (val) { config.min_pass_width_cm = *val * mp_units::si::unit_symbols::cm; }
        } else if (key == "min_pass_length" || key == "min_pass_length_cm") {
            auto val = parse_value(key, value);
            if (val) { config.min_pass_length_cm = *val * mp_units::si::unit_symbols::cm; }
        } else if (key == "min_pass_height" || key == "min_pass_height_cm") {
            auto val = parse_value(key, value);
            if (val) { config.min_pass_height_cm = *val * mp_units::si::unit_symbols::cm; }
        } else if (key == "lidar_z_min_cm" || key == "lidar_z_min") {
            auto val = parse_value(key, value);
            if (val) { config.lidar_z_min_cm = *val * mp_units::si::unit_symbols::cm; }
        } else if (key == "lidar_z_max_cm" || key == "lidar_z_max") {
            auto val = parse_value(key, value);
            if (val) { config.lidar_z_max_cm = *val * mp_units::si::unit_symbols::cm; }
        } else if (key == "lidar_fovc") {
            auto val = parse_value(key, value);
            if (val) { config.lidar_fovc = static_cast<int>(*val); }
        } else if (key == "lidar_d_cm" || key == "lidar_d") {
            auto val = parse_value(key, value);
            if (val) { config.lidar_d_cm = *val * mp_units::si::unit_symbols::cm; }
        }
    };

    std::vector<double> numeric_values;
    std::string line;
    while (std::getline(file, line)) {
        line = trim_copy(strip_inline_comment(line));
        if (line.empty()) { continue; }

        auto delimiter = line.find('=');
        if (delimiter == std::string::npos) { delimiter = line.find(':'); }
        if (delimiter != std::string::npos) {
            apply_key_value(trim_copy(line.substr(0, delimiter)),
                            trim_copy(line.substr(delimiter + 1)));
            continue;
        }

        std::size_t whitespace = line.find_first_of(" \t");
        if (whitespace != std::string::npos) {
            apply_key_value(trim_copy(line.substr(0, whitespace)),
                            trim_copy(line.substr(whitespace + 1)));
            continue;
        }

        auto numeric_value = parse_double(line);
        if (numeric_value) { numeric_values.push_back(*numeric_value); }
        else { errors += "Invalid drone_config line: " + line + "\n"; }
    }

    // Support the course-style plain numeric drone_config format, one value per line.
    if (!numeric_values.empty()) {
        if (numeric_values.size() >= DRONE_NUMERIC_TOTAL_COUNT) {
            config.max_rotation      = numeric_values[IDX_MAX_ROTATION_DEG]   * mp_units::si::unit_symbols::deg;
            config.max_elevation     = numeric_values[IDX_MAX_ELEVATION_CM]   * mp_units::si::unit_symbols::cm;
            config.max_advance       = numeric_values[IDX_MAX_ADVANCE_CM]     * mp_units::si::unit_symbols::cm;
            config.min_pass_width_cm = numeric_values[IDX_MIN_PASS_WIDTH_CM]  * mp_units::si::unit_symbols::cm;
            config.min_pass_length_cm = numeric_values[IDX_MIN_PASS_LENGTH_CM] * mp_units::si::unit_symbols::cm;
            config.min_pass_height_cm = numeric_values[IDX_MIN_PASS_HEIGHT_CM] * mp_units::si::unit_symbols::cm;
            config.lidar_z_min_cm    = numeric_values[IDX_LIDAR_Z_MIN_CM]     * mp_units::si::unit_symbols::cm;
            config.lidar_z_max_cm    = numeric_values[IDX_LIDAR_Z_MAX_CM]     * mp_units::si::unit_symbols::cm;
            config.lidar_d_cm        = numeric_values[IDX_LIDAR_D_CM]         * mp_units::si::unit_symbols::cm;
            config.lidar_fovc        = static_cast<int>(numeric_values[IDX_LIDAR_FOVC]);
        } else {
            errors += "Numeric drone_config is missing required values; using defaults for missing fields.\n";
        }
    }
    validate_drone_config(config, errors);
    return config;
}

MissionConfig parse_mission_config(const std::string& filename, std::string& errors) {
    MissionConfig mission;
    mission.min_x             = 0.0 * mp_units::si::unit_symbols::cm;
    mission.max_x             = 0.0 * mp_units::si::unit_symbols::cm;
    mission.min_y             = 0.0 * mp_units::si::unit_symbols::cm;
    mission.max_y             = 0.0 * mp_units::si::unit_symbols::cm;
    mission.min_height        = 0.0 * mp_units::si::unit_symbols::cm;
    mission.max_height        = 0.0 * mp_units::si::unit_symbols::cm;
    mission.start_position    = {0.0 * mp_units::si::unit_symbols::cm,
                                  0.0 * mp_units::si::unit_symbols::cm,
                                  0.0 * mp_units::si::unit_symbols::cm};
    mission.start_orientation = 0.0 * mp_units::si::unit_symbols::deg;
    mission.resolution_xy_digits     = 0;
    mission.resolution_height_digits = 0;
    mission.has_bounds = false;

    bool has_start_x      = false;
    bool has_start_y      = false;
    bool has_start_height = false;
    bool has_start_angle  = false;
    std::vector<double> numeric_values;

    std::ifstream file(filename);
    if (!file) {
        errors += "Cannot open " + filename + "\n";
        return mission;
    }

    auto parse_value = [&](const std::string& key, const std::string& value) {
        auto val = parse_double(value);
        if (!val) { errors += "Invalid " + key + ": " + value + "\n"; }
        return val;
    };

    auto apply_key_value = [&](const std::string& key, const std::string& value) {
        // Accept a few key aliases because the supplied examples are not perfectly uniform.
        if (key == "mission_min_x_cm" || key == "mission_min_x" || key == "min_x") {
            auto val = parse_value(key, value);
            if (val) { mission.min_x = *val * mp_units::si::unit_symbols::cm; mission.has_bounds = true; }
        } else if (key == "mission_max_x_cm" || key == "mission_max_x" || key == "max_x") {
            auto val = parse_value(key, value);
            if (val) { mission.max_x = *val * mp_units::si::unit_symbols::cm; mission.has_bounds = true; }
        } else if (key == "mission_min_y_cm" || key == "mission_min_y" || key == "min_y") {
            auto val = parse_value(key, value);
            if (val) { mission.min_y = *val * mp_units::si::unit_symbols::cm; mission.has_bounds = true; }
        } else if (key == "mission_max_y_cm" || key == "mission_max_y" || key == "max_y") {
            auto val = parse_value(key, value);
            if (val) { mission.max_y = *val * mp_units::si::unit_symbols::cm; mission.has_bounds = true; }
        } else if (key == "mission_min_height_cm" || key == "mission_min_height" ||
                   key == "min_height" || key == "min_h") {
            auto val = parse_value(key, value);
            if (val) { mission.min_height = *val * mp_units::si::unit_symbols::cm; mission.has_bounds = true; }
        } else if (key == "mission_max_height_cm" || key == "mission_max_height" ||
                   key == "max_height" || key == "max_h") {
            auto val = parse_value(key, value);
            if (val) { mission.max_height = *val * mp_units::si::unit_symbols::cm; mission.has_bounds = true; }
        } else if (key == "start_x_cm" || key == "start_x" ||
                   key == "initial_x_cm" || key == "initial_x" ||
                   key == "initial_position_x_cm" || key == "initial_position_x") {
            auto val = parse_value(key, value);
            if (val) { mission.start_position.x = *val * mp_units::si::unit_symbols::cm; has_start_x = true; }
        } else if (key == "start_y_cm" || key == "start_y" ||
                   key == "initial_y_cm" || key == "initial_y" ||
                   key == "initial_position_y_cm" || key == "initial_position_y") {
            auto val = parse_value(key, value);
            if (val) { mission.start_position.y = *val * mp_units::si::unit_symbols::cm; has_start_y = true; }
        } else if (key == "start_height_cm" || key == "start_height" ||
                   key == "start_z_cm" || key == "start_z" || key == "start_h" ||
                   key == "initial_height_cm" || key == "initial_height" ||
                   key == "initial_z_cm" || key == "initial_z" || key == "initial_h" ||
                   key == "initial_position_z_cm" || key == "initial_position_z" ||
                   key == "initial_position_height_cm" || key == "initial_position_height") {
            auto val = parse_value(key, value);
            if (val) { mission.start_position.z = *val * mp_units::si::unit_symbols::cm; has_start_height = true; }
        } else if (key == "start_angle_deg" || key == "start_xy_angle_deg" ||
                   key == "start_orientation_deg" || key == "initial_angle_deg" ||
                   key == "initial_xy_angle_deg" || key == "initial_orientation_deg") {
            auto val = parse_value(key, value);
            if (val) { mission.start_orientation = *val * mp_units::si::unit_symbols::deg; has_start_angle = true; }
        } else if (key == "resolution_xy" || key == "resolution_xy_digits" ||
                   key == "xy_resolution_digits" || key == "decimal_xy") {
            auto val = parse_int(value);
            if (val) { mission.resolution_xy_digits = *val; }
            else { errors += "Invalid resolution_xy: " + value + "\n"; }
        } else if (key == "resolution_height" || key == "resolution_height_digits" ||
                   key == "h_resolution_digits" || key == "decimal_h") {
            auto val = parse_int(value);
            if (val) { mission.resolution_height_digits = *val; }
            else { errors += "Invalid resolution_height: " + value + "\n"; }
        } else if (key == "resolution" || key == "resolution_digits") {
            auto val = parse_int(value);
            if (val) { mission.resolution_xy_digits = *val; mission.resolution_height_digits = *val; }
            else { errors += "Invalid resolution: " + value + "\n"; }
        } else if (key == "recharge_positions" || key == "recharge_position") {
            auto positions = splitList(value, ';');
            for (const auto& position : positions) {
                auto trimmed = trim_copy(position);
                if (trimmed.empty()) { continue; }
                auto parts = splitList(trimmed, ',');
                if (parts.size() != RECHARGE_POSITION_COMPONENT_COUNT) {
                    errors += "Invalid recharge position format: " + trimmed + "\n";
                    continue;
                }
                auto px = parse_double(parts[0]);
                auto py = parse_double(parts[1]);
                auto pz = parse_double(parts[2]);
                if (!px || !py || !pz) {
                    errors += "Invalid recharge position values: " + trimmed + "\n";
                    continue;
                }
                mission.recharge_positions.push_back({
                    *px * mp_units::si::unit_symbols::cm,
                    *py * mp_units::si::unit_symbols::cm,
                    *pz * mp_units::si::unit_symbols::cm
                });
            }
        }
    };

    std::string line;
    while (std::getline(file, line)) {
        line = trim_copy(strip_inline_comment(line));
        if (line.empty()) { continue; }

        auto delimiter = line.find('=');
        if (delimiter == std::string::npos) { delimiter = line.find(':'); }
        if (delimiter != std::string::npos) {
            apply_key_value(trim_copy(line.substr(0, delimiter)),
                            trim_copy(line.substr(delimiter + 1)));
            continue;
        }

        std::size_t whitespace = line.find_first_of(" \t");
        if (whitespace != std::string::npos) {
            apply_key_value(trim_copy(line.substr(0, whitespace)),
                            trim_copy(line.substr(whitespace + 1)));
            continue;
        }

        auto numeric_value = parse_double(line);
        if (numeric_value) { numeric_values.push_back(*numeric_value); }
        else { errors += "Invalid mission_config line: " + line + "\n"; }
    }

    // Older test resources use a plain numeric mission format, so keep supporting it.
    if (!numeric_values.empty()) {
        if (numeric_values.size() >= NUMERIC_BOUNDS_COUNT) {
            mission.min_x      = numeric_values[IDX_MIN_X] * mp_units::si::unit_symbols::cm;
            mission.max_x      = numeric_values[IDX_MAX_X] * mp_units::si::unit_symbols::cm;
            mission.min_y      = numeric_values[IDX_MIN_Y] * mp_units::si::unit_symbols::cm;
            mission.max_y      = numeric_values[IDX_MAX_Y] * mp_units::si::unit_symbols::cm;
            mission.min_height = numeric_values[IDX_MIN_H] * mp_units::si::unit_symbols::cm;
            mission.max_height = numeric_values[IDX_MAX_H] * mp_units::si::unit_symbols::cm;
            mission.has_bounds = true;
        }
        if (numeric_values.size() >= NUMERIC_START_COUNT) {
            mission.start_position = {
                numeric_values[IDX_START_X]     * mp_units::si::unit_symbols::cm,
                numeric_values[IDX_START_Y]     * mp_units::si::unit_symbols::cm,
                numeric_values[IDX_START_Z]     * mp_units::si::unit_symbols::cm
            };
            has_start_x = has_start_y = has_start_height = true;

            if (numeric_values.size() >= NUMERIC_TOTAL_COUNT) {
                mission.start_orientation = numeric_values[IDX_START_ANGLE] * mp_units::si::unit_symbols::deg;
                has_start_angle = true;
            }
        } else {
            errors += "Numeric mission_config is missing start position values.\n";
        }
    }

    if (mission.has_bounds) {
        double min_x = mission.min_x.numerical_value_in(mp_units::si::unit_symbols::cm);
        double max_x = mission.max_x.numerical_value_in(mp_units::si::unit_symbols::cm);
        double min_y = mission.min_y.numerical_value_in(mp_units::si::unit_symbols::cm);
        double max_y = mission.max_y.numerical_value_in(mp_units::si::unit_symbols::cm);
        double min_z = mission.min_height.numerical_value_in(mp_units::si::unit_symbols::cm);
        double max_z = mission.max_height.numerical_value_in(mp_units::si::unit_symbols::cm);
        if (min_x > max_x || min_y > max_y || min_z > max_z) {
            errors += "Invalid mission bounds; ignoring mission bounds.\n";
            mission.has_bounds = false;
        }
    }

    if (!has_start_x || !has_start_y || !has_start_height) {
        // A centered start is safer than the origin when mission bounds are known.
        if (mission.has_bounds) {
            double min_x = mission.min_x.numerical_value_in(mp_units::si::unit_symbols::cm);
            double max_x = mission.max_x.numerical_value_in(mp_units::si::unit_symbols::cm);
            double min_y = mission.min_y.numerical_value_in(mp_units::si::unit_symbols::cm);
            double max_y = mission.max_y.numerical_value_in(mp_units::si::unit_symbols::cm);
            double min_z = mission.min_height.numerical_value_in(mp_units::si::unit_symbols::cm);
            double max_z = mission.max_height.numerical_value_in(mp_units::si::unit_symbols::cm);
            if (!has_start_x)      { mission.start_position.x = ((min_x + max_x) / MISSION_CENTER_DIVISOR) * mp_units::si::unit_symbols::cm; }
            if (!has_start_y)      { mission.start_position.y = ((min_y + max_y) / MISSION_CENTER_DIVISOR) * mp_units::si::unit_symbols::cm; }
            if (!has_start_height) { mission.start_position.z = ((min_z + max_z) / MISSION_CENTER_DIVISOR) * mp_units::si::unit_symbols::cm; }
            errors += "Missing mission start position; using mission bounds center for missing values.\n";
        } else {
            errors += "Missing mission start position; using origin for missing values.\n";
        }
    }
    if (!has_start_angle) {
        mission.start_orientation = 0.0 * mp_units::si::unit_symbols::deg;
    }

    return mission;
}

Map3D parse_map_input(const std::string& filename, std::string& errors) {
    std::ifstream file(filename);
    if (!file) {
        errors += "Cannot open " + filename + "\n";
        return Map3D(DEFAULT_MAP_DIM, DEFAULT_MAP_DIM, DEFAULT_MAP_DIM);
    }

    size_t width, height, depth;
    file >> width >> height >> depth;
    if (file.fail()) {
        errors += "Invalid map dimensions\n";
        return Map3D(DEFAULT_MAP_DIM, DEFAULT_MAP_DIM, DEFAULT_MAP_DIM);
    }

    Map3D map(width, height, depth);
    // Missing or unsupported cells stay UNKNOWN so the rest of the file can still be used.
    for (size_t z = 0; z < depth; ++z) {
        for (size_t y = 0; y < height; ++y) {
            for (size_t x = 0; x < width; ++x) {
                int val;
                file >> val;
                if (file.fail()) {
                    errors += "Invalid map value at " + std::to_string(x) + "," +
                              std::to_string(y) + "," + std::to_string(z) + "\n";
                    val = Map3D::UNKNOWN;
                } else if (val != Map3D::FREE && val != Map3D::OCCUPIED) {
                    errors += "Unsupported map value at " + std::to_string(x) + "," +
                              std::to_string(y) + "," + std::to_string(z) + "; using UNKNOWN\n";
                    val = Map3D::UNKNOWN;
                }
                map.set(x, y, z, val);
            }
        }
    }
    return map;
}
