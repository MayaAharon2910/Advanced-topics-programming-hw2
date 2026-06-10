#include "MockLidarSensor.h"
#include "MockPositionSensor.h"
#include "MockMovementDriver.h"
#include "FileUtils.h"
#include "Drone.h"
#include "MapExport.h"
#include "Logger.h"
#include "SimulationConfig.h"
#include "ILidarSensor.h"
#include "IMovementDriver.h"
#include "IPositionSensor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>

namespace {

constexpr int    SUCCESS_EXIT_CODE          = 0;
constexpr int    FAILURE_EXIT_CODE          = 1;
constexpr int    MAX_ARGUMENT_COUNT         = 2;
constexpr int    INPUT_PATH_ARGUMENT_INDEX  = 1;

constexpr char   DEFAULT_INPUT_OUTPUT_PATH[]= ".";
constexpr char   INPUT_ERRORS_FILENAME[]    = "input_errors.txt";
constexpr char   DRONE_CONFIG_FILENAME[]    = "drone_config.txt";
constexpr char   MISSION_CONFIG_FILENAME[]  = "mission_config.txt";
constexpr char   MAP_INPUT_FILENAME[]       = "map_input.txt";
constexpr char   MAP_OUTPUT_FILENAME[]      = "map_output.txt";
constexpr char   SCORE_FILENAME[]           = "score.txt";
constexpr char   SIMULATION_LOG_FILENAME[]    = "simulation_log.txt";
constexpr char   SIMULATION_CONFIG_FILENAME[] = "simulation_config.txt";

constexpr size_t MAX_MAP_VOXELS             = 10'000'000;
constexpr int    MAX_SIMULATION_ITERATIONS  = 50'000;
constexpr int    FAILED_SIMULATION_SCORE    = 0;
constexpr int    ITERATION_PRINT_INTERVAL   = 1000;

std::filesystem::path resolveFilePath(const std::filesystem::path& base,
                                      const std::string& filename) {
    return base / filename;
}

void updateInputErrorsFile(const std::filesystem::path& input_output_path,
                           const std::string& recovered_errors) {
    auto errors_path = resolveFilePath(input_output_path, INPUT_ERRORS_FILENAME);

    if (recovered_errors.empty()) {
        // Avoid leaving stale warnings beside a clean run.
        std::error_code ignored;
        std::filesystem::remove(errors_path, ignored);
        return;
    }

    std::ofstream errors_file(errors_path, std::ios::out | std::ios::trunc);
    if (errors_file) {
        errors_file << recovered_errors;
    }
}

struct MapStatistics {
    size_t free_cells = 0;
    size_t occupied_cells = 0;
    size_t unknown_cells = 0;
    size_t out_of_bounds_cells = 0;
    size_t correct_free = 0;
    size_t wrong_free = 0;
    size_t correct_occupied = 0;
    size_t wrong_occupied = 0;
};

MapStatistics countMapVoxels(const Map3D& map, const Map3D& ground_truth_map) {
    MapStatistics stats;
    for (size_t z = 0; z < map.depth(); ++z) {
        for (size_t y = 0; y < map.height(); ++y) {
            for (size_t x = 0; x < map.width(); ++x) {
                int state = map.at(x, y, z);
                if (state == Map3D::FREE) {
                    ++stats.free_cells;
                } else if (state == Map3D::OCCUPIED) {
                    ++stats.occupied_cells;
                } else if (state == Map3D::UNKNOWN) {
                    ++stats.unknown_cells;
                } else if (state == Map3D::OUT_OF_BOUNDS) {
                    ++stats.out_of_bounds_cells;
                }

                int expected = ground_truth_map.at(x, y, z);
                if (state == Map3D::FREE) {
                    if (expected == Map3D::FREE) {
                        ++stats.correct_free;
                    } else {
                        ++stats.wrong_free;
                    }
                } else if (state == Map3D::OCCUPIED) {
                    if (expected == Map3D::OCCUPIED) {
                        ++stats.correct_occupied;
                    } else {
                        ++stats.wrong_occupied;
                    }
                }
            }
        }
    }
    return stats;
}

bool mapDimensionsAreReasonable(size_t width, size_t height, size_t depth) {
    if (width == 0 || height == 0 || depth == 0) {
        return false;
    }

    // Guard the allocation before Map3D multiplies the dimensions.
    if (width > MAX_MAP_VOXELS / height) {
        return false;
    }

    return (width * height) <= MAX_MAP_VOXELS / depth;
}

template <typename Quantity>
int roundedCentimeters(const Quantity& value) {
    return static_cast<int>(
        std::round(value.numerical_value_in(mp_units::si::unit_symbols::cm)));
}

void applyMissionBounds(Map3D& map, const MissionConfig& mission_config) {
    if (!mission_config.has_bounds) {
        return;
    }

    map.setMissionBounds(
        roundedCentimeters(mission_config.min_x),
        roundedCentimeters(mission_config.max_x),
        roundedCentimeters(mission_config.min_y),
        roundedCentimeters(mission_config.max_y),
        roundedCentimeters(mission_config.min_height),
        roundedCentimeters(mission_config.max_height)
    );
}

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

std::string formatTimestamp(const TimePoint& start) {
    using namespace std::chrono;

    long long ms   = duration_cast<milliseconds>(Clock::now() - start).count();
    long long msec = ms % 1000;
    long long sec  = (ms / 1'000) % 60;
    long long min  = (ms / 60'000) % 60;
    long long hr   = ms / 3'600'000;

    std::ostringstream out;
    out << '[' << std::setfill('0')
        << std::setw(2) << hr << ':'
        << std::setw(2) << min << ':'
        << std::setw(2) << sec << '.'
        << std::setw(3) << msec << ']';

    return out.str();
}

std::string posStr(const StrongPosition3D& position) {
    std::ostringstream out;

    out << "x=" << roundedCentimeters(position.x)
        << " y=" << roundedCentimeters(position.y)
        << " z=" << roundedCentimeters(position.z);

    return out.str();
}

std::string signedDouble(double value, int precision = 1) {
    std::ostringstream out;

    out << std::fixed << std::setprecision(precision);
    if (value >= 0.0) {
        out << '+';
    }
    out << value;

    return out.str();
}

} // namespace

int main(int argc, char** argv) {
    if (argc > MAX_ARGUMENT_COUNT) {
        std::cout << "Usage: drone_mapper [<input_output_files_path>]\n";
        return FAILURE_EXIT_CODE;
    }

    std::filesystem::path input_output_path = DEFAULT_INPUT_OUTPUT_PATH;

    if (argc == MAX_ARGUMENT_COUNT && argv[INPUT_PATH_ARGUMENT_INDEX][0] != '\0') {
        input_output_path = argv[INPUT_PATH_ARGUMENT_INDEX];
    }

    if (!std::filesystem::exists(input_output_path) ||
        !std::filesystem::is_directory(input_output_path)) {
        std::cout << "Unrecoverable error: input/output path is not a directory: "
                  << input_output_path << '\n';
        return FAILURE_EXIT_CODE;
    }

    auto simulation_config_path =
        resolveFilePath(input_output_path, SIMULATION_CONFIG_FILENAME);
    SimulationConfig sim_cfg = parseSimulationConfig(simulation_config_path.string());

    auto simulation_log_path =
        resolveFilePath(input_output_path, SIMULATION_LOG_FILENAME);

    Logger logger(simulation_log_path.string(), sim_cfg.log_level, sim_cfg.log_enabled);

    auto logToFile = [&](const std::string& line) {
        logger.info(line);
    };

    auto logDebug = [&](const std::string& line) {
        logger.debug(line);
    };

    auto logWarning = [&](const std::string& line) {
        logger.warning(line);
    };

    auto logError = [&](const std::string& line) {
        logger.error(line);
    };

    auto config_path  = resolveFilePath(input_output_path, DRONE_CONFIG_FILENAME);
    auto mission_path = resolveFilePath(input_output_path, MISSION_CONFIG_FILENAME);
    auto map_path     = resolveFilePath(input_output_path, MAP_INPUT_FILENAME);

    std::string recovered_errors;

    // Recoverable parser issues are accumulated and written once after all inputs are read.
    std::string config_errors;
    Config config = parse_drone_config(config_path.string(), config_errors);

    if (!config_errors.empty()) {
        std::cout << "Warning: config file issues:\n" << config_errors << '\n';
        logWarning("Warning: config file issues: " + config_errors);
        recovered_errors += config_errors;
    }

    std::string mission_errors;
    MissionConfig mission_config =
        parse_mission_config(mission_path.string(), mission_errors);

    if (!mission_errors.empty()) {
        std::cout << "Warning: mission config issues:\n" << mission_errors << '\n';
        logWarning("Warning: mission config issues: " + mission_errors);
        recovered_errors += mission_errors;
    }

    if (!std::filesystem::exists(map_path)) {
        updateInputErrorsFile(input_output_path, recovered_errors);

        std::cout << "Unrecoverable error: missing mandatory map file "
                  << map_path << '\n';

        logError("Unrecoverable error: missing mandatory map file " + map_path.string());
        return FAILURE_EXIT_CODE;
    }

    size_t map_width = 0;
    size_t map_height = 0;
    size_t map_depth = 0;

    if (!getMapDimensions(map_path.string(), map_width, map_height, map_depth) ||
        !mapDimensionsAreReasonable(map_width, map_height, map_depth)) {
        updateInputErrorsFile(input_output_path, recovered_errors);

        std::cout << "Unrecoverable error: invalid map dimensions in "
                  << map_path << '\n';

        logError("Unrecoverable error: invalid map dimensions in " + map_path.string());
        return FAILURE_EXIT_CODE;
    }

    std::string map_errors;
    Map3D ground_truth_map = parse_map_input(map_path.string(), map_errors);

    if (!map_errors.empty()) {
        std::cout << "Warning: map input issues:\n" << map_errors << '\n';
        logWarning("Warning: map input issues: " + map_errors);
        recovered_errors += map_errors;
    }

    // Detect supported map resolution for this simulation run.
    // Exercise 1: simulator map is a voxel grid with 1cm resolution stored as integer voxels,
    // so the supported number of decimal digits after the point is 0 for XY and height.
    int map_resolution_xy_digits = 0;
    int map_resolution_height_digits = 0;

    // Compare requested mission resolution with the map-supported resolution.
    if (mission_config.resolution_xy_digits != map_resolution_xy_digits ||
        mission_config.resolution_height_digits != map_resolution_height_digits) {
        std::ostringstream msg;
        msg << "Resolution mismatch detected:\n";
        msg << "  Supported (map) resolution: XY digits=" << map_resolution_xy_digits
            << ", height digits=" << map_resolution_height_digits << "\n";
        msg << "  Note: the supported resolution depends on the simulation map provided to the sensors.\n";
        msg << "  Requested mission resolution: XY digits=" << mission_config.resolution_xy_digits
            << ", height digits=" << mission_config.resolution_height_digits << "\n";
        msg << "  Action: continuing the mapping using the supported map resolution (ignoring the requested resolution).\n";

        recovered_errors += msg.str();

        logWarning("Resolution mismatch: requested mission resolution differs from map resolution; using map resolution.");
    }

    updateInputErrorsFile(input_output_path, recovered_errors);
    applyMissionBounds(ground_truth_map, mission_config);

    StrongPosition3D start_pos = mission_config.start_position;
    auto start_orient = mission_config.start_orientation;

    // Conservative sphere radius: diameter = max of all min-pass dimensions.
    auto max_pass = config.min_pass_width_cm;
    if (config.min_pass_length_cm > max_pass) max_pass = config.min_pass_length_cm;
    if (config.min_pass_height_cm > max_pass) max_pass = config.min_pass_height_cm;
    const double sphere_radius_cm =
        max_pass.numerical_value_in(mp_units::si::unit_symbols::cm) / 2.0;

    MockLidarSensor lidar(
        ground_truth_map,
        config.lidar_z_min_cm,
        config.lidar_z_max_cm,
        config.lidar_fovc,
        config.lidar_d_cm
    );

    MockPositionSensor mock_position_sensor(start_pos, start_orient);
    MockMovementDriver mock_driver(start_pos,
                                   start_orient,
                                   ground_truth_map,
                                   sphere_radius_cm,
                                   config.max_rotation,
                                   config.max_advance,
                                   config.max_elevation);

    // The simulator owns the mock objects, but all command execution is routed
    // through the abstract interfaces. This keeps the drone independent from
    // the concrete mock classes and makes the mocks transparent to it.
    ILidarSensor& lidar_interface = lidar;
    IPositionSensor& position_sensor_interface = mock_position_sensor;
    IMovementDriver& driver_interface = mock_driver;

    // Validate the full drone sphere at the starting position before the mission starts.
    // This rejects starts that are out of bounds, inside an obstacle, or too close to an obstacle.
    if (!mock_driver.canDroneOccupy(start_pos)) {
        std::cout << "Unrecoverable error: drone cannot safely occupy the start position.\n";
        logError("Unrecoverable error: drone cannot safely occupy the start position.");
        return FAILURE_EXIT_CODE;
    }

    Drone drone(
        ground_truth_map.width(),
        ground_truth_map.height(),
        ground_truth_map.depth(),
        config,
        mission_config,
        lidar_interface,
        position_sensor_interface,
        driver_interface
    );

    const double z_max_cm =
        config.lidar_z_max_cm.numerical_value_in(mp_units::si::unit_symbols::cm);

    const TimePoint simulation_start = Clock::now();

    auto timestamp = [&]() {
        return formatTimestamp(simulation_start);
    };

    const size_t total_voxels = map_width * map_height * map_depth;

    std::cout << "Starting drone mapping simulation...\n";
    if (sim_cfg.log_enabled) {
        std::cout << "Simulation log: "
                  << std::filesystem::absolute(simulation_log_path) << '\n';
        std::cout << "Log level: "
                  << (sim_cfg.debug_mode ? "DEBUG" :
                      sim_cfg.log_level == LogLevel::WARNING ? "WARNING" :
                      sim_cfg.log_level == LogLevel::ERROR   ? "ERROR"   : "INFO")
                  << '\n';
    } else {
        std::cout << "Logging disabled via simulation_config.txt\n";
    }

    logToFile("Drone Mapper Simulation Log");
    logToFile("===========================");
    logToFile("");

    {
        std::ostringstream line;
        line << timestamp() << " [INIT] Working directory: "
             << std::filesystem::absolute(input_output_path);
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [MAP] Dimensions: "
             << map_width << " x " << map_height << " x " << map_depth
             << " (" << total_voxels << " voxels)";
        logToFile(line.str());
    }

    if (mission_config.has_bounds) {
        std::ostringstream line;
        line << timestamp() << " [MAP] Mission bounds: "
             << "x=[" << roundedCentimeters(mission_config.min_x)
             << "," << roundedCentimeters(mission_config.max_x) << "] "
             << "y=[" << roundedCentimeters(mission_config.min_y)
             << "," << roundedCentimeters(mission_config.max_y) << "] "
             << "z=[" << roundedCentimeters(mission_config.min_height)
             << "," << roundedCentimeters(mission_config.max_height) << "]";
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [INIT] Start pose: "
             << posStr(start_pos)
             << " heading="
             << std::fixed << std::setprecision(1)
             << start_orient.numerical_value_in(mp_units::si::unit_symbols::deg)
             << " degrees";
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [INIT] LiDAR: "
             << "z_min="
             << config.lidar_z_min_cm.numerical_value_in(mp_units::si::unit_symbols::cm)
             << "cm "
             << "z_max=" << z_max_cm << "cm "
             << "fovc=" << config.lidar_fovc << " "
             << "d="
             << config.lidar_d_cm.numerical_value_in(mp_units::si::unit_symbols::cm)
             << "cm";
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [INIT] Iteration limit: "
             << MAX_SIMULATION_ITERATIONS;
        logToFile(line.str());
    }

    logToFile("");

    bool finished = false;
    bool simulation_failed = false;
    int iterations = 0;

    // The main loop is intentionally command-driven: the drone decides and executes
    // through its injected interfaces; main only observes the returned action.
    while (!finished && iterations < MAX_SIMULATION_ITERATIONS) {
        ++iterations;

        if (iterations % ITERATION_PRINT_INTERVAL == 0) {
            std::cout << "Iteration " << iterations << "...\n";
        }

        DroneCommand command = drone.executeNextAction();

        if (!command.succeeded) {
            simulation_failed = true;
            finished = true;

            std::ostringstream line;
            line << timestamp() << " [COLLISION] #" << iterations;
            if (command.type == DroneCommandType::Rotate) {
                std::cout << "Collision detected while rotating.\n";
                line << " command=Rotate angle=" << signedDouble(command.angle_deg);
            } else if (command.type == DroneCommandType::Advance) {
                std::cout << "Collision detected while advancing.\n";
                line << " command=Advance distance=" << signedDouble(command.value_cm);
            } else if (command.type == DroneCommandType::Elevate) {
                std::cout << "Collision detected while elevating.\n";
                line << " command=Elevate height=" << signedDouble(command.value_cm);
            } else {
                std::cout << "Simulation failed while executing a drone command.\n";
                line << " command=SensorOrDriverFailure";
            }
            logToFile(line.str());
            break;
        }

        switch (command.type) {
            case DroneCommandType::GetLocation: {
                const auto& position = drone.getPosition();
                const double heading =
                    drone.getOrientation()
                        .numerical_value_in(mp_units::si::unit_symbols::deg);

                std::ostringstream line;
                line << timestamp() << " [LOC] #"
                     << iterations
                     << " position=" << posStr(position)
                     << " heading=" << std::fixed << std::setprecision(1)
                     << heading;
                logToFile(line.str());
            } break;

            case DroneCommandType::Scan: {
                std::ostringstream line;
                line << timestamp() << " [SCAN] #"
                     << iterations
                     << " az=" << std::fixed << std::setprecision(1)
                     << command.executed_azimuth_deg
                     << " elevation=" << command.executed_elevation_deg
                     << " beams=" << command.scan_beams
                     << " hits=" << command.scan_hits
                     << " open=" << command.scan_open
                     << " position=" << posStr(drone.getPosition());
                logDebug(line.str());
            } break;

            case DroneCommandType::Rotate: {
                const double heading =
                    drone.getOrientation()
                        .numerical_value_in(mp_units::si::unit_symbols::deg);

                std::ostringstream line;
                line << timestamp() << " [ROTATE] #"
                     << iterations
                     << " angle=" << signedDouble(command.angle_deg)
                     << " heading=" << std::fixed << std::setprecision(1)
                     << heading
                     << " position=" << posStr(drone.getPosition());
                logToFile(line.str());
            } break;

            case DroneCommandType::Advance: {
                std::ostringstream line;
                line << timestamp() << " [ADVANCE] #"
                     << iterations
                     << " distance=" << signedDouble(command.value_cm)
                     << "cm position=" << posStr(drone.getPosition());
                logToFile(line.str());
            } break;

            case DroneCommandType::Elevate: {
                std::ostringstream line;
                line << timestamp() << " [ELEVATE] #"
                     << iterations
                     << " height=" << signedDouble(command.value_cm)
                     << "cm position=" << posStr(drone.getPosition());
                logToFile(line.str());
            } break;

            case DroneCommandType::Finished: {
                std::ostringstream line;
                line << timestamp()
                     << " [DONE] Exploration complete; no more frontiers reachable.";
                logToFile(line.str());

                finished = true;
            } break;
        }
    }

    if (iterations >= MAX_SIMULATION_ITERATIONS && !finished) {
        std::cout << "Warning: simulation timeout after "
                  << MAX_SIMULATION_ITERATIONS
                  << " iterations; stopping main loop.\n";

        std::ostringstream line;
        line << timestamp() << " [WARN] Iteration limit reached after "
             << MAX_SIMULATION_ITERATIONS << " iterations.";
        logToFile(line.str());

        finished = true;
    }

    auto map_output_path =
        resolveFilePath(input_output_path, MAP_OUTPUT_FILENAME);

    auto score_output_path =
        resolveFilePath(input_output_path, SCORE_FILENAME);

    std::cout << "Mapping completed.\n";
    std::cout << "Exporting map output and calculating final score.\n";

    bool export_ok = exportMapToFile(drone.getMap(), map_output_path.string());
    (void)export_ok;

    // Collisions override map accuracy because the simulated mission failed.
    int score = simulation_failed
                    ? FAILED_SIMULATION_SCORE
                    : calculateScore(drone.getMap(), ground_truth_map, start_pos);

    {
        std::ofstream score_file(score_output_path, std::ios::out | std::ios::trunc);
        score_file << "Score: " << score << "/100\n";
        score_file << "Final Position: (" << drone.getPosition().x << ", "
                   << drone.getPosition().y << ", "
                   << drone.getPosition().z << ")\n";
        score_file << "Final Orientation: " << drone.getOrientation() << "\n";
    }

    logToFile("");
    logToFile("Simulation results:");

    {
        std::ostringstream line;
        line << timestamp() << " [RESULT] Status: "
             << (simulation_failed ? "FAILED collision" : "Completed successfully");
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [RESULT] Iterations: "
             << iterations << " / " << MAX_SIMULATION_ITERATIONS;
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [RESULT] Final pose: "
             << posStr(drone.getPosition())
             << " heading="
             << std::fixed << std::setprecision(1)
             << drone.getOrientation()
                     .numerical_value_in(mp_units::si::unit_symbols::deg);
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [RESULT] Score: "
             << score << "/100";
        logToFile(line.str());
    }

    MapStatistics map_stats = countMapVoxels(drone.getMap(), ground_truth_map);

    logToFile("");
    logToFile("Final Map Statistics:");

    {
        std::ostringstream line;
        line << "Mapped FREE cells: " << map_stats.free_cells;
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << "Mapped OCCUPIED cells: " << map_stats.occupied_cells;
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << "UNKNOWN cells: " << map_stats.unknown_cells;
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << "OUT_OF_BOUNDS cells: " << map_stats.out_of_bounds_cells;
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << "correct_free: " << map_stats.correct_free;
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << "wrong_free: " << map_stats.wrong_free;
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << "correct_occupied: " << map_stats.correct_occupied;
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << "wrong_occupied: " << map_stats.wrong_occupied;
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << std::fixed << std::setprecision(2);
        double total_mapped = map_stats.free_cells + map_stats.occupied_cells;
        double total_cells = total_mapped + map_stats.unknown_cells;
        double mapped_percent = (total_cells > 0) ? (total_mapped / total_cells) * 100.0 : 0.0;
        line << "Mapped percentage: " << mapped_percent << "%";
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [OUTPUT] Map output: "
             << std::filesystem::absolute(map_output_path);
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [OUTPUT] Score file: "
             << std::filesystem::absolute(score_output_path);
        logToFile(line.str());
    }

    {
        std::ostringstream line;
        line << timestamp() << " [OUTPUT] Simulation log: "
             << std::filesystem::absolute(simulation_log_path);
        logToFile(line.str());
    }

    std::cout << "Final Score: " << score << "/100\n";

    return SUCCESS_EXIT_CODE;
}
