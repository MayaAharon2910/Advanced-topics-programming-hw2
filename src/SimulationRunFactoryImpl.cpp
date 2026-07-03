#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/Logger.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <memory>
#include <cmath>
#include <stdexcept>
#include <string>

namespace drone_mapper {
namespace {

std::shared_ptr<NpyArray> tryLoadNpy(const std::filesystem::path& candidate) {
    if (!std::filesystem::exists(candidate)) return nullptr;
    auto map = std::make_shared<NpyArray>();
    const char* error = map->LoadNPY(candidate.string().c_str());
    if (error != nullptr) return nullptr;
    return map;
}

// Tries to load a .npy file using a three-step fallback:
//   1. raw path relative to CWD
//   2. relative to composition directory
//   3. relative to simulation config directory
std::shared_ptr<NpyArray> loadNpyArray(const std::filesystem::path& raw_path,
                                        const std::filesystem::path& composition_dir = {},
                                        const std::filesystem::path& sim_dir = {}) {
    // 1. Try as given (CWD-relative or absolute)
    if (auto r = tryLoadNpy(raw_path)) return r;
    // 2. Try relative to composition file directory
    if (!composition_dir.empty()) {
        if (auto r = tryLoadNpy(composition_dir / raw_path)) return r;
    }
    // 3. Try relative to simulation file directory
    if (!sim_dir.empty()) {
        if (auto r = tryLoadNpy(sim_dir / raw_path)) return r;
    }
    throw std::runtime_error(
        std::string("Failed to load NPY file (tried CWD, composition dir, sim dir): ")
        + raw_path.string());
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    const types::MapConfig hidden_map_config{
        types::MappingBounds{},
        simulation.map_offset,
        simulation.map_resolution,
    };
    auto hidden_array = loadNpyArray(
        simulation.map_filename,
        simulation.source_file.empty()
            ? std::filesystem::current_path()
            : simulation.source_file.parent_path(),
        output_path.parent_path());
    const auto& hidden_shape = hidden_array->Shape();
    if (hidden_shape.size() != 3) {
        throw std::runtime_error("Hidden map NPY must be a 3D array.");
    }
    auto hidden_map = std::make_unique<Map3DImpl>(
        hidden_array,
        hidden_map_config);

    const auto mission_bounds = (mission.mission_bounds.min_x.force_numerical_value_in(cm) == 0.0 &&
                                 mission.mission_bounds.max_x.force_numerical_value_in(cm) == 0.0 &&
                                 mission.mission_bounds.min_y.force_numerical_value_in(cm) == 0.0 &&
                                 mission.mission_bounds.max_y.force_numerical_value_in(cm) == 0.0 &&
                                 mission.mission_bounds.min_height.force_numerical_value_in(cm) == 0.0 &&
                                 mission.mission_bounds.max_height.force_numerical_value_in(cm) == 0.0)
                                    ? hidden_map->getMapConfig().boundaries
                                    : mission.mission_bounds;
    const double requested_factor = mission.output_mapping_resolution_factor;
    PhysicalLength output_resolution = mission.gps_resolution;
    if (requested_factor >= 1.0) {
        output_resolution = (mission.gps_resolution.force_numerical_value_in(cm) / requested_factor) * cm;
    } else {
        Logger::logError(
            "OUTPUT_MAPPING_RESOLUTION_FACTOR_IGNORED_TOO_SMALL",
            "output_mapping_resolution_factor < 1; using gps_resolution as default output resolution");
    }

    const types::MapConfig output_map_config{
        mission_bounds,
        hidden_map_config.offset,
        output_resolution,
    };
    const double hidden_resolution_cm = simulation.map_resolution.force_numerical_value_in(cm);
    const double output_resolution_cm = output_resolution.force_numerical_value_in(cm);
    if (hidden_resolution_cm <= 0.0 || output_resolution_cm <= 0.0) {
        throw std::runtime_error("Map resolutions must be positive.");
    }
    const auto output_width = static_cast<size_t>(
        std::ceil(static_cast<double>(hidden_shape[0]) * hidden_resolution_cm / output_resolution_cm));
    const auto output_height = static_cast<size_t>(
        std::ceil(static_cast<double>(hidden_shape[1]) * hidden_resolution_cm / output_resolution_cm));
    const auto output_depth = static_cast<size_t>(
        std::ceil(static_cast<double>(hidden_shape[2]) * hidden_resolution_cm / output_resolution_cm));
    auto output_map = std::make_unique<Map3DImpl>(
        output_width,
        output_height,
        output_depth,
        output_map_config);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, mission_bounds, drone.radius);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    // Sync mission boundaries: MockMovement uses mission_bounds (derived from map if
    // mission.mission_bounds was all-zero). MappingAlgorithmImpl must use the same bounds
    // so its BFS never plans moves that MockMovement would reject.
    // We shrink max bounds by a small epsilon to prevent floating-point drift during
    // advance (heading imprecision causes slight off-axis drift that exceeds exact max).
    types::MissionConfigData synced_mission = mission;
    constexpr double kBoundaryEpsilon = 1.0; // cm — one full cell margin
    types::MappingBounds shrunk = mission_bounds;
    shrunk.max_x = (mission_bounds.max_x.force_numerical_value_in(cm) - kBoundaryEpsilon) * x_extent[cm];
    shrunk.max_y = (mission_bounds.max_y.force_numerical_value_in(cm) - kBoundaryEpsilon) * y_extent[cm];
    shrunk.max_height = (mission_bounds.max_height.force_numerical_value_in(cm) - kBoundaryEpsilon) * z_extent[cm];
    shrunk.min_x = (mission_bounds.min_x.force_numerical_value_in(cm) + kBoundaryEpsilon) * x_extent[cm];
    shrunk.min_y = (mission_bounds.min_y.force_numerical_value_in(cm) + kBoundaryEpsilon) * y_extent[cm];
    shrunk.min_height = (mission_bounds.min_height.force_numerical_value_in(cm) + kBoundaryEpsilon) * z_extent[cm];
    synced_mission.mission_bounds = shrunk;

    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(synced_mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        mission,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm);
    // Inject lidar config via setter — avoids touching any skeleton interface.
    drone_control->setLidarConfig(lidar);

    // Ensure output_results exists and place output map there.
    const std::filesystem::path output_results = output_path / "output_results";
    std::filesystem::create_directories(output_results);
    const std::filesystem::path output_map_file =
        output_results / ("output_map_" + std::to_string(next_run_index_++) + ".npy");
    auto mission_control = std::make_unique<MissionControlImpl>(
        mission,
        drone,
        *hidden_map,
        *output_map,
        *drone_control,
        output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        drone,
        lidar,
        output_map_file);
}

} // namespace drone_mapper
