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

std::shared_ptr<NpyArray> loadNpyArray(const std::filesystem::path& path) {
    auto map = std::make_shared<NpyArray>();
    const std::string path_string = path.string();
    const char* error = map->LoadNPY(path_string.c_str());
    if (error != nullptr) {
        throw std::runtime_error(std::string("Failed to load NPY file: ") + error);
    }
    return map;
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
    auto hidden_array = loadNpyArray(simulation.map_filename);
    const auto& hidden_shape = hidden_array->Shape();
    if (hidden_shape.size() != 3) {
        throw std::runtime_error("Hidden map NPY must be a 3D array.");
    }
    auto hidden_map = std::make_unique<Map3DImpl>(
        hidden_array,
        hidden_map_config);

    const auto mission_bounds = (mission.boundaries.min_x.force_numerical_value_in(cm) == 0.0 &&
                                 mission.boundaries.max_x.force_numerical_value_in(cm) == 0.0 &&
                                 mission.boundaries.min_y.force_numerical_value_in(cm) == 0.0 &&
                                 mission.boundaries.max_y.force_numerical_value_in(cm) == 0.0 &&
                                 mission.boundaries.min_height.force_numerical_value_in(cm) == 0.0 &&
                                 mission.boundaries.max_height.force_numerical_value_in(cm) == 0.0)
                                    ? hidden_map->getMapConfig().boundaries
                                    : mission.boundaries;
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
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        mission,
        lidar,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm);

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
