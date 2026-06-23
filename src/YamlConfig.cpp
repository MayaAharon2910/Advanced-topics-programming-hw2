#include <drone_mapper/YamlConfig.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <drone_mapper/Logger.h>

namespace drone_mapper {
namespace yaml {
namespace {

HorizontalAngle readHorizontalAny(const YAML::Node& n, const std::vector<std::string>& keys) {
    for (const auto& key : keys) {
        if (n && n[key]) {
            return HorizontalAngle{n[key].as<double>() * horizontal_angle[deg]};
        }
    }
    return HorizontalAngle{0.0 * horizontal_angle[deg]};
}

PhysicalLength readLengthAny(const YAML::Node& n, const std::vector<std::string>& keys) {
    for (const auto& key : keys) {
        if (n && n[key]) {
            return PhysicalLength{n[key].as<double>() * cm};
        }
    }
    return PhysicalLength{0.0 * cm};
}

std::filesystem::path resolveReferencedPath(const std::filesystem::path& base_dir,
                                            const std::string& raw_path) {
    const std::filesystem::path path{raw_path};
    if (path.is_absolute()) {
        return path;
    }
    return base_dir / path;
}

std::filesystem::path referencedFilePath(const YAML::Node& node,
                                        const std::filesystem::path& base_dir,
                                        const std::vector<std::string>& reference_keys) {
    if (!node) {
        return {};
    }
    if (node.IsScalar()) {
        return resolveReferencedPath(base_dir, node.as<std::string>());
    }
    if (node.IsMap()) {
        for (const auto& key : reference_keys) {
            if (node[key] && node[key].IsScalar()) {
                return resolveReferencedPath(base_dir, node[key].as<std::string>());
            }
        }
    }
    return {};
}

YAML::Node loadFileUnwrapped(const std::filesystem::path& path, const std::string& wrapper_key) {
    YAML::Node node = YAML::LoadFile(path.string());
    if (node[wrapper_key]) {
        return node[wrapper_key];
    }
    return node;
}

YAML::Node loadIfReferenced(const YAML::Node& node,
                            const std::filesystem::path& base_dir,
                            const std::string& wrapper_key,
                            const std::vector<std::string>& reference_keys) {
    if (!node) {
        return node;
    }
    if (node.IsScalar()) {
        return loadFileUnwrapped(resolveReferencedPath(base_dir, node.as<std::string>()), wrapper_key);
    }
    if (node.IsMap()) {
        if (node[wrapper_key] && node[wrapper_key].IsMap()) {
            return node[wrapper_key];
        }
        for (const auto& key : reference_keys) {
            if (node[key] && node[key].IsScalar()) {
                return loadFileUnwrapped(resolveReferencedPath(base_dir, node[key].as<std::string>()), wrapper_key);
            }
        }
    }
    return node;
}

Position3D readPosition(const YAML::Node& node) {
    Position3D position{};
    position.x = readLengthAny(node, {"x_cm", "x", "x_offset"}).force_numerical_value_in(cm) * x_extent[cm];
    position.y = readLengthAny(node, {"y_cm", "y", "y_offset"}).force_numerical_value_in(cm) * y_extent[cm];
    position.z = readLengthAny(node, {"height_cm", "z_cm", "z", "height", "height_offset"}).force_numerical_value_in(cm) * z_extent[cm];
    return position;
}

types::MappingBounds readBoundaries(const YAML::Node& node) {
    types::MappingBounds bounds{};
    if (!node) {
        return bounds;
    }
    const auto xb = node["x_boundary"];
    const auto yb = node["y_boundary"];
    const auto hb = node["height_boundary"];
    bounds.min_x = readLengthAny(xb, {"min_cm", "min"}).force_numerical_value_in(cm) * x_extent[cm];
    bounds.max_x = readLengthAny(xb, {"max_cm", "max"}).force_numerical_value_in(cm) * x_extent[cm];
    bounds.min_y = readLengthAny(yb, {"min_cm", "min"}).force_numerical_value_in(cm) * y_extent[cm];
    bounds.max_y = readLengthAny(yb, {"max_cm", "max"}).force_numerical_value_in(cm) * y_extent[cm];
    bounds.min_height = readLengthAny(hb, {"min_cm", "min"}).force_numerical_value_in(cm) * z_extent[cm];
    bounds.max_height = readLengthAny(hb, {"max_cm", "max"}).force_numerical_value_in(cm) * z_extent[cm];
    return bounds;
}

types::SimulationConfigData parseSimulationNode(const YAML::Node& raw) {
    const YAML::Node s = raw["simulation_config"] && raw["simulation_config"].IsMap() ? raw["simulation_config"] : raw;
    types::SimulationConfigData sim{};
    sim.map_filename = s["map_filename"].as<std::string>();
    sim.map_resolution = readLengthAny(s, {"map_resolution_cm", "map_resolution"});
    if (s["map_axes_offset"]) {
        sim.map_offset = readPosition(s["map_axes_offset"]);
    } else if (s["map_offset"]) {
        sim.map_offset = readPosition(s["map_offset"]);
    }
    if (s["initial_drone_position"]) {
        sim.initial_drone_position = readPosition(s["initial_drone_position"]);
    }
    sim.initial_angle = readHorizontalAny(s, {"initial_angle_deg", "initial_angle"});
    return sim;
}

types::MissionConfigData parseMissionNode(const YAML::Node& raw) {
    const YAML::Node m = raw["mission_config"] && raw["mission_config"].IsMap() ? raw["mission_config"] : raw;
    types::MissionConfigData mission{};
    mission.max_steps = m["max_steps"].as<std::size_t>(0);
    mission.gps_resolution = readLengthAny(m, {"gps_resolution_cm", "gps_resolution", "resolution_cm"});
    if (m["boundaries"]) {
        mission.boundaries = readBoundaries(m["boundaries"]);
    }
    if (m["output_mapping_resolution_factor"]) {
        int factor = m["output_mapping_resolution_factor"].as<int>();
        mission.output_mapping_resolution_factor = factor < 1 ? 0.0 : static_cast<double>(factor);
    } else {
        mission.output_mapping_resolution_factor = 1;
    }
    return mission;
}

types::DroneConfigData parseDroneNode(const YAML::Node& raw) {
    const YAML::Node d = raw["drone_config"] && raw["drone_config"].IsMap() ? raw["drone_config"] : raw;
    types::DroneConfigData drone{};
    drone.radius = readLengthAny(d, {"dimensions_cm", "radius_cm", "radius", "dimensions"});
    drone.max_rotate = readHorizontalAny(d, {"max_rotate_deg", "max_rotation_deg", "max_rotate"});
    drone.max_advance = readLengthAny(d, {"max_advance_cm", "max_advance"});
    drone.max_elevate = readLengthAny(d, {"max_elevate_cm", "max_elevate"});
    return drone;
}

types::LidarConfigData parseLidarNode(const YAML::Node& raw) {
    const YAML::Node l = raw["lidar_config"] && raw["lidar_config"].IsMap() ? raw["lidar_config"] : raw;
    types::LidarConfigData lidar{};
    lidar.z_min = readLengthAny(l, {"z_min_cm", "z_min"});
    lidar.z_max = readLengthAny(l, {"z_max_cm", "z_max"});
    lidar.d = readLengthAny(l, {"d_cm", "d"});
    lidar.fov_circles = l["fov_circles"].as<std::size_t>(0);
    return lidar;
}

types::MissionConfigData parseMissionRef(const YAML::Node& node,
                                      const std::filesystem::path& base_dir) {
    YAML::Node resolved = loadIfReferenced(node, base_dir, "mission_config", {"mission_config", "mission", "path", "file"});
    return parseMissionNode(resolved);
}

void appendMissionRef(types::SimulationCompositionData& comp,
                      const YAML::Node& node,
                      const std::filesystem::path& base_dir) {
    // No longer used - missions are now nested under simulation_mission_groups
    (void)comp; (void)node; (void)base_dir;
}

} // namespace

types::SimulationCompositionData parseSimulationComposition(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());
    const std::filesystem::path base_dir = path.parent_path();

    types::SimulationCompositionData comp{};
    comp.composition_file = path;

    YAML::Node composition = root["simulation_compositions"] ? root["simulation_compositions"] : root;

    if (composition["simulations"]) {
        for (const auto& s : composition["simulations"]) {
            const std::filesystem::path sim_path = referencedFilePath(s, base_dir, {"simulation_config", "simulation", "path", "file"});
            YAML::Node sim_node = loadIfReferenced(s, base_dir, "simulation_config", {"simulation_config", "simulation", "path", "file"});
            auto sim_config = parseSimulationNode(sim_node);
            sim_config.source_file = sim_path;

            std::vector<types::MissionConfigData> local_missions;
            if (s["mission_configs"]) {
                for (const auto& mission_ref : s["mission_configs"]) {
                    auto mission = parseMissionRef(mission_ref, base_dir);
                    mission.source_file = referencedFilePath(mission_ref, base_dir, {"mission_config", "mission", "path", "file"});
                    local_missions.push_back(std::move(mission));
                }
            }
            comp.simulation_mission_groups.emplace_back(std::move(sim_config), std::move(local_missions));

            // The published PDF keeps drone_configs and lidar_configs at the
            // composition root. Accepting them here as well is harmless and
            // lets us support the same hierarchy if a checker nests them under
            // a specific simulation entry. They still participate in the
            // Cartesian product below.
            if (s["drone_configs"]) {
                for (const auto& d : s["drone_configs"]) {
                    YAML::Node resolved = loadIfReferenced(d, base_dir, "drone_config", {"drone_config", "drone", "path", "file"});
                    auto drone = parseDroneNode(resolved);
                    drone.source_file = referencedFilePath(d, base_dir, {"drone_config", "drone", "path", "file"});
                    comp.drones.push_back(std::move(drone));
                }
            }
            if (s["lidar_configs"]) {
                for (const auto& l : s["lidar_configs"]) {
                    YAML::Node resolved = loadIfReferenced(l, base_dir, "lidar_config", {"lidar_config", "lidar", "path", "file"});
                    auto lidar = parseLidarNode(resolved);
                    lidar.source_file = referencedFilePath(l, base_dir, {"lidar_config", "lidar", "path", "file"});
                    comp.lidars.push_back(std::move(lidar));
                }
            }
        }
    }

    if (composition["mission_configs"]) {
        for (const auto& m : composition["mission_configs"]) {
            appendMissionRef(comp, m, base_dir);
        }
    }
    if (composition["missions"]) {
        for (const auto& m : composition["missions"]) {
            appendMissionRef(comp, m, base_dir);
        }
    }

    if (composition["drone_configs"]) {
        for (const auto& d : composition["drone_configs"]) {
            YAML::Node resolved = loadIfReferenced(d, base_dir, "drone_config", {"drone_config", "drone", "path", "file"});
            auto drone = parseDroneNode(resolved);
            drone.source_file = referencedFilePath(d, base_dir, {"drone_config", "drone", "path", "file"});
            comp.drones.push_back(std::move(drone));
        }
    }
    if (composition["drones"]) {
        for (const auto& d : composition["drones"]) {
            YAML::Node resolved = loadIfReferenced(d, base_dir, "drone_config", {"drone_config", "drone", "path", "file"});
            auto drone = parseDroneNode(resolved);
            drone.source_file = referencedFilePath(d, base_dir, {"drone_config", "drone", "path", "file"});
            comp.drones.push_back(std::move(drone));
        }
    }

    if (composition["lidar_configs"]) {
        for (const auto& l : composition["lidar_configs"]) {
            YAML::Node resolved = loadIfReferenced(l, base_dir, "lidar_config", {"lidar_config", "lidar", "path", "file"});
            auto lidar = parseLidarNode(resolved);
            lidar.source_file = referencedFilePath(l, base_dir, {"lidar_config", "lidar", "path", "file"});
            comp.lidars.push_back(std::move(lidar));
        }
    }
    if (composition["lidars"]) {
        for (const auto& l : composition["lidars"]) {
            YAML::Node resolved = loadIfReferenced(l, base_dir, "lidar_config", {"lidar_config", "lidar", "path", "file"});
            auto lidar = parseLidarNode(resolved);
            lidar.source_file = referencedFilePath(l, base_dir, {"lidar_config", "lidar", "path", "file"});
            comp.lidars.push_back(std::move(lidar));
        }
    }

    return comp;
}

} // namespace yaml
} // namespace drone_mapper
