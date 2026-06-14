#include <drone_mapper/YamlConfig.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <drone_mapper/Logger.h>

namespace drone_mapper {
namespace yaml {

// Explicit namespace usage avoided per skeleton requirements.

static HorizontalAngle readHorizontal(const YAML::Node& n, const std::string& key) {
    if (!n || !n[key]) return HorizontalAngle{0.0 * mp_units::si::unit_symbols::deg};
    return HorizontalAngle{n[key].as<double>() * mp_units::si::unit_symbols::deg};
}

static PhysicalLength readLength(const YAML::Node& n, const std::string& key) {
    if (!n || !n[key]) return PhysicalLength{0.0 * mp_units::si::unit_symbols::cm};
    return PhysicalLength{n[key].as<double>() * mp_units::si::unit_symbols::cm};
}

static std::filesystem::path resolveReferencedPath(const std::filesystem::path& base_dir,
                                                   const std::string& raw_path) {
    const std::filesystem::path path{raw_path};
    if (path.is_absolute()) {
        return path;
    }
    return base_dir / path;
}

static YAML::Node loadIfReferenced(const YAML::Node& node,
                                   const std::filesystem::path& base_dir,
                                   const std::vector<std::string>& reference_keys) {
    if (!node) {
        return node;
    }
    if (node.IsScalar()) {
        return YAML::LoadFile(resolveReferencedPath(base_dir, node.as<std::string>()).string());
    }
    if (node.IsMap()) {
        for (const auto& key : reference_keys) {
            if (node[key] && node[key].IsScalar()) {
                return YAML::LoadFile(resolveReferencedPath(base_dir, node[key].as<std::string>()).string());
            }
        }
    }
    return node;
}

static drone_mapper::Position3D readPosition(const YAML::Node& node) {
    drone_mapper::Position3D position;
    position.x = drone_mapper::XLength{readLength(node, "x").force_numerical_value_in(drone_mapper::cm) * drone_mapper::cm};
    position.y = drone_mapper::YLength{readLength(node, "y").force_numerical_value_in(drone_mapper::cm) * drone_mapper::cm};
    position.z = drone_mapper::ZLength{readLength(node, "z").force_numerical_value_in(drone_mapper::cm) * drone_mapper::cm};
    return position;
}

static drone_mapper::types::SimulationConfigData parseSimulationNode(const YAML::Node& s) {
    drone_mapper::types::SimulationConfigData sim;
    sim.map_filename = s["map_filename"].as<std::string>();
    sim.map_resolution = readLength(s, "map_resolution");
    if (s["map_offset"]) {
        sim.map_offset = readPosition(s["map_offset"]);
    }
    if (s["initial_drone_position"]) {
        sim.initial_drone_position = readPosition(s["initial_drone_position"]);
    }
    if (s["initial_angle"]) {
        sim.initial_angle = HorizontalAngle{s["initial_angle"].as<double>() * mp_units::si::unit_symbols::deg};
    }
    return sim;
}

static drone_mapper::types::MissionConfigData parseMissionNode(const YAML::Node& m) {
    drone_mapper::types::MissionConfigData mission;
    mission.max_steps = m["max_steps"].as<std::size_t>(0);
    mission.gps_resolution = readLength(m, "gps_resolution");
    if (m["resolution_cm"] && !m["gps_resolution"]) {
        mission.gps_resolution = readLength(m, "resolution_cm");
    }
    if (m["output_mapping_resolution_factor"]) {
        int factor = m["output_mapping_resolution_factor"].as<int>();
        if (factor < 1) {
            const char* msg = "ERROR: output_mapping_resolution_factor < 1; ignoring and using 1";
            std::cerr << msg << "\n";
            drone_mapper::Logger::logError("YAML_OUTPUT_MAPPING_RESOLUTION_FACTOR_INVALID", msg);
            mission.output_mapping_resolution_factor = 1;
        } else {
            mission.output_mapping_resolution_factor = static_cast<double>(factor);
        }
    } else {
        mission.output_mapping_resolution_factor = 1;
    }
    return mission;
}

static drone_mapper::types::DroneConfigData parseDroneNode(const YAML::Node& d) {
    drone_mapper::types::DroneConfigData drone;
    drone.dimensions = readLength(d, "dimensions");
    drone.max_rotate = readHorizontal(d, "max_rotate");
    drone.max_advance = readLength(d, "max_advance");
    drone.max_elevate = readLength(d, "max_elevate");
    return drone;
}

static drone_mapper::types::LidarConfigData parseLidarNode(const YAML::Node& l) {
    drone_mapper::types::LidarConfigData lidar;
    lidar.z_min = readLength(l, "z_min");
    lidar.z_max = readLength(l, "z_max");
    lidar.d = readLength(l, "d");
    lidar.fov_circles = l["fov_circles"].as<std::size_t>(0);
    return lidar;
}

drone_mapper::types::SimulationCompositionData parseSimulationComposition(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());
    const std::filesystem::path base_dir = path.parent_path();

    drone_mapper::types::SimulationCompositionData comp;
    comp.composition_file = path;

    // simulations
    if (root["simulations"]) {
        for (const auto& s : root["simulations"]) {
            YAML::Node resolved = loadIfReferenced(s, base_dir, {"simulation_config", "simulation", "path", "file"});
            if (resolved["simulations"]) {
                for (const auto& nested : resolved["simulations"]) {
                    comp.simulations.push_back(parseSimulationNode(nested));
                }
            } else {
                if (resolved["simulation"]) {
                    resolved = resolved["simulation"];
                }
                comp.simulations.push_back(parseSimulationNode(resolved));
            }
        }
    }

    // missions
    if (root["missions"]) {
        for (const auto& m : root["missions"]) {
            YAML::Node resolved = loadIfReferenced(m, base_dir, {"mission_config", "mission", "path", "file"});
            if (resolved["missions"]) {
                for (const auto& nested : resolved["missions"]) {
                    comp.missions.push_back(parseMissionNode(nested));
                }
            } else {
                if (resolved["mission"]) {
                    resolved = resolved["mission"];
                }
                comp.missions.push_back(parseMissionNode(resolved));
            }
        }
    }

    // drones
    if (root["drones"]) {
        for (const auto& d : root["drones"]) {
            YAML::Node resolved = loadIfReferenced(d, base_dir, {"drone_config", "drone", "path", "file"});
            if (resolved["drones"]) {
                for (const auto& nested : resolved["drones"]) {
                    comp.drones.push_back(parseDroneNode(nested));
                }
            } else {
                if (resolved["drone"]) {
                    resolved = resolved["drone"];
                }
                comp.drones.push_back(parseDroneNode(resolved));
            }
        }
    }

    // lidars
    if (root["lidars"]) {
        for (const auto& l : root["lidars"]) {
            YAML::Node resolved = loadIfReferenced(l, base_dir, {"lidar_config", "lidar", "path", "file"});
            if (resolved["lidars"]) {
                for (const auto& nested : resolved["lidars"]) {
                    comp.lidars.push_back(parseLidarNode(nested));
                }
            } else {
                if (resolved["lidar"]) {
                    resolved = resolved["lidar"];
                }
                comp.lidars.push_back(parseLidarNode(resolved));
            }
        }
    }

    return comp;
}

} // namespace yaml
} // namespace drone_mapper
