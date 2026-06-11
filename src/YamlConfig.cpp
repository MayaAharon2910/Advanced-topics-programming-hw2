#include <drone_mapper/YamlConfig.h>

#include <yaml-cpp/yaml.h>

#include <iostream>

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

drone_mapper::types::SimulationCompositionData parseSimulationComposition(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());

    drone_mapper::types::SimulationCompositionData comp;
    comp.composition_file = path;

    // simulations
    if (root["simulations"]) {
        for (const auto& s : root["simulations"]) {
            drone_mapper::types::SimulationConfigData sim;
            sim.map_filename = s["map_filename"].as<std::string>();
            sim.map_resolution = readLength(s, "map_resolution");
            // map_offset optional
            if (s["map_offset"]) {
                drone_mapper::Position3D off;
                off.x = drone_mapper::XLength{ readLength(s["map_offset"], "x").force_numerical_value_in(drone_mapper::cm) * drone_mapper::cm };
                off.y = drone_mapper::YLength{ readLength(s["map_offset"], "y").force_numerical_value_in(drone_mapper::cm) * drone_mapper::cm };
                off.z = drone_mapper::ZLength{ readLength(s["map_offset"], "z").force_numerical_value_in(drone_mapper::cm) * drone_mapper::cm };
                sim.map_offset = off;
            }
            if (s["initial_drone_position"]) {
                drone_mapper::Position3D p;
                p.x = drone_mapper::XLength{ readLength(s["initial_drone_position"], "x").force_numerical_value_in(drone_mapper::cm) * drone_mapper::cm };
                p.y = drone_mapper::YLength{ readLength(s["initial_drone_position"], "y").force_numerical_value_in(drone_mapper::cm) * drone_mapper::cm };
                p.z = drone_mapper::ZLength{ readLength(s["initial_drone_position"], "z").force_numerical_value_in(drone_mapper::cm) * drone_mapper::cm };
                sim.initial_drone_position = p;
            }
            if (s["initial_angle"]) {
                sim.initial_angle = HorizontalAngle{s["initial_angle"].as<double>() * mp_units::si::unit_symbols::deg};
            }
            comp.simulations.push_back(sim);
        }
    }

    // missions
    if (root["missions"]) {
        for (const auto& m : root["missions"]) {
            drone_mapper::types::MissionConfigData mission;
            mission.max_steps = m["max_steps"].as<std::size_t>(0);
            mission.gps_resolution = readLength(m, "gps_resolution");
            // output_mapping_resolution_factor default 1 if missing
            if (m["output_mapping_resolution_factor"]) {
                int factor = m["output_mapping_resolution_factor"].as<int>();
                if (factor < 1) {
                    std::cerr << "ERROR: output_mapping_resolution_factor < 1; ignoring and using 1\n";
                    mission.output_mapping_resolution_factor = 1;
                } else {
                    mission.output_mapping_resolution_factor = static_cast<double>(factor);
                }
            } else {
                mission.output_mapping_resolution_factor = 1;
            }
            comp.missions.push_back(mission);
        }
    }

    // drones
    if (root["drones"]) {
        for (const auto& d : root["drones"]) {
            drone_mapper::types::DroneConfigData drone;
            drone.dimensions = readLength(d, "dimensions");
            drone.max_rotate = readHorizontal(d, "max_rotate");
            drone.max_advance = readLength(d, "max_advance");
            drone.max_elevate = readLength(d, "max_elevate");
            comp.drones.push_back(drone);
        }
    }

    // lidars
    if (root["lidars"]) {
        for (const auto& l : root["lidars"]) {
            drone_mapper::types::LidarConfigData lidar;
            lidar.z_min = readLength(l, "z_min");
            lidar.z_max = readLength(l, "z_max");
            lidar.d = readLength(l, "d");
            lidar.fov_circles = l["fov_circles"].as<std::size_t>(0);
            comp.lidars.push_back(lidar);
        }
    }

    return comp;
}

} // namespace yaml
} // namespace drone_mapper
