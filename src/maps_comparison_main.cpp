#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Map3DImpl.h>

#include <yaml-cpp/yaml.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include <drone_mapper/Logger.h>
#include <drone_mapper/Units.h>

namespace {

drone_mapper::Position3D readOffset(const YAML::Node& node) {
    if (!node) {
        return drone_mapper::Position3D{};
    }
    return drone_mapper::Position3D{
        node["x_offset"].as<double>(0.0) * drone_mapper::x_extent[drone_mapper::cm],
        node["y_offset"].as<double>(0.0) * drone_mapper::y_extent[drone_mapper::cm],
        node["height_offset"].as<double>(0.0) * drone_mapper::z_extent[drone_mapper::cm],
    };
}

drone_mapper::types::MappingBounds readBounds(const YAML::Node& node) {
    drone_mapper::types::MappingBounds b{};
    if (!node) {
        return b;
    }
    b.min_x = node["x_boundary"]["min_cm"].as<double>(0.0) * drone_mapper::x_extent[drone_mapper::cm];
    b.max_x = node["x_boundary"]["max_cm"].as<double>(0.0) * drone_mapper::x_extent[drone_mapper::cm];
    b.min_y = node["y_boundary"]["min_cm"].as<double>(0.0) * drone_mapper::y_extent[drone_mapper::cm];
    b.max_y = node["y_boundary"]["max_cm"].as<double>(0.0) * drone_mapper::y_extent[drone_mapper::cm];
    b.min_height = node["height_boundary"]["min_cm"].as<double>(0.0) * drone_mapper::z_extent[drone_mapper::cm];
    b.max_height = node["height_boundary"]["max_cm"].as<double>(0.0) * drone_mapper::z_extent[drone_mapper::cm];
    return b;
}

drone_mapper::types::MapConfig readMapConfig(const YAML::Node& node) {
    drone_mapper::types::MapConfig cfg{};
    cfg.resolution = node["map_res_cm"].as<double>(1.0) * drone_mapper::cm;
    cfg.offset = readOffset(node["map_offset"]);
    cfg.boundaries = readBounds(node["map_boundaries"]);
    return cfg;
}

std::string parseConfigPath(const std::string& raw_arg) {
    const std::string prefix = "comparison_config=";
    if (raw_arg.rfind(prefix, 0) == 0) {
        return raw_arg.substr(prefix.size());
    }
    return raw_arg;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }
    try {
        const std::string origin_path = argv[1];
        const std::string target_path = argv[2];

        auto load = [](const std::string& p) -> std::shared_ptr<NpyArray> {
            auto arr = std::make_shared<NpyArray>();
            const char* err = arr->LoadNPY(p);
            if (err != nullptr) {
                throw std::runtime_error(std::string("Failed to load NPY: ") + err);
            }
            return arr;
        };

        auto origin_arr = load(origin_path);
        auto target_arr = load(target_path);

        drone_mapper::types::MapConfig origin_cfg{};
        origin_cfg.offset = drone_mapper::Position3D{};
        origin_cfg.resolution = 1.0 * drone_mapper::cm;
        drone_mapper::types::MapConfig target_cfg = origin_cfg;

        if (argc == 4) {
            const std::string config_path = parseConfigPath(argv[3]);
            YAML::Node root = YAML::LoadFile(config_path);
            YAML::Node comparison = root["comparison_config"] ? root["comparison_config"] : root;
            if (!comparison["original"] || !comparison["target"]) {
                throw std::runtime_error("comparison_config must contain original and target map configs");
            }
            origin_cfg = readMapConfig(comparison["original"]);
            target_cfg = readMapConfig(comparison["target"]);
        }

        drone_mapper::Map3DImpl origin_map(origin_arr, origin_cfg);
        drone_mapper::Map3DImpl target_map(target_arr, target_cfg);

        auto scores = drone_mapper::MapsComparison::compare(origin_map, {&target_map});
        if (scores.empty()) {
            const char* msg = "MapsComparison: no score produced";
            std::cout << "-1\n";
            std::cerr << msg << "\n";
            drone_mapper::Logger::logError("MAPS_COMPARISON_NO_SCORE", msg);
            return 1;
        }

        std::cout.setf(std::ios::fixed);
        std::cout.precision(6);
        std::cout << scores.front() << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cout << "-1\n";
        std::cerr << ex.what() << std::endl;
        drone_mapper::Logger::logError("MAPS_COMPARISON_EXCEPTION", ex.what());
        return 1;
    }
}
