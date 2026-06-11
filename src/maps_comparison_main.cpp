#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Map3DImpl.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <drone_mapper/Units.h>
#include <drone_mapper/Logger.h>

namespace {

} // namespace

int main(int argc, char** argv) {
    (void)argv;
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }
    try {
        const std::string origin_path = argv[1];
        const std::string target_path = argv[2];

        // Load NPY arrays
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


        // Default MapConfig: zero offset, resolution 1cm
        drone_mapper::types::MapConfig default_cfg;
        default_cfg.offset = drone_mapper::Position3D{0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm, 0.0 * drone_mapper::cm};
        default_cfg.resolution = 1.0 * drone_mapper::cm;

        drone_mapper::Map3DImpl origin_map(origin_arr, default_cfg);
        drone_mapper::Map3DImpl target_map(target_arr, default_cfg);

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
        // On error: print -1 to stdout and descriptive message to stderr
        std::cout << "-1\n";
        std::cerr << ex.what() << std::endl;
        drone_mapper::Logger::logError("MAPS_COMPARISON_EXCEPTION", ex.what());
        return 1;
    }
}
