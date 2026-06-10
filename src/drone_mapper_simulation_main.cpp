#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <filesystem>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    const std::filesystem::path composition_file =
        (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"};
    const std::filesystem::path output_path =
        (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();

    auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager simulation{std::move(run_factory)};

    // Parse composition YAML strictly using cpp-yaml
    drone_mapper::types::SimulationCompositionData composition;
    try {
        composition = drone_mapper::yaml::parseSimulationComposition(composition_file);
    } catch (const std::exception& ex) {
        std::cerr << "ERROR: Failed to parse composition file: " << ex.what() << "\n";
        return 2;
    }

    const drone_mapper::types::SimulationManagerReport report = simulation.run(composition, output_path);

    std::cout << "Assignment 2 simulator skeleton ran "
              << report.runs.size()
              << " run(s).\n";
    return 0;
}
