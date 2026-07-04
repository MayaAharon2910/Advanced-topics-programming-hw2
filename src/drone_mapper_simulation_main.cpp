#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <drone_mapper/YamlConfig.h>
#include <drone_mapper/Logger.h>

static std::filesystem::path resolve_input_path_or_cwd(const char* arg, const std::filesystem::path& cwd, const char* default_name) {
    if (!arg) return cwd / default_name;
    const std::string s(arg);
    if (!s.empty() && s.front() == '/') return std::filesystem::path{s};
    return cwd / s;
}

static std::filesystem::path resolve_output_path_or_cwd(const char* arg, const std::filesystem::path& cwd) {
    if (!arg) return cwd;
    const std::string s(arg);
    if (!s.empty() && s.front() == '/') return std::filesystem::path{s};
    return cwd / s;
}

int main(int argc, char** argv) {
    const std::filesystem::path cwd = std::filesystem::current_path();
    const std::filesystem::path composition_file = (argc >= 2)
                                                      ? resolve_input_path_or_cwd(argv[1], cwd, "simulation.yaml")
                                                      : (cwd / "simulation.yaml");
    const std::filesystem::path output_path = (argc >= 3)
                                                 ? resolve_output_path_or_cwd(argv[2], cwd)
                                                 : cwd;

    // Errors must be written to the error log file immediately when they occur,
    // including composition parse failures below — so the Logger's output
    // directory is initialized here, BEFORE any YAML parsing. SimulationManager
    // sets the same directory again later, which is a harmless no-op.
    drone_mapper::Logger::setOutputDirectory(output_path / "output_results");

    auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager simulation{std::move(run_factory)};

    // Parse composition YAML strictly using cpp-yaml
    drone_mapper::types::SimulationCompositionData composition;
    try {
        composition = drone_mapper::yaml::parseSimulationComposition(composition_file);
    } catch (const std::exception& ex) {
        const std::string msg = "ERROR: Failed to parse composition file '" + composition_file.string() + "': " + ex.what();
        std::cerr << msg << "\n";
        drone_mapper::Logger::logError("COMPOSITION_PARSE_FAILED", msg);
        return 2;
    }

    const drone_mapper::types::SimulationManagerReport report = simulation.run(composition, output_path);

    std::cout << "Drone mapper simulation completed "
              << report.runs.size()
              << " run(s).\n";
    return 0;
}
