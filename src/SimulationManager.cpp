#include <drone_mapper/SimulationManager.h>

#include <drone_mapper/Logger.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {
namespace {

[[nodiscard]] const char* missionStatusToString(types::MissionRunStatus status) {
    switch (status) {
        case types::MissionRunStatus::Completed: return "completed";
        case types::MissionRunStatus::MaxSteps: return "max_steps";
        case types::MissionRunStatus::Error: return "error";
    }
    return "error";
}

[[nodiscard]] const char* resolutionStatusToString(types::ResolutionRequestStatus status) {
    switch (status) {
        case types::ResolutionRequestStatus::Accepted: return "ACCEPTED";
        case types::ResolutionRequestStatus::Ignored: return "IGNORED";
        case types::ResolutionRequestStatus::IgnoredTooSmall: return "IGNORED_TOO_SMALL";
    }
    return "IGNORED";
}

std::string utcNow() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &now_time);
#else
    gmtime_r(&now_time, &tm);
#endif
    std::ostringstream out;
    out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
}

std::string yamlQuote(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (const char ch : value) {
        if (ch == '"' || ch == '\\') {
            escaped.push_back('\\');
        }
        escaped.push_back(ch);
    }
    escaped.push_back('"');
    return escaped;
}

std::string reportPath(const std::filesystem::path& source_file, const std::string& fallback) {
    return source_file.empty() ? fallback : source_file.string();
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;
    const auto output_results = output_path / "output_results";
    std::filesystem::create_directories(output_results);
    Logger::setOutputDirectory(output_results);

    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const types::MissionConfigData& mission : missions) {
            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(simulation, mission, drone, lidar, output_path);
                        types::SimulationResult sim_res = run->run();
                        if (!sim_res.mission_results.empty() &&
                            sim_res.mission_results.front().status == types::MissionRunStatus::Error) {
                            sim_res.mission_score = -1.0;
                        }
                        runs.push_back(std::move(sim_res));
                    } catch (const std::exception& ex) {
                        Logger::logError("SIMULATION_RUN_CREATE_FAILED", ex.what());
                        types::SimulationResult failed{};
                        failed.simulation_config = simulation;
                        failed.mission_config = mission;
                        failed.drone_config = drone;
                        failed.lidar_config = lidar;
                        failed.resolution_request_status = types::ResolutionRequestStatus::Ignored;
                        failed.mission_score = -1.0;
                        types::MissionRunResult mr{};
                        mr.status = types::MissionRunStatus::Error;
                        mr.errors.push_back(types::ErrorRef{"SIMULATION_RUN_CREATE_FAILED", ex.what()});
                        failed.mission_results.push_back(std::move(mr));
                        runs.push_back(std::move(failed));
                    }
                }
            }
        }
    }

    const std::size_t total_runs = runs.size();
    std::size_t error_runs = 0;
    std::size_t scored_runs = 0;
    double sum_score = 0.0;
    double min_score = std::numeric_limits<double>::infinity();
    double max_score = -std::numeric_limits<double>::infinity();

    for (const auto& r : runs) {
        const bool is_error = !r.mission_results.empty() &&
                              r.mission_results.front().status == types::MissionRunStatus::Error;
        if (is_error || r.mission_score < 0.0) {
            ++error_runs;
            continue;
        }
        ++scored_runs;
        sum_score += r.mission_score;
        min_score = std::min(min_score, r.mission_score);
        max_score = std::max(max_score, r.mission_score);
    }

    const double average_score = scored_runs == 0 ? 0.0 : sum_score / static_cast<double>(scored_runs);
    if (scored_runs == 0) {
        min_score = 0.0;
        max_score = 0.0;
    }

    const std::string generated_at = utcNow();
    std::ostringstream yaml_text;
    yaml_text << std::fixed << std::setprecision(2);
    yaml_text << "score_report:\n";
    yaml_text << "  composition_file: " << yamlQuote(composition.composition_file.string()) << "\n";
    yaml_text << "  generated_at_utc: " << yamlQuote(generated_at) << "\n";
    yaml_text << "  metric: " << yamlQuote("output_map_accuracy") << "\n";
    yaml_text << "  score_range:\n";
    yaml_text << "    min: 0\n";
    yaml_text << "    max: 100\n";
    yaml_text << "    error_score: -1\n";
    yaml_text << "  summary:\n";
    yaml_text << "    total_runs: " << total_runs << "\n";
    yaml_text << "    scored_runs: " << scored_runs << "\n";
    yaml_text << "    error_runs: " << error_runs << "\n";
    yaml_text << "    average_score: " << average_score << "\n";
    yaml_text << "    min_score: " << min_score << "\n";
    yaml_text << "    max_score: " << max_score << "\n";
    yaml_text << "  simulations:\n";

    for (const auto& r : runs) {
        const auto& mr = r.mission_results.empty() ? types::MissionRunResult{} : r.mission_results.front();
        yaml_text << "    - simulation_config: "
                  << yamlQuote(reportPath(r.simulation_config.source_file, r.simulation_config.map_filename.string())) << "\n";
        yaml_text << "      missions:\n";
        yaml_text << "        - mission_config: "
                  << yamlQuote(reportPath(r.mission_config.source_file, "inline_mission_config")) << "\n";
        yaml_text << "          resolution_cm: " << r.output_map_config.resolution.force_numerical_value_in(cm) << "\n";
        yaml_text << "          resolution_request_status: " << resolutionStatusToString(r.resolution_request_status) << "\n";
        yaml_text << "          runs:\n";
        yaml_text << "            - drone_config: "
                  << yamlQuote(reportPath(r.drone_config.source_file, "inline_drone_config")) << "\n";
        yaml_text << "              lidar_config: "
                  << yamlQuote(reportPath(r.lidar_config.source_file, "inline_lidar_config")) << "\n";
        yaml_text << "              status: " << yamlQuote(missionStatusToString(mr.status)) << "\n";
        yaml_text << "              steps: " << mr.steps << "\n";
        yaml_text << "              score: " << r.mission_score << "\n";
        if (!r.output_map_file.empty()) {
            yaml_text << "              output_map_file: " << yamlQuote(r.output_map_file.string()) << "\n";
        }
        if (mr.status == types::MissionRunStatus::Error && !mr.errors.empty()) {
            yaml_text << "              error_ref:\n";
            yaml_text << "                code: " << yamlQuote(mr.errors.front().code) << "\n";
            yaml_text << "                message: " << yamlQuote(mr.errors.front().message) << "\n";
        }
    }

    const auto yaml_file = output_path / "simulation_output.yaml";
    try {
        std::ofstream out(yaml_file);
        out << yaml_text.str();
    } catch (const std::exception& ex) {
        Logger::logError("SIMULATION_OUTPUT_WRITE_FAILED", ex.what());
    }

    return types::SimulationManagerReport{generated_at, "output_map_accuracy", {0.0, 100.0}, -1, std::move(runs)};
}

} // namespace drone_mapper
