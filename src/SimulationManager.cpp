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
#include <yaml-cpp/yaml.h>

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
        case types::ResolutionRequestStatus::IgnoredTooSmall: return "IGNORED TOO SMALL";
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

    const auto sourceOr = [](const std::vector<std::filesystem::path>& sources, std::size_t idx) {
        return idx < sources.size() ? sources[idx] : std::filesystem::path{};
    };

    for (std::size_t si = 0; si < composition.simulation_mission_groups.size(); ++si) {
        const auto& [simulation, missions] = composition.simulation_mission_groups[si];
        const std::filesystem::path sim_source = sourceOr(composition.simulation_source_files, si);
        const std::vector<std::filesystem::path> no_mission_sources{};
        const std::vector<std::filesystem::path>& mission_sources =
            si < composition.mission_source_files_by_group.size() ? composition.mission_source_files_by_group[si]
                                                                    : no_mission_sources;
        for (std::size_t mi = 0; mi < missions.size(); ++mi) {
            const types::MissionConfigData& mission = missions[mi];
            const std::filesystem::path mission_source = sourceOr(mission_sources, mi);
            for (std::size_t di = 0; di < composition.drones.size(); ++di) {
                const types::DroneConfigData& drone = composition.drones[di];
                const std::filesystem::path drone_source = sourceOr(composition.drone_source_files, di);
                for (std::size_t li = 0; li < composition.lidars.size(); ++li) {
                    const types::LidarConfigData& lidar = composition.lidars[li];
                    const std::filesystem::path lidar_source = sourceOr(composition.lidar_source_files, li);
                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(simulation, mission, drone, lidar, output_path);
                        types::SimulationResult sim_res = run->run();
                        if (!sim_res.mission_results.empty() &&
                            sim_res.mission_results.front().status == types::MissionRunStatus::Error) {
                            sim_res.mission_score = -1.0;
                        }
                        sim_res.simulation_source_file = sim_source;
                        sim_res.mission_source_file = mission_source;
                        sim_res.drone_source_file = drone_source;
                        sim_res.lidar_source_file = lidar_source;
                        runs.push_back(std::move(sim_res));
                    } catch (const std::exception& ex) {
                        Logger::logError("SIMULATION_RUN_CREATE_FAILED", ex.what());
                        types::SimulationResult failed{};
                        failed.simulation_config = simulation;
                        failed.mission_config = mission;
                        failed.drone_config = drone;
                        failed.lidar_config = lidar;
                        failed.simulation_source_file = sim_source;
                        failed.mission_source_file = mission_source;
                        failed.drone_source_file = drone_source;
                        failed.lidar_source_file = lidar_source;
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

    YAML::Node root;
    YAML::Node report = YAML::Node(YAML::NodeType::Map);
    report["composition_file"] = composition.composition_file.string();
    report["generated_at_utc"] = generated_at;
    report["metric"] = "output_map_accuracy";

    YAML::Node score_range = YAML::Node(YAML::NodeType::Map);
    score_range["min"] = 0;
    score_range["max"] = 100;
    score_range["error_score"] = -1;
    report["score_range"] = score_range;

    YAML::Node summary = YAML::Node(YAML::NodeType::Map);
    summary["total_runs"] = static_cast<int>(total_runs);
    summary["scored_runs"] = static_cast<int>(scored_runs);
    summary["error_runs"] = static_cast<int>(error_runs);
    summary["average_score"] = average_score;
    summary["min_score"] = min_score;
    summary["max_score"] = max_score;
    report["summary"] = summary;

    YAML::Node simulations = YAML::Node(YAML::NodeType::Sequence);
    const std::size_t n = runs.size();
    std::size_t i = 0;
    while (i < n) {
        const auto& sim_key = runs[i].simulation_config;
        const auto& sim_source_key = runs[i].simulation_source_file;
        YAML::Node sim_node = YAML::Node(YAML::NodeType::Map);
        sim_node["simulation_config"] = reportPath(sim_source_key, sim_key.map_filename.string());

        YAML::Node missions = YAML::Node(YAML::NodeType::Sequence);
        std::size_t j = i;
        while (j < n && runs[j].simulation_source_file == sim_source_key &&
               runs[j].simulation_config.map_filename == sim_key.map_filename) {
            const auto& mission_source_key = runs[j].mission_source_file;
            YAML::Node mission_node = YAML::Node(YAML::NodeType::Map);
            mission_node["mission_config"] = reportPath(mission_source_key, "inline_mission_config");
            mission_node["resolution_cm"] = runs[j].output_map_config.resolution.force_numerical_value_in(cm);
            mission_node["resolution_request_status"] = resolutionStatusToString(runs[j].resolution_request_status);

            YAML::Node runs_node = YAML::Node(YAML::NodeType::Sequence);
            std::size_t k = j;
            while (k < n && runs[k].simulation_source_file == sim_source_key &&
                   runs[k].simulation_config.map_filename == sim_key.map_filename &&
                   runs[k].mission_source_file == mission_source_key) {
                YAML::Node run_node = YAML::Node(YAML::NodeType::Map);
                run_node["drone_config"] = reportPath(runs[k].drone_source_file, "inline_drone_config");
                run_node["lidar_config"] = reportPath(runs[k].lidar_source_file, "inline_lidar_config");
                const auto& mr = runs[k].mission_results.empty() ? types::MissionRunResult{} : runs[k].mission_results.front();
                run_node["status"] = missionStatusToString(mr.status);
                run_node["steps"] = static_cast<int>(mr.steps);
                run_node["score"] = runs[k].mission_score;
                if (!runs[k].output_map_file.empty()) run_node["output_map_file"] = runs[k].output_map_file.string();
                if (mr.status == types::MissionRunStatus::Error && !mr.errors.empty()) {
                    YAML::Node err = YAML::Node(YAML::NodeType::Map);
                    err["code"] = mr.errors.front().code;
                    err["message"] = mr.errors.front().message;
                    run_node["error_ref"] = err;
                }
                runs_node.push_back(run_node);
                ++k;
            }
            mission_node["runs"] = runs_node;
            missions.push_back(mission_node);
            j = k;
        }

        sim_node["missions"] = missions;
        simulations.push_back(sim_node);
        i = j;
    }

    report["simulations"] = simulations;
    root["score_report"] = report;

    const auto yaml_file = output_path / "simulation_output.yaml";
    try {
        YAML::Emitter emitter;
        emitter << root;
        std::ofstream out(yaml_file);
        out << emitter.c_str();
    } catch (const std::exception& ex) {
        Logger::logError("SIMULATION_OUTPUT_WRITE_FAILED", ex.what());
    }

    return types::SimulationManagerReport{generated_at, "output_map_accuracy", {0.0, 100.0}, -1, std::move(runs)};
}

} // namespace drone_mapper
