#pragma once

#include "Logger.h"
#include <string>
#include <fstream>

/**
 * Optional simulation configuration loaded from simulation_config.txt.
 *
 * If the file is absent, all fields keep their default values and the
 * program behaves exactly as before — the file is never mandatory.
 *
 * Supported keys (key=value format, # comments, blank lines ignored):
 *
 *   log_enabled=true|false      whether to write simulation_log.txt  (default: true)
 *   log_level=DEBUG|INFO|WARNING|ERROR   minimum level to record     (default: INFO)
 *   debug_mode=true|false       enables DEBUG-level messages          (default: false)
 */
struct SimulationConfig {
    bool     log_enabled = true;
    LogLevel log_level   = LogLevel::INFO;
    bool     debug_mode  = false;
};

// Loads optional simulator logging settings, returning defaults when the file is absent.
inline SimulationConfig parseSimulationConfig(const std::string& filepath)
{
    SimulationConfig cfg;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        return cfg;   // file absent — use defaults silently
    }

    std::string line;
    while (std::getline(file, line)) {
        // strip comments and whitespace
        auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        while (!line.empty() && (line.back() == ' ' || line.back() == '\r')) {
            line.pop_back();
        }
        if (line.empty()) continue;

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        // trim key and value
        while (!key.empty() && key.back() == ' ') key.pop_back();
        while (!val.empty() && val.front() == ' ') val.erase(val.begin());

        if (key == "log_enabled") {
            cfg.log_enabled = (val == "true" || val == "1");
        } else if (key == "log_level") {
            if      (val == "DEBUG")   cfg.log_level = LogLevel::DEBUG;
            else if (val == "INFO")    cfg.log_level = LogLevel::INFO;
            else if (val == "WARNING") cfg.log_level = LogLevel::WARNING;
            else if (val == "ERROR")   cfg.log_level = LogLevel::ERROR;
        } else if (key == "debug_mode") {
            cfg.debug_mode = (val == "true" || val == "1");
            if (cfg.debug_mode) {
                cfg.log_level = LogLevel::DEBUG;
            }
        }
    }

    return cfg;
}
