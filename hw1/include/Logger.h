#pragma once

#include <fstream>
#include <string>

enum class LogLevel {
    DEBUG   = 0,
    INFO    = 1,
    WARNING = 2,
    ERROR   = 3
};

/**
 * Simple file logger with configurable minimum log level.
 *
 * Controlled by the optional simulation_config.txt file:
 *   log_enabled=true|false
 *   log_level=DEBUG|INFO|WARNING|ERROR
 *   debug_mode=true|false
 *
 * Messages below min_level are silently discarded.
 * When disabled (log_enabled=false) all messages are discarded.
 */
class Logger {
public:
    // Opens a log file when enabled and stores the minimum level to write.
    Logger(const std::string& filepath,
           LogLevel           min_level = LogLevel::INFO,
           bool               enabled   = true);

    // Writes a message when logging is enabled and the level passes the filter.
    void log(LogLevel level, const std::string& message);

    // Writes a DEBUG-level message.
    void debug  (const std::string& msg) { log(LogLevel::DEBUG,   msg); }
    // Writes an INFO-level message.
    void info   (const std::string& msg) { log(LogLevel::INFO,    msg); }
    // Writes a WARNING-level message.
    void warning(const std::string& msg) { log(LogLevel::WARNING, msg); }
    // Writes an ERROR-level message.
    void error  (const std::string& msg) { log(LogLevel::ERROR,   msg); }


private:
    std::ofstream file_;
    LogLevel      min_level_;
    bool          enabled_;

    // Converts a log level enum into the fixed-width text tag written to the log.
    static std::string levelTag(LogLevel level);
};
