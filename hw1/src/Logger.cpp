#include "Logger.h"

Logger::Logger(const std::string& filepath, LogLevel min_level, bool enabled)
    : min_level_(min_level), enabled_(enabled)
{
    if (enabled_) {
        file_.open(filepath, std::ios::out | std::ios::trunc);
    }
}

void Logger::log(LogLevel level, const std::string& message)
{
    if (!enabled_ || !file_.is_open()) {
        return;
    }
    if (level < min_level_) {
        return;
    }
    // Flush per line so a timeout or crash still leaves a useful trace on disk.
    file_ << levelTag(level) << ' ' << message << '\n';
    file_.flush();
}

std::string Logger::levelTag(LogLevel level)
{
    switch (level) {
        case LogLevel::DEBUG:   return "[DEBUG  ]";
        case LogLevel::INFO:    return "[INFO   ]";
        case LogLevel::WARNING: return "[WARNING]";
        case LogLevel::ERROR:   return "[ERROR  ]";
    }
    return "[INFO   ]";
}
