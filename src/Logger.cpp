#include <drone_mapper/Logger.h>

#include <fstream>
#include <iostream>

namespace drone_mapper {

void Logger::setOutputDirectory(const std::filesystem::path& dir) {
    output_dir_ = dir;
    try {
        std::filesystem::create_directories(output_dir_);
    } catch (...) {
        // best-effort: ignore
    }
}

void Logger::logError(const std::string& code, const std::string& message) {
    // Synchronous immediate logging: write to stderr and append to error log file.
    std::cerr << code << ": " << message << std::endl;
    if (output_dir_.empty()) return;
    const auto log_file = output_dir_ / "error_log.txt";
    std::ofstream out(log_file, std::ios::app);
    if (out) {
        out << code << ": " << message << "\n";
    }
}

} // namespace drone_mapper
