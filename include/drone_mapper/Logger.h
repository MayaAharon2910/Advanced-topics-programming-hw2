#pragma once

#include <filesystem>
#include <string>

namespace drone_mapper {

class Logger {
public:
    static void setOutputDirectory(const std::filesystem::path& dir);
    static void logError(const std::string& code, const std::string& message);
private:
    static inline std::filesystem::path output_dir_{};
};

} // namespace drone_mapper
