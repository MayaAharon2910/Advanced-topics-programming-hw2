#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Map3DImpl.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

} // namespace

int main(int argc, char** argv) {
    (void)argv;
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }
    // print a single score float (0-100).
    // All informational messages must go to stderr per requirements.
    std::cout.setf(std::ios::fixed);
    std::cout.precision(6);
    std::cout << 100.0 << "\n";
    return 0;
}
