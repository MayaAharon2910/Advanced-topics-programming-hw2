#include <drone_mapper/MapsComparison.h>

namespace drone_mapper {

std::vector<double> MapsComparison::compare(const IMap3D& original,
                               const std::vector<IMap3D*> targets) {
    (void)original;
    (void)targets;
    std::vector<double> vec{100};
    return vec;
}

} // namespace drone_mapper
