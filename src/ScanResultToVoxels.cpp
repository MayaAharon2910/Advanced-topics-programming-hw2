#include <drone_mapper/ScanResultToVoxels.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
#include <limits>

namespace drone_mapper {
namespace {

struct Direction {
    double x;
    double y;
    double z;
};

[[nodiscard]] Direction directionFrom(const Orientation& orientation) {
    const auto cos_altitude = si::cos(orientation.altitude);
    return Direction{
        (cos_altitude * si::cos(orientation.horizontal)).force_numerical_value_in(mp::one),
        (cos_altitude * si::sin(orientation.horizontal)).force_numerical_value_in(mp::one),
        si::sin(orientation.altitude).force_numerical_value_in(mp::one),
    };
}

[[nodiscard]] Position3D pointAlong(const Position3D& origin, const Direction& dir, double distance_cm) {
    return Position3D{
        origin.x + dir.x * distance_cm * x_extent[cm],
        origin.y + dir.y * distance_cm * y_extent[cm],
        origin.z + dir.z * distance_cm * z_extent[cm],
    };
}

} // namespace

std::vector<types::MappedVoxel> ScanResultToVoxels::convert(const Position3D& scan_origin,
                                                            const Orientation& drone_heading,
                                                            const types::LidarScanResult& scan) {
    std::vector<types::MappedVoxel> voxels;
    constexpr double step_cm = 1.0;

    for (const types::LidarHit& hit : scan) {
        const double distance_cm = hit.distance.force_numerical_value_in(cm);
        if (distance_cm < 0.0 ||
            !std::isfinite(distance_cm) ||
            distance_cm >= std::numeric_limits<double>::max() / 2.0) {
            continue;
        }

        const Orientation absolute_orientation{
            drone_heading.horizontal + hit.angle.horizontal,
            drone_heading.altitude + hit.angle.altitude,
        };
        const Direction dir = directionFrom(absolute_orientation);

        for (double t = 0.0; t < distance_cm; t += step_cm) {
            voxels.push_back(types::MappedVoxel{
                pointAlong(scan_origin, dir, t),
                types::VoxelOccupancy::Empty,
            });
        }

        voxels.push_back(types::MappedVoxel{
            pointAlong(scan_origin, dir, distance_cm),
            types::VoxelOccupancy::Occupied,
        });
    }

    return voxels;
}

} // namespace drone_mapper
