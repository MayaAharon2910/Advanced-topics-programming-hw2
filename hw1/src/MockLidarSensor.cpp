#include "MockLidarSensor.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace {

struct Vec3 {
    double x;
    double y;
    double z;
};

constexpr int FIRST_FOV_RING = 1;
constexpr int BEAM_BRANCHING_FACTOR = 4;
constexpr double AXIS_ALIGNMENT_EPSILON = 1e-6;
constexpr double FULL_CIRCLE_RAD = 2.0 * std::numbers::pi_v<double>;
constexpr double RAD_TO_DEG = 180.0 / std::numbers::pi_v<double>;
constexpr double MIN_DIRECTION_COMPONENT = -1.0;
constexpr double MAX_DIRECTION_COMPONENT = 1.0;

Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(double s, const Vec3& v) {
    return {s * v.x, s * v.y, s * v.z};
}


double length(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 normalize(const Vec3& v) {
    double len = length(v);
    if (len == 0.0) {
        return {0.0, 0.0, 0.0};
    }
    return {v.x / len, v.y / len, v.z / len};
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

std::optional<double> castRay(const Map3D& map,
                              const Vec3& start,
                              const Vec3& dir,
                              double max_dist,
                              double min_dist) {
    int x = static_cast<int>(std::floor(start.x));
    int y = static_cast<int>(std::floor(start.y));
    int z = static_cast<int>(std::floor(start.z));

    int stepX = (dir.x > 0.0) ? 1 : -1;
    int stepY = (dir.y > 0.0) ? 1 : -1;
    int stepZ = (dir.z > 0.0) ? 1 : -1;

    const double infinity = std::numeric_limits<double>::infinity();
    double tMaxX = (dir.x == 0.0) ? infinity : ((dir.x > 0.0) ? ((x + 1.0 - start.x) / dir.x) : ((start.x - x) / -dir.x));
    double tMaxY = (dir.y == 0.0) ? infinity : ((dir.y > 0.0) ? ((y + 1.0 - start.y) / dir.y) : ((start.y - y) / -dir.y));
    double tMaxZ = (dir.z == 0.0) ? infinity : ((dir.z > 0.0) ? ((z + 1.0 - start.z) / dir.z) : ((start.z - z) / -dir.z));

    double tDeltaX = (dir.x == 0.0) ? infinity : std::fabs(1.0 / dir.x);
    double tDeltaY = (dir.y == 0.0) ? infinity : std::fabs(1.0 / dir.y);
    double tDeltaZ = (dir.z == 0.0) ? infinity : std::fabs(1.0 / dir.z);

    // 3D DDA: walk voxel boundaries instead of sampling tiny distance steps.
    double t = 0.0;
    while (t < max_dist) {
        if (x >= 0 && x < static_cast<int>(map.width()) &&
            y >= 0 && y < static_cast<int>(map.height()) &&
            z >= 0 && z < static_cast<int>(map.depth())) {
            if (map.at(x, y, z) == Map3D::OCCUPIED) {
                double dist = t;
                if (dist < min_dist) {
                    return 0.0;
                }
                return dist;
            }
        }

        if (tMaxX < tMaxY && tMaxX < tMaxZ) {
            x += stepX;
            t = tMaxX;
            tMaxX += tDeltaX;
        } else if (tMaxY < tMaxZ) {
            y += stepY;
            t = tMaxY;
            tMaxY += tDeltaY;
        } else {
            z += stepZ;
            t = tMaxZ;
            tMaxZ += tDeltaZ;
        }
    }
    return std::nullopt;
}

} // namespace

MockLidarSensor::MockLidarSensor(const Map3D& map,
                                 mp_units::quantity<mp_units::si::unit_symbols::cm> z_min,
                                 mp_units::quantity<mp_units::si::unit_symbols::cm> z_max,
                                 int fovc,
                                 mp_units::quantity<mp_units::si::unit_symbols::cm> d)
    : map_(map),
      z_min_(z_min.numerical_value_in(mp_units::si::unit_symbols::cm)),
      z_max_(z_max.numerical_value_in(mp_units::si::unit_symbols::cm)),
      fovc_(fovc),
      d_(d.numerical_value_in(mp_units::si::unit_symbols::cm)) {}

std::vector<ScanResult> MockLidarSensor::scan(const StrongPosition3D& position,
                                              mp_units::quantity<mp_units::si::unit_symbols::deg, double> azimuth,
                                              mp_units::quantity<mp_units::si::unit_symbols::deg, double> elevation) const {
    double az_rad = azimuth.numerical_value_in(mp_units::si::unit_symbols::rad);
    double el_rad = elevation.numerical_value_in(mp_units::si::unit_symbols::rad);

    Vec3 dir_central = {
        std::cos(el_rad) * std::cos(az_rad),
        std::cos(el_rad) * std::sin(az_rad),
        std::sin(el_rad)
    };
    dir_central = normalize(dir_central);

    Vec3 drone_pos = {
        position.x.numerical_value_in(mp_units::si::unit_symbols::cm),
        position.y.numerical_value_in(mp_units::si::unit_symbols::cm),
        position.z.numerical_value_in(mp_units::si::unit_symbols::cm)
    };

    std::vector<Vec3> beam_dirs;
    beam_dirs.reserve(1 + (fovc_ > FIRST_FOV_RING ? (1 << (2 * fovc_)) : 0));

    beam_dirs.push_back(dir_central);

    // Each FOV ring places beams on a circle around the central ray.
    for (int i = FIRST_FOV_RING; i < fovc_; ++i) {
        int num_beams_circle = 1;
        for (int p = 0; p < i; ++p) {
            num_beams_circle *= BEAM_BRANCHING_FACTOR;
        }

        Vec3 circle_center = drone_pos + z_min_ * dir_central;
        Vec3 perp1;
        if (std::fabs(dir_central.x) > AXIS_ALIGNMENT_EPSILON ||
            std::fabs(dir_central.z) > AXIS_ALIGNMENT_EPSILON) {
            perp1 = normalize(cross(dir_central, {0.0, 1.0, 0.0}));
        } else {
            perp1 = normalize(cross(dir_central, {1.0, 0.0, 0.0}));
        }
        Vec3 perp2 = normalize(cross(dir_central, perp1));

        double radius = i * d_;
        for (int j = 0; j < num_beams_circle; ++j) {
            double theta = j * FULL_CIRCLE_RAD / static_cast<double>(num_beams_circle);
            Vec3 offset = radius * (std::cos(theta) * perp1 + std::sin(theta) * perp2);
            Vec3 target = circle_center + offset;
            Vec3 beam_dir = normalize(target - drone_pos);
            beam_dirs.push_back(beam_dir);
        }
    }

    std::vector<ScanResult> results;
    results.reserve(beam_dirs.size());
    for (const auto& dir : beam_dirs) {
        auto dist = castRay(map_, drone_pos, dir, z_max_, z_min_);
        if (!dist.has_value()) {
            continue;
        }

        double beam_azimuth = std::atan2(dir.y, dir.x);
        double beam_elevation = std::asin(std::clamp(dir.z, MIN_DIRECTION_COMPONENT, MAX_DIRECTION_COMPONENT));
        results.push_back(ScanResult{
            *dist,
            beam_azimuth * RAD_TO_DEG,
            beam_elevation * RAD_TO_DEG,
            dir.x,
            dir.y,
            dir.z});
    }
    return results;
}
