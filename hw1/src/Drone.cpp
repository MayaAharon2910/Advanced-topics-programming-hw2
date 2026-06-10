#include "Drone.h"
#include "ILidarSensor.h"
#include "IMovementDriver.h"
#include "IPositionSensor.h"
#include <algorithm>
#include <array>
#include <queue>
#include <set>
#include <cmath>
#include <numbers>
#include <limits>
#include <optional>
#include <tuple>
#include <stdexcept>

namespace {

// Small helpers in this anonymous namespace are private to Drone.cpp.
// They keep the public Drone class focused on the exploration state machine.

constexpr double COMMAND_EPSILON    = 1e-6;
constexpr double FULL_CIRCLE_DEG    = 360.0;
constexpr double HALF_CIRCLE_DEG    = 180.0;
constexpr double ELEVATION_UP_DEG   = 90.0;
constexpr double ELEVATION_DOWN_DEG = -90.0;
constexpr double VOXEL_SIZE_CM      = 1.0;
constexpr double RAD_TO_DEG          = 180.0 / 3.14159265358979323846;
constexpr size_t CARDINAL_DIRECTION_COUNT = 6;
constexpr double SCAN_FORWARD_RELATIVE_DEG = 0.0;
constexpr double SCAN_RIGHT_RELATIVE_DEG   = 90.0;
constexpr double SCAN_BACK_RELATIVE_DEG    = 180.0;
constexpr double SCAN_LEFT_RELATIVE_DEG    = -90.0;
constexpr int FIRST_FOV_RING = 1;
constexpr int BEAM_BRANCHING_FACTOR = 4;
constexpr double AXIS_ALIGNMENT_EPSILON = 1e-6;
constexpr double FULL_CIRCLE_RAD = 2.0 * std::numbers::pi_v<double>;
constexpr double DEG_TO_RAD = std::numbers::pi_v<double> / HALF_CIRCLE_DEG;
constexpr double MIN_DIRECTION_COMPONENT = -1.0;
constexpr double MAX_DIRECTION_COMPONENT = 1.0;

struct GridOffset {
    int dx;
    int dy;
    int dz;
};

struct Vec3 {
    double x;
    double y;
    double z;
};

Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}


Vec3 operator*(double s, const Vec3& v) {
    return {s * v.x, s * v.y, s * v.z};
}


double length(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 normalize(const Vec3& v) {
    const double len = length(v);
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

bool sameBeamDirection(const ScanResult& result, const Vec3& expected) {
    constexpr double DIRECTION_DOT_MATCH_THRESHOLD = 0.999;
    const Vec3 received = normalize({result.dir_x, result.dir_y, result.dir_z});
    const Vec3 normalized_expected = normalize(expected);
    const double dot = received.x * normalized_expected.x +
                       received.y * normalized_expected.y +
                       received.z * normalized_expected.z;
    return dot >= DIRECTION_DOT_MATCH_THRESHOLD;
}

constexpr std::array<GridOffset, CARDINAL_DIRECTION_COUNT> BFS_NEIGHBOR_DIRECTIONS = {{
    {-1, 0, 0},
    {1, 0, 0},
    {0, -1, 0},
    {0, 1, 0},
    {0, 0, -1},
    {0, 0, 1}
}};

constexpr std::array<GridOffset, CARDINAL_DIRECTION_COUNT> SWEEP_DIRECTIONS = {{
    {1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {-1, 0, 0},
    {0, 0, 1},
    {0, 0, -1}
}};

template <typename Quantity>
int roundedCentimeters(const Quantity& value) {
    return static_cast<int>(std::round(value.numerical_value_in(mp_units::si::unit_symbols::cm)));
}

StrongPosition3D positionFromVoxel(int x, int y, int z) {
    return {
        x * VOXEL_SIZE_CM * mp_units::si::unit_symbols::cm,
        y * VOXEL_SIZE_CM * mp_units::si::unit_symbols::cm,
        z * VOXEL_SIZE_CM * mp_units::si::unit_symbols::cm
    };
}


bool isInsideMap(const Map3D& map, int x, int y, int z) {
    return x >= 0 && y >= 0 && z >= 0 &&
           x < static_cast<int>(map.width()) &&
           y < static_cast<int>(map.height()) &&
           z < static_cast<int>(map.depth());
}

// drone ןד a sphere.
// pass dimension as the diameter, so the safety check is conservative.
mp_units::quantity<mp_units::si::unit_symbols::cm> computeSphereRadius(const Config& config) {
    auto r = config.min_pass_width_cm;
    if (config.min_pass_length_cm > r) r = config.min_pass_length_cm;
    if (config.min_pass_height_cm > r) r = config.min_pass_height_cm;
    return r / 2;
}

// A voxel is safe only if the whole spherical footprint fits inside known FREE space.
bool sphereAreaIsFree(const Map3D& map, int cx, int cy, int cz,
                      mp_units::quantity<mp_units::si::unit_symbols::cm> radius) {
    const double r_cm = radius.numerical_value_in(mp_units::si::unit_symbols::cm);
    const int    r    = static_cast<int>(std::ceil(r_cm));
    const double r2   = r_cm * r_cm;
    for (int dz = -r; dz <= r; ++dz) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (static_cast<double>(dx*dx + dy*dy + dz*dz) >= r2) continue;
                const int nx = cx + dx;
                const int ny = cy + dy;
                const int nz = cz + dz;
                if (!isInsideMap(map, nx, ny, nz)) return false;
                if (map.at(nx, ny, nz) != Map3D::FREE) return false;
            }
        }
    }
    return true;
}

bool isNavigableVoxel(const Map3D& map, int x, int y, int z,
                      mp_units::quantity<mp_units::si::unit_symbols::cm> sphere_radius) {
    return sphereAreaIsFree(map, x, y, z, sphere_radius);
}

// Convert one LiDAR beam into internal-map updates.
// Cells crossed by the ray become FREE; a valid hit cell becomes OCCUPIED.
// The traversal is DDA-style, so thin voxel walls are not skipped.
void markScanRay(Map3D& map,
                 const StrongPosition3D& current_position,
                 const ScanResult& result,
                 double max_dist) {
    double pos_x = current_position.x.numerical_value_in(mp_units::si::unit_symbols::cm);
    double pos_y = current_position.y.numerical_value_in(mp_units::si::unit_symbols::cm);
    double pos_z = current_position.z.numerical_value_in(mp_units::si::unit_symbols::cm);

    double dx = result.dir_x;
    double dy = result.dir_y;
    double dz = result.dir_z;
    double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len == 0.0) {
        return;
    }
    dx /= len;
    dy /= len;
    dz /= len;

    double reported_distance = result.distance;

    // A zero distance means the obstacle is too close for an accurate range.
    // Mark the first voxel outside the current cell as occupied.
    if (reported_distance <= 0.0) {
        int current_x = roundedCentimeters(current_position.x);
        int current_y = roundedCentimeters(current_position.y);
        int current_z = roundedCentimeters(current_position.z);

        double test_dist = 0.0;
        int hit_x = current_x;
        int hit_y = current_y;
        int hit_z = current_z;

        while (hit_x == current_x && hit_y == current_y && hit_z == current_z &&
               test_dist <= max_dist) {
            test_dist += VOXEL_SIZE_CM;
            hit_x = static_cast<int>(std::floor(pos_x + dx * test_dist));
            hit_y = static_cast<int>(std::floor(pos_y + dy * test_dist));
            hit_z = static_cast<int>(std::floor(pos_z + dz * test_dist));
        }

        if (isInsideMap(map, hit_x, hit_y, hit_z) &&
            !(hit_x == current_x && hit_y == current_y && hit_z == current_z)) {
            map.set(hit_x, hit_y, hit_z, Map3D::OCCUPIED);
        }
        return;
    }

    // No finite hit before max range means open air, not a wall at the end.
    bool is_valid_hit = true;
    if (!std::isfinite(reported_distance) || reported_distance >= max_dist) {
        reported_distance = max_dist;
        is_valid_hit = false; // It's just open air, do not place a wall at the end!
    }

    double effective_dist = reported_distance;

    int x = static_cast<int>(std::floor(pos_x));
    int y = static_cast<int>(std::floor(pos_y));
    int z = static_cast<int>(std::floor(pos_z));

    int step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
    int step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
    int step_z = dz > 0.0 ? 1 : (dz < 0.0 ? -1 : 0);

    double t_max_x = (step_x == 0) ? std::numeric_limits<double>::infinity()
                                   : ((step_x > 0)
                                          ? ((std::floor(pos_x) + 1.0 - pos_x) / dx)
                                          : ((pos_x - std::floor(pos_x)) / -dx));
    double t_max_y = (step_y == 0) ? std::numeric_limits<double>::infinity()
                                   : ((step_y > 0)
                                          ? ((std::floor(pos_y) + 1.0 - pos_y) / dy)
                                          : ((pos_y - std::floor(pos_y)) / -dy));
    double t_max_z = (step_z == 0) ? std::numeric_limits<double>::infinity()
                                   : ((step_z > 0)
                                          ? ((std::floor(pos_z) + 1.0 - pos_z) / dz)
                                          : ((pos_z - std::floor(pos_z)) / -dz));

    double t_delta_x = (step_x == 0) ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dx);
    double t_delta_y = (step_y == 0) ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dy);
    double t_delta_z = (step_z == 0) ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dz);

    constexpr double DDA_EPSILON = 1e-9;

    double t = 0.0;
    while (t < effective_dist - DDA_EPSILON) {
        if (!isInsideMap(map, x, y, z) || map.at(x, y, z) == Map3D::OUT_OF_BOUNDS) {
            return;
        }
        if (!is_valid_hit && map.at(x, y, z) == Map3D::OCCUPIED) {
            return;
        }
        map.set(x, y, z, Map3D::FREE);

        double next_t = 0.0;
        enum class StepAxis {
            X,
            Y,
            Z
        };
        StepAxis step_axis = StepAxis::Z;
        if (t_max_x < t_max_y && t_max_x < t_max_z) {
            next_t = t_max_x;
            step_axis = StepAxis::X;
        } else if (t_max_y < t_max_z) {
            next_t = t_max_y;
            step_axis = StepAxis::Y;
        } else {
            next_t = t_max_z;
            step_axis = StepAxis::Z;
        }

        if (!std::isfinite(next_t) || next_t > effective_dist + DDA_EPSILON) {
            return;
        }

        if (step_axis == StepAxis::X) {
            x += step_x;
            t_max_x += t_delta_x;
        } else if (step_axis == StepAxis::Y) {
            y += step_y;
            t_max_y += t_delta_y;
        } else {
            z += step_z;
            t_max_z += t_delta_z;
        }
        t = next_t;
    }

    if (!isInsideMap(map, x, y, z) ||
        map.at(x, y, z) == Map3D::OUT_OF_BOUNDS) {
        return;
    }

    if (is_valid_hit) {
        map.set(x, y, z, Map3D::OCCUPIED);
    } else if (map.at(x, y, z) != Map3D::OCCUPIED) {
        map.set(x, y, z, Map3D::FREE);
    }
}


// Strict frontier: unknown space directly touches this free voxel in one of 6 directions.
bool hasUnknownOrthogonalNeighbor(const Map3D& map, int x, int y, int z) {
    for (const auto& direction : BFS_NEIGHBOR_DIRECTIONS) {
        int nx = x + direction.dx;
        int ny = y + direction.dy;
        int nz = z + direction.dz;

        if (isInsideMap(map, nx, ny, nz) &&
            map.at(nx, ny, nz) == Map3D::UNKNOWN) {
            return true;
        }
    }

    return false;
}

// Wider frontier helper: allow nearby unknown cells only if a known wall does not block them.
bool hasLineOfSightToUnknown(const Map3D& map, int x, int y, int z, int dx, int dy, int dz) {
    int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});

    if (steps <= 1) {
        return true;
    }

    // Do not call a diagonal unknown a frontier if a known wall blocks the view.
    for (int step = 1; step < steps; ++step) {
        int ix = x + static_cast<int>(std::round(static_cast<double>(dx) * step / steps));
        int iy = y + static_cast<int>(std::round(static_cast<double>(dy) * step / steps));
        int iz = z + static_cast<int>(std::round(static_cast<double>(dz) * step / steps));

        if (!isInsideMap(map, ix, iy, iz)) {
            return false;
        }

        int value = map.at(ix, iy, iz);
        if (value == Map3D::OCCUPIED || value == Map3D::OUT_OF_BOUNDS) {
            return false;
        }
    }

    return true;
}

// Looks for nearby unknown space in a small 3D neighborhood.
bool hasUnknownMooreNeighbor(const Map3D& map, int x, int y, int z) {
    constexpr int search_radius = 2;

    for (int dz = -search_radius; dz <= search_radius; ++dz) {
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
            for (int dx = -search_radius; dx <= search_radius; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }

                int nx = x + dx;
                int ny = y + dy;
                int nz = z + dz;

                if (isInsideMap(map, nx, ny, nz) &&
                    map.at(nx, ny, nz) == Map3D::UNKNOWN &&
                    hasLineOfSightToUnknown(map, x, y, z, dx, dy, dz)) {
                    return true;
                }
            }
        }
    }

    return false;
}

// Main frontier definition used first by BFS.
bool isFrontier6Voxel(const Map3D& map, int x, int y, int z) {
    return map.at(x, y, z) == Map3D::FREE &&
           hasUnknownOrthogonalNeighbor(map, x, y, z);
}

// Fallback frontier definition used only when no strict frontier is reachable.
bool isFrontier26Voxel(const Map3D& map, int x, int y, int z) {
    return map.at(x, y, z) == Map3D::FREE &&
           hasUnknownMooreNeighbor(map, x, y, z);
}

// Angle helpers keep rotations minimal and deterministic.
double normalize_angle(double angle_deg) {
    double normalized = std::fmod(angle_deg, FULL_CIRCLE_DEG);
    if (normalized < 0.0) {
        normalized += FULL_CIRCLE_DEG;
    }
    return normalized;
}

double smallest_turn(double from_deg, double to_deg) {
    double diff = normalize_angle(to_deg) - normalize_angle(from_deg);
    if (diff > HALF_CIRCLE_DEG) {
        diff -= FULL_CIRCLE_DEG;
    } else if (diff < -HALF_CIRCLE_DEG) {
        diff += FULL_CIRCLE_DEG;
    }
    return diff;
}

DroneCommand makeRotateCommand(double angle_deg) {
    DroneCommand command;
    command.type = DroneCommandType::Rotate;
    command.angle_deg = angle_deg;
    return command;
}

DroneCommand makeAdvanceCommand(double distance_cm) {
    DroneCommand command;
    command.type = DroneCommandType::Advance;
    command.value_cm = distance_cm;
    return command;
}

DroneCommand makeElevateCommand(double height_cm) {
    DroneCommand command;
    command.type = DroneCommandType::Elevate;
    command.value_cm = height_cm;
    return command;
}

DroneCommand makeScanCommand(bool has_angles = false,
                             double azimuth_deg = 0.0,
                             double elevation_deg = 0.0) {
    DroneCommand command;
    command.type = DroneCommandType::Scan;
    command.has_angles = has_angles;
    command.azimuth_deg = azimuth_deg;
    command.elevation_deg = elevation_deg;
    return command;
}

DroneCommand makeGetLocationCommand() {
    DroneCommand command;
    command.type = DroneCommandType::GetLocation;
    return command;
}

// Split a long command into legal chunks according to the movement limits.
void appendChunkedCommand(std::vector<DroneCommand>& commands,
                          double total,
                          double max_step,
                          DroneCommand (*make_command)(double)) {
    if (std::abs(total) < COMMAND_EPSILON) {
        return;
    }
    if (max_step <= COMMAND_EPSILON) {
        commands.push_back(make_command(total));
        return;
    }
    // Respect per-command limits from the config by splitting longer moves/turns.
    double remaining = total;
    while (std::abs(remaining) > COMMAND_EPSILON) {
        double step = std::min(std::abs(remaining), max_step);
        if (remaining < 0.0) { step = -step; }
        commands.push_back(make_command(step));
        remaining -= step;
    }
}

// Build the concrete movement commands for one voxel step.
std::vector<DroneCommand> buildCommandsForStep(const StrongPosition3D& current_position,
                                               const mp_units::quantity<mp_units::si::unit_symbols::deg>& current_orientation,
                                               const StrongPosition3D& target_position,
                                               const Config& config) {
    std::vector<DroneCommand> commands;
    int cx = roundedCentimeters(current_position.x);
    int cy = roundedCentimeters(current_position.y);
    int cz = roundedCentimeters(current_position.z);
    int tx = roundedCentimeters(target_position.x);
    int ty = roundedCentimeters(target_position.y);
    int tz = roundedCentimeters(target_position.z);

    if (tz != cz) {
        appendChunkedCommand(commands,
                             static_cast<double>(tz - cz),
                             std::abs(config.max_elevation.numerical_value_in(mp_units::si::unit_symbols::cm)),
                             makeElevateCommand);
    }

    if (tx != cx || ty != cy) {
        double desired_angle = 0.0;
        if      (tx > cx) { desired_angle = static_cast<double>(FORWARD); }
        else if (tx < cx) { desired_angle = static_cast<double>(BACK); }
        else if (ty > cy) { desired_angle = static_cast<double>(RIGHT); }
        else              { desired_angle = static_cast<double>(LEFT); }

        double turn = smallest_turn(current_orientation.numerical_value_in(mp_units::si::unit_symbols::deg), desired_angle);
        if (std::abs(turn) > COMMAND_EPSILON) {
            appendChunkedCommand(commands,
                                 turn,
                                 std::abs(config.max_rotation.numerical_value_in(mp_units::si::unit_symbols::deg)),
                                 makeRotateCommand);
        }
        double distance = std::sqrt(static_cast<double>((tx - cx) * (tx - cx) + (ty - cy) * (ty - cy)));
        appendChunkedCommand(commands,
                             distance,
                             std::abs(config.max_advance.numerical_value_in(mp_units::si::unit_symbols::cm)),
                             makeAdvanceCommand);
    }

    return commands;
}

// Fixed scan batch used at each new position.
std::vector<DroneCommand> createScanCommands() {
    std::vector<DroneCommand> scans;
    scans.reserve(CARDINAL_DIRECTION_COUNT);

    scans.push_back(makeScanCommand(true, SCAN_FORWARD_RELATIVE_DEG, 0.0));
    scans.push_back(makeScanCommand(true, SCAN_RIGHT_RELATIVE_DEG, 0.0));
    scans.push_back(makeScanCommand(true, SCAN_BACK_RELATIVE_DEG, 0.0));
    scans.push_back(makeScanCommand(true, SCAN_LEFT_RELATIVE_DEG, 0.0));
    scans.push_back(makeScanCommand(true, SCAN_FORWARD_RELATIVE_DEG, ELEVATION_UP_DEG));
    scans.push_back(makeScanCommand(true, SCAN_FORWARD_RELATIVE_DEG, ELEVATION_DOWN_DEG));

    return scans;
}

// Generate the exact beam directions that the LiDAR fires for a scan command.
std::vector<Vec3> expectedBeamDirections(const Config& config,
                                         double absolute_azimuth_deg,
                                         double elevation_deg) {
    const double az_rad = absolute_azimuth_deg * DEG_TO_RAD;
    const double el_rad = elevation_deg * DEG_TO_RAD;

    Vec3 dir_central = normalize({
        std::cos(el_rad) * std::cos(az_rad),
        std::cos(el_rad) * std::sin(az_rad),
        std::sin(el_rad)
    });

    std::vector<Vec3> beam_dirs;
    beam_dirs.push_back(dir_central);

    const int fovc = std::max(1, config.lidar_fovc);
    const double z_min = config.lidar_z_min_cm.numerical_value_in(mp_units::si::unit_symbols::cm);
    const double d = config.lidar_d_cm.numerical_value_in(mp_units::si::unit_symbols::cm);

    Vec3 perp1;
    if (std::fabs(dir_central.x) > AXIS_ALIGNMENT_EPSILON ||
        std::fabs(dir_central.z) > AXIS_ALIGNMENT_EPSILON) {
        perp1 = normalize(cross(dir_central, {0.0, 1.0, 0.0}));
    } else {
        perp1 = normalize(cross(dir_central, {1.0, 0.0, 0.0}));
    }
    Vec3 perp2 = normalize(cross(dir_central, perp1));

    for (int i = FIRST_FOV_RING; i < fovc; ++i) {
        int num_beams_circle = 1;
        for (int p = 0; p < i; ++p) {
            num_beams_circle *= BEAM_BRANCHING_FACTOR;
        }

        const double radius = i * d;
        for (int j = 0; j < num_beams_circle; ++j) {
            const double theta = j * FULL_CIRCLE_RAD / static_cast<double>(num_beams_circle);
            const Vec3 offset = radius * (std::cos(theta) * perp1 + std::sin(theta) * perp2);
            const Vec3 target_from_origin = z_min * dir_central + offset;
            beam_dirs.push_back(normalize(target_from_origin));
        }
    }

    return beam_dirs;
}

ScanResult makeMissedBeamResult(const Vec3& direction, double max_distance) {
    const Vec3 dir = normalize(direction);
    return ScanResult{
        max_distance,
        std::atan2(dir.y, dir.x) * RAD_TO_DEG,
        std::asin(std::clamp(dir.z, MIN_DIRECTION_COMPONENT, MAX_DIRECTION_COMPONENT)) * RAD_TO_DEG,
        dir.x,
        dir.y,
        dir.z
    };
}

// Point the LiDAR at a specific UNKNOWN voxel that blocks safe movement.
std::optional<DroneCommand> makeTargetedScanCommand(const Map3D& map,
                                                    const StrongPosition3D& current_position,
                                                    const mp_units::quantity<mp_units::si::unit_symbols::deg>& current_orientation,
                                                    int target_x,
                                                    int target_y,
                                                    int target_z) {
    if (!isInsideMap(map, target_x, target_y, target_z) ||
        map.at(target_x, target_y, target_z) != Map3D::UNKNOWN) {
        return std::nullopt;
    }

    const double current_x = current_position.x.numerical_value_in(mp_units::si::unit_symbols::cm);
    const double current_y = current_position.y.numerical_value_in(mp_units::si::unit_symbols::cm);
    const double current_z = current_position.z.numerical_value_in(mp_units::si::unit_symbols::cm);

    const double vx = static_cast<double>(target_x) - current_x;
    const double vy = static_cast<double>(target_y) - current_y;
    const double vz = static_cast<double>(target_z) - current_z;
    const double horizontal_distance = std::sqrt(vx * vx + vy * vy);

    const double absolute_azimuth = std::atan2(vy, vx) * RAD_TO_DEG;
    const double current_azimuth = current_orientation.numerical_value_in(mp_units::si::unit_symbols::deg);
    const double relative_azimuth = smallest_turn(current_azimuth, absolute_azimuth);
    const double elevation = std::atan2(vz, horizontal_distance) * RAD_TO_DEG;

    // The simulator applies scan angles relative to the current heading.
    return makeScanCommand(true, relative_azimuth, elevation);
}

} // namespace

Drone::Drone(size_t map_width,
             size_t map_height,
             size_t map_depth,
             const Config& config,
             const MissionConfig& mission_config,
             ILidarSensor& lidar,
             IPositionSensor& position_sensor,
             IMovementDriver& driver)
    : current_position_{0.0 * mp_units::si::unit_symbols::cm,
                        0.0 * mp_units::si::unit_symbols::cm,
                        0.0 * mp_units::si::unit_symbols::cm},
      orientation_(0.0 * mp_units::si::unit_symbols::deg),
      internal_map_(map_width, map_height, map_depth),
      config_(config),
      mission_config_(mission_config),
      lidar_(&lidar),
      position_sensor_(&position_sensor),
      driver_(&driver),
      sphere_radius_(computeSphereRadius(config)) {
    if (mission_config_.has_bounds) {
        internal_map_.setMissionBounds(roundedCentimeters(mission_config_.min_x),
                                       roundedCentimeters(mission_config_.max_x),
                                       roundedCentimeters(mission_config_.min_y),
                                       roundedCentimeters(mission_config_.max_y),
                                       roundedCentimeters(mission_config_.min_height),
                                       roundedCentimeters(mission_config_.max_height));
        internal_map_.fillOutOfBoundsVoxels();
    }
}

// Called after GetLocation. Keeps Drone's cached pose in sync with the simulator.
void Drone::updatePose(const Pose3D& pose) {
    current_position_ = pose.position;
    orientation_ = pose.orientation;
    require_location_update_ = false;
    internal_map_.set(roundedCentimeters(current_position_.x),
                      roundedCentimeters(current_position_.y),
                      roundedCentimeters(current_position_.z),
                      Map3D::FREE);
    if (!current_path_.empty()) {
        StrongPosition3D current_step = current_path_.front();
        if (roundedCentimeters(current_step.x) == roundedCentimeters(current_position_.x) &&
            roundedCentimeters(current_step.y) == roundedCentimeters(current_position_.y) &&
            roundedCentimeters(current_step.z) == roundedCentimeters(current_position_.z)) {
            consumeCurrentPathStep();
        }
    }
}

// Called after Scan. This compatibility overload processes only the beams that were returned.
void Drone::processScanResults(const std::vector<ScanResult>& results) {
    processScanResultsForExecutedScan(
        results,
        orientation_.numerical_value_in(mp_units::si::unit_symbols::deg),
        0.0);
}

void Drone::processScanResultsForExecutedScan(const std::vector<ScanResult>& results,
                                              double absolute_azimuth_deg,
                                              double elevation_deg) {
    internal_map_.set(roundedCentimeters(current_position_.x),
                      roundedCentimeters(current_position_.y),
                      roundedCentimeters(current_position_.z),
                      Map3D::FREE);

    const double max_distance = config_.lidar_z_max_cm.numerical_value_in(mp_units::si::unit_symbols::cm);
    const auto expected_beams = expectedBeamDirections(config_, absolute_azimuth_deg, elevation_deg);
    std::vector<bool> used_hits(results.size(), false);

    for (const auto& expected : expected_beams) {
        const ScanResult* hit = nullptr;
        for (size_t i = 0; i < results.size(); ++i) {
            if (!used_hits[i] && sameBeamDirection(results[i], expected)) {
                used_hits[i] = true;
                hit = &results[i];
                break;
            }
        }

        if (hit != nullptr) {
            markScanRay(internal_map_, current_position_, *hit, max_distance);
        } else {
            markScanRay(internal_map_, current_position_, makeMissedBeamResult(expected, max_distance), max_distance);
        }
    }

    preserveVisitedPositionsAsFree();
    if (scan_batch_completion_pending_) {
        finishCurrentScanBatch();
    }
}

// Main state-machine entry point: return exactly one command per call.
DroneCommand Drone::getNextCommand() {
    if (require_location_update_) {
        return makeGetLocationCommand();
    }

    switch (state_) {
        case ExplorationState::Scanning:
            return nextScanningCommand();
        case ExplorationState::Planning:
            return nextPlanningCommand();
        case ExplorationState::Moving:
            return nextMovingCommand();
        case ExplorationState::Finished:
            return DroneCommand{DroneCommandType::Finished};
    }

    state_ = ExplorationState::Finished;
    return DroneCommand{DroneCommandType::Finished};
}

// Executes one full autonomous step through the injected sensor/driver interfaces.
// main.cpp remains the simulator manager; it no longer calls the concrete mocks or
// the interfaces directly for command execution.
DroneCommand Drone::executeNextAction() {
    DroneCommand command = getNextCommand();

    switch (command.type) {
        case DroneCommandType::GetLocation: {
            if (position_sensor_ == nullptr) {
                command.succeeded = false;
                state_ = ExplorationState::Finished;
                return command;
            }
            updatePose(position_sensor_->getPosition());
            return command;
        }

        case DroneCommandType::Scan: {
            if (lidar_ == nullptr) {
                command.succeeded = false;
                state_ = ExplorationState::Finished;
                return command;
            }

            const double current_azimuth =
                orientation_.numerical_value_in(mp_units::si::unit_symbols::deg);
            const double relative_azimuth = command.has_angles ? command.azimuth_deg : 0.0;
            const double azimuth = current_azimuth + relative_azimuth;
            const double elevation = command.has_angles ? command.elevation_deg : 0.0;

            auto results = lidar_->scan(
                current_position_,
                azimuth * mp_units::si::unit_symbols::deg,
                elevation * mp_units::si::unit_symbols::deg
            );

            const double max_distance =
                config_.lidar_z_max_cm.numerical_value_in(mp_units::si::unit_symbols::cm);
            const int hits = static_cast<int>(
                std::count_if(results.begin(), results.end(),
                              [max_distance](const ScanResult& result) {
                                  return result.distance < max_distance;
                              }));

            command.executed_azimuth_deg = azimuth;
            command.executed_elevation_deg = elevation;
            command.scan_beams = expectedBeamDirections(config_, azimuth, elevation).size();
            command.scan_hits = hits;
            command.scan_open = static_cast<int>(command.scan_beams) - hits;

            processScanResultsForExecutedScan(results, azimuth, elevation);
            return command;
        }

        case DroneCommandType::Rotate: {
            if (driver_ == nullptr || position_sensor_ == nullptr) {
                command.succeeded = false;
                state_ = ExplorationState::Finished;
                return command;
            }

            try {
                command.succeeded = driver_->rotate(
                    command.angle_deg * mp_units::si::unit_symbols::deg);
            } catch (const std::invalid_argument&) {
                command.succeeded = false;
            }
            if (!command.succeeded) {
                command.collision_detected = true;
                state_ = ExplorationState::Finished;
                return command;
            }
            updatePose(position_sensor_->getPosition());
            return command;
        }

        case DroneCommandType::Advance: {
            if (driver_ == nullptr || position_sensor_ == nullptr) {
                command.succeeded = false;
                state_ = ExplorationState::Finished;
                return command;
            }

            try {
                command.succeeded = driver_->moveForward(
                    command.value_cm * mp_units::si::unit_symbols::cm);
            } catch (const std::invalid_argument&) {
                command.succeeded = false;
            }
            if (!command.succeeded) {
                command.collision_detected = true;
                state_ = ExplorationState::Finished;
                return command;
            }
            updatePose(position_sensor_->getPosition());
            return command;
        }

        case DroneCommandType::Elevate: {
            if (driver_ == nullptr || position_sensor_ == nullptr) {
                command.succeeded = false;
                state_ = ExplorationState::Finished;
                return command;
            }

            try {
                command.succeeded = driver_->elevate(
                    command.value_cm * mp_units::si::unit_symbols::cm);
            } catch (const std::invalid_argument&) {
                command.succeeded = false;
            }
            if (!command.succeeded) {
                command.collision_detected = true;
                state_ = ExplorationState::Finished;
                return command;
            }
            updatePose(position_sensor_->getPosition());
            return command;
        }

        case DroneCommandType::Finished:
            return command;
    }

    state_ = ExplorationState::Finished;
    return DroneCommand{DroneCommandType::Finished};
}

// Scanning state: issue the fixed scan batch, then move to planning.
DroneCommand Drone::nextScanningCommand() {
    if (pending_commands_.empty() && scan_batch_completion_pending_) {
        finishCurrentScanBatch();
        if (state_ == ExplorationState::Planning) {
            return nextPlanningCommand();
        }
    }

    if (pending_commands_.empty() && currentPositionWasVisited()) {
        state_ = ExplorationState::Planning;
        return nextPlanningCommand();
    }

    if (pending_commands_.empty()) {
        enqueueScanCommands();
    }

    DroneCommand next = pending_commands_.front();
    pending_commands_.pop_front();
    if (pending_commands_.empty()) {
        scan_batch_completion_pending_ = true;
    }
    return next;
}

// Planning state:
// 1. continue an existing BFS path if still safe,
// 2. try a cheap local sweep,
// 3. run BFS to Frontier6 and then Frontier26,
// 4. try a targeted scan before finishing.
DroneCommand Drone::nextPlanningCommand() {
    // 1. Continue an existing BFS path if the next step is still safe.
    while (!current_path_.empty()) {
        StrongPosition3D next_step = current_path_.front();
        if (!targetIsSafeForDrone(next_step)) {
            if (enqueueTargetedScanForTarget(next_step)) {
                current_path_.clear();
                state_ = ExplorationState::Scanning;
                return nextScanningCommand();
            }
            current_path_.clear();
            break;
        }

        auto step_commands = buildCommandsForStep(current_position_, orientation_, next_step, config_);
        if (!step_commands.empty()) {
            pending_commands_.insert(pending_commands_.end(), step_commands.begin(), step_commands.end());
            state_ = ExplorationState::Moving;
            return nextMovingCommand();
        }
        current_path_.clear();
        break;
    }

    // 2. Cheap local sweep before running BFS.
    if (enqueueSweepMove()) {
        state_ = ExplorationState::Moving;
        return nextMovingCommand();
    }

    // 3. BFS recovery: find the nearest reachable simple frontier.
    auto path = bfs_to_goal(BfsGoalMode::Frontier6);
    if (path.empty()) {
        path = bfs_to_goal(BfsGoalMode::Frontier26);
    }

    if (!path.empty()) {
        current_path_ = std::move(path);
        return nextPlanningCommand();
    }

    // 4. Before giving up, try a targeted scan around the current position.
    if (enqueueTargetedScanAroundCurrentPosition()) {
        state_ = ExplorationState::Scanning;
        return nextScanningCommand();
    }

    state_ = ExplorationState::Finished;
    return DroneCommand{DroneCommandType::Finished};
}

// Moving state: replay queued Rotate/Advance/Elevate commands.
DroneCommand Drone::nextMovingCommand() {
    if (!pending_commands_.empty()) {
        DroneCommand next = pending_commands_.front();
        pending_commands_.pop_front();
        if (pending_commands_.empty()) {
            state_ = ExplorationState::Scanning;
        }
        return next;
    }

    state_ = ExplorationState::Scanning;
    return nextScanningCommand();
}

// Queue the next fixed scan batch.
void Drone::enqueueScanCommands() {
    scan_batch_completion_pending_ = false;
    auto scans = createScanCommands();
    pending_commands_.insert(pending_commands_.end(), scans.begin(), scans.end());
}

// Safety check used by planning before committing to a target voxel.
bool Drone::targetIsSafeForDrone(const StrongPosition3D& position) const {
    int x = roundedCentimeters(position.x);
    int y = roundedCentimeters(position.y);
    int z = roundedCentimeters(position.z);
    return isNavigableVoxel(internal_map_, x, y, z, sphere_radius_);
}

// A BFS goal must be a frontier and must not be a position already scanned.
bool Drone::isBfsGoal(int x, int y, int z, BfsGoalMode goal_mode) const {
    if (internal_map_.at(x, y, z) != Map3D::FREE) {
        return false;
    }

    auto key = std::make_tuple(x, y, z);
    if (visited_positions_.find(key) != visited_positions_.end()) {
        return false;
    }

    switch (goal_mode) {
        case BfsGoalMode::Frontier6:
            return isFrontier6Voxel(internal_map_, x, y, z);
        case BfsGoalMode::Frontier26:
            return isFrontier26Voxel(internal_map_, x, y, z);
    }

    return false;
}

bool Drone::currentPositionWasVisited() const {
    return visited_positions_.find({
        roundedCentimeters(current_position_.x),
        roundedCentimeters(current_position_.y),
        roundedCentimeters(current_position_.z)
    }) != visited_positions_.end();
}

// Once all scan commands were processed, mark this voxel as scanned and plan next.
void Drone::finishCurrentScanBatch() {
    scan_batch_completion_pending_ = false;
    markCurrentPositionVisited();
    state_ = ExplorationState::Planning;
}

void Drone::markCurrentPositionVisited() {
    visited_positions_.insert({
        roundedCentimeters(current_position_.x),
        roundedCentimeters(current_position_.y),
        roundedCentimeters(current_position_.z)
    });
    preserveVisitedPositionsAsFree();
}

// If we already stood in a voxel, keep it FREE even if a nearby scan is noisy.
void Drone::preserveVisitedPositionsAsFree() {
    for (const auto& position : visited_positions_) {
        auto [x, y, z] = position;
        internal_map_.set(x, y, z, Map3D::FREE);
    }
    internal_map_.set(roundedCentimeters(current_position_.x),
                      roundedCentimeters(current_position_.y),
                      roundedCentimeters(current_position_.z),
                      Map3D::FREE);
}

// Local exploration step. This is cheaper than BFS and helps avoid planning every move.
bool Drone::enqueueSweepMove() {
    const int current_x = roundedCentimeters(current_position_.x);
    const int current_y = roundedCentimeters(current_position_.y);
    const int current_z = roundedCentimeters(current_position_.z);

    for (const auto& direction : SWEEP_DIRECTIONS) {
        const int next_x = current_x + direction.dx;
        const int next_y = current_y + direction.dy;
        const int next_z = current_z + direction.dz;

        if (!isInsideMap(internal_map_, next_x, next_y, next_z)) {
            continue;
        }
        if (visited_positions_.find({next_x, next_y, next_z}) != visited_positions_.end()) {
            continue;
        }
        if (!isNavigableVoxel(internal_map_, next_x, next_y, next_z, sphere_radius_)) {
            continue;
        }

        StrongPosition3D target = positionFromVoxel(next_x, next_y, next_z);
        auto step_commands = buildCommandsForStep(current_position_, orientation_, target, config_);
        if (!step_commands.empty()) {
            pending_commands_.insert(pending_commands_.end(), step_commands.begin(), step_commands.end());
            return true;
        }
    }

    return false;
}

// If the target is blocked only because its safety sphere contains UNKNOWN cells,
// scan one of those cells directly.
bool Drone::enqueueTargetedScanForTarget(const StrongPosition3D& target_position) {
    const int target_x = roundedCentimeters(target_position.x);
    const int target_y = roundedCentimeters(target_position.y);
    const int target_z = roundedCentimeters(target_position.z);

    const double r_cm = sphere_radius_.numerical_value_in(mp_units::si::unit_symbols::cm);
    const int radius_cells = static_cast<int>(std::ceil(r_cm));
    const double radius_squared = r_cm * r_cm;

    for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if (static_cast<double>(dx * dx + dy * dy + dz * dz) >= radius_squared) {
                    continue;
                }

                const int x = target_x + dx;
                const int y = target_y + dy;
                const int z = target_z + dz;

                auto command = makeTargetedScanCommand(internal_map_, current_position_, orientation_, x, y, z);
                if (command.has_value()) {
                    pending_commands_.push_back(*command);
                    return true;
                }
            }
        }
    }

    return false;
}

// Last recovery attempt before finishing: scan a directly adjacent unknown cell.
bool Drone::enqueueTargetedScanAroundCurrentPosition() {
    const int current_x = roundedCentimeters(current_position_.x);
    const int current_y = roundedCentimeters(current_position_.y);
    const int current_z = roundedCentimeters(current_position_.z);

    for (const auto& direction : BFS_NEIGHBOR_DIRECTIONS) {
        const int x = current_x + direction.dx;
        const int y = current_y + direction.dy;
        const int z = current_z + direction.dz;

        auto command = makeTargetedScanCommand(internal_map_, current_position_, orientation_, x, y, z);
        if (command.has_value()) {
            pending_commands_.push_back(*command);
            return true;
        }
    }

    return false;
}

// Remove the path step once the simulator confirms that we reached it.
void Drone::consumeCurrentPathStep() {
    if (!current_path_.empty()) {
        current_path_.erase(current_path_.begin());
    }
}

// BFS over known navigable voxels. Returns a shortest voxel-step path to a frontier.
std::vector<StrongPosition3D> Drone::bfs_to_goal(BfsGoalMode goal_mode) const {
    std::vector<StrongPosition3D> path;
    int sx = roundedCentimeters(current_position_.x);
    int sy = roundedCentimeters(current_position_.y);
    int sz = roundedCentimeters(current_position_.z);

    if (!isInsideMap(internal_map_, sx, sy, sz) ||
        internal_map_.at(sx, sy, sz) != Map3D::FREE) {
        return path;
    }

    size_t w = internal_map_.width();
    size_t h = internal_map_.height();
    size_t d = internal_map_.depth();
    size_t total = w * h * d;

    std::vector<bool> visited(total, false);
    std::vector<int> parent(total, -1);

    auto to_index = [w, h, d](int x, int y, int z) -> size_t {
        return static_cast<size_t>(x) * h * d + static_cast<size_t>(y) * d + static_cast<size_t>(z);
    };

    auto from_index = [w, h, d](size_t idx) -> std::tuple<int, int, int> {
        int z = idx % d;
        int y = (idx / d) % h;
        int x = idx / (h * d);
        return {x, y, z};
    };

    std::queue<std::tuple<int, int, int>> q;
    int start_idx = to_index(sx, sy, sz);
    q.push({sx, sy, sz});
    visited[start_idx] = true;

    int goal_idx = -1;
    while (!q.empty()) {
        auto [cx, cy, cz] = q.front();
        q.pop();

        bool is_current_position = cx == sx && cy == sy && cz == sz;
        if (!is_current_position && isBfsGoal(cx, cy, cz, goal_mode)) {
            goal_idx = to_index(cx, cy, cz);
            break;
        }

        for (const auto& direction : BFS_NEIGHBOR_DIRECTIONS) {
            int nx = cx + direction.dx;
            int ny = cy + direction.dy;
            int nz = cz + direction.dz;
            if (isInsideMap(internal_map_, nx, ny, nz)) {
                size_t idx = to_index(nx, ny, nz);
                if (!visited[idx] && isNavigableVoxel(internal_map_, nx, ny, nz, sphere_radius_)) {
                    visited[idx] = true;
                    parent[idx] = to_index(cx, cy, cz);
                    q.push({nx, ny, nz});
                }
            }
        }
    }

    // Reconstruct path by backtracking through parent pointers.
    if (goal_idx >= 0) {
        std::vector<StrongPosition3D> reverse_path;
        int idx = goal_idx;
        while (idx != start_idx && idx >= 0) {
            auto [x, y, z] = from_index(idx);
            reverse_path.push_back(positionFromVoxel(x, y, z));
            idx = parent[idx];
        }
        std::reverse(reverse_path.begin(), reverse_path.end());
        path = reverse_path;
    }

    return path;
}
