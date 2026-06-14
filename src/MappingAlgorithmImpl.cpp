#include <drone_mapper/MappingAlgorithmImpl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#include <mp-units/systems/si/math.h>

namespace drone_mapper {
namespace {

constexpr double kHalfCircleDeg = 180.0;
constexpr double kFullCircleDeg = 360.0;
constexpr double kEpsilon = 1e-6;

struct GridOffset { int dx; int dy; int dz; };

constexpr std::array<GridOffset, 6> kBfsNeighbors{{
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
}};

constexpr std::array<GridOffset, 6> kSweepDirections{{
    {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
}};

double normalizeAngle(double angle_deg) {
    double normalized = std::fmod(angle_deg, kFullCircleDeg);
    if (normalized < 0.0) {
        normalized += kFullCircleDeg;
    }
    return normalized;
}

double smallestTurn(double from_deg, double to_deg) {
    double diff = normalizeAngle(to_deg) - normalizeAngle(from_deg);
    if (diff > kHalfCircleDeg) {
        diff -= kFullCircleDeg;
    } else if (diff < -kHalfCircleDeg) {
        diff += kFullCircleDeg;
    }
    return diff;
}

types::MovementCommand makeHover() {
    return types::MovementCommand{};
}

types::MovementCommand makeRotate(double angle_deg) {
    types::MovementCommand cmd{};
    cmd.type = types::MovementCommandType::Rotate;
    cmd.rotation = angle_deg >= 0.0 ? types::RotationDirection::Left : types::RotationDirection::Right;
    cmd.angle = std::abs(angle_deg) * horizontal_angle[deg];
    return cmd;
}

types::MovementCommand makeAdvance(double distance_cm) {
    types::MovementCommand cmd{};
    cmd.type = types::MovementCommandType::Advance;
    cmd.distance = distance_cm * cm;
    return cmd;
}

types::MovementCommand makeElevate(double distance_cm) {
    types::MovementCommand cmd{};
    cmd.type = types::MovementCommandType::Elevate;
    cmd.distance = distance_cm * cm;
    return cmd;
}

template <typename MakeCommand>
void appendChunked(std::deque<types::MovementCommand>& commands,
                   double total,
                   double max_step,
                   MakeCommand make_command) {
    if (std::abs(total) < kEpsilon) {
        return;
    }
    if (max_step <= kEpsilon) {
        commands.push_back(make_command(total));
        return;
    }
    double remaining = total;
    while (std::abs(remaining) > kEpsilon) {
        double step = std::min(std::abs(remaining), max_step);
        if (remaining < 0.0) {
            step = -step;
        }
        commands.push_back(make_command(step));
        remaining -= step;
    }
}

} // namespace

MappingAlgorithmImpl::MappingAlgorithmImpl(types::MissionConfigData mission)
    : mission_(std::move(mission)) {}

MappingAlgorithmImpl::MappingAlgorithmImpl(types::MissionConfigData mission, types::DroneConfigData drone)
    : mission_(std::move(mission)), drone_(std::move(drone)), has_drone_config_(true) {}

double MappingAlgorithmImpl::cellSizeCm() const {
    const double gps_res = mission_.gps_resolution.force_numerical_value_in(cm);
    return gps_res > 0.0 ? gps_res : 1.0;
}

MappingAlgorithmImpl::GridKey MappingAlgorithmImpl::toGrid(const Position3D& position) const {
    const double cell = cellSizeCm();
    return GridKey{
        static_cast<int>(std::llround(position.x.force_numerical_value_in(cm) / cell)),
        static_cast<int>(std::llround(position.y.force_numerical_value_in(cm) / cell)),
        static_cast<int>(std::llround(position.z.force_numerical_value_in(cm) / cell)),
    };
}

Position3D MappingAlgorithmImpl::toPosition(const GridKey& key) const {
    const double cell = cellSizeCm();
    return Position3D{
        static_cast<double>(key.x) * cell * x_extent[cm],
        static_cast<double>(key.y) * cell * y_extent[cm],
        static_cast<double>(key.z) * cell * z_extent[cm],
    };
}

types::VoxelOccupancy MappingAlgorithmImpl::at(const GridKey& key) const {
    const auto it = known_voxels_.find(key);
    return it == known_voxels_.end() ? types::VoxelOccupancy::Unmapped : it->second;
}

void MappingAlgorithmImpl::setKnown(const GridKey& key, types::VoxelOccupancy value) {
    const auto current = at(key);
    if (current == types::VoxelOccupancy::Empty && value == types::VoxelOccupancy::Unmapped) {
        return;
    }
    known_voxels_[key] = value;
}

bool MappingAlgorithmImpl::isInsideMissionBounds(const GridKey& key) const {
    const auto& b = mission_.boundaries;
    const bool bounds_unset = b.min_x.force_numerical_value_in(cm) == 0.0 &&
                              b.max_x.force_numerical_value_in(cm) == 0.0 &&
                              b.min_y.force_numerical_value_in(cm) == 0.0 &&
                              b.max_y.force_numerical_value_in(cm) == 0.0 &&
                              b.min_height.force_numerical_value_in(cm) == 0.0 &&
                              b.max_height.force_numerical_value_in(cm) == 0.0;
    if (bounds_unset) {
        return true;
    }
    const Position3D pos = toPosition(key);
    return pos.x.force_numerical_value_in(cm) >= b.min_x.force_numerical_value_in(cm) &&
           pos.x.force_numerical_value_in(cm) <= b.max_x.force_numerical_value_in(cm) &&
           pos.y.force_numerical_value_in(cm) >= b.min_y.force_numerical_value_in(cm) &&
           pos.y.force_numerical_value_in(cm) <= b.max_y.force_numerical_value_in(cm) &&
           pos.z.force_numerical_value_in(cm) >= b.min_height.force_numerical_value_in(cm) &&
           pos.z.force_numerical_value_in(cm) <= b.max_height.force_numerical_value_in(cm);
}

bool MappingAlgorithmImpl::isNavigable(const GridKey& key) const {
    if (!isInsideMissionBounds(key)) {
        return false;
    }
    const auto value = at(key);
    return value == types::VoxelOccupancy::Empty || value == types::VoxelOccupancy::Unmapped;
}

void MappingAlgorithmImpl::markCurrentVisited() {
    const GridKey here = toGrid(current_position_);
    visited_positions_.insert(std::make_tuple(here.x, here.y, here.z));
    setKnown(here, types::VoxelOccupancy::Empty);
}

void MappingAlgorithmImpl::ingestScan(const types::DroneState& state, const types::LidarScanResult& scan) {
    const GridKey here = toGrid(state.position);
    setKnown(here, types::VoxelOccupancy::Empty);

    for (const auto& hit : scan) {
        const Orientation abs_or{
            HorizontalAngle{state.heading.horizontal + hit.angle.horizontal},
            AltitudeAngle{state.heading.altitude + hit.angle.altitude},
        };
        const auto cos_alt = mp_units::si::cos(abs_or.altitude);
        const double dx = (cos_alt * mp_units::si::cos(abs_or.horizontal)).force_numerical_value_in(mp_units::one);
        const double dy = (cos_alt * mp_units::si::sin(abs_or.horizontal)).force_numerical_value_in(mp_units::one);
        const double dz = mp_units::si::sin(abs_or.altitude).force_numerical_value_in(mp_units::one);
        double distance_cm = hit.distance.force_numerical_value_in(cm);
        if (!std::isfinite(distance_cm) || distance_cm < 0.0) {
            continue;
        }
        const double step = std::max(1.0, cellSizeCm() * 0.5);
        for (double t = 0.0; t < distance_cm; t += step) {
            const Position3D sample{
                state.position.x + dx * t * x_extent[cm],
                state.position.y + dy * t * y_extent[cm],
                state.position.z + dz * t * z_extent[cm],
            };
            setKnown(toGrid(sample), types::VoxelOccupancy::Empty);
        }
        const Position3D hit_pos{
            state.position.x + dx * distance_cm * x_extent[cm],
            state.position.y + dy * distance_cm * y_extent[cm],
            state.position.z + dz * distance_cm * z_extent[cm],
        };
        setKnown(toGrid(hit_pos), types::VoxelOccupancy::Occupied);
    }
}

void MappingAlgorithmImpl::applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) {
    for (const auto& voxel : voxels) {
        setKnown(toGrid(voxel.position), voxel.value);
    }
}

bool MappingAlgorithmImpl::hasUnknownOrthogonalNeighbor(const GridKey& key) const {
    for (const auto& d : kBfsNeighbors) {
        const GridKey n{key.x + d.dx, key.y + d.dy, key.z + d.dz};
        if (isInsideMissionBounds(n) && at(n) == types::VoxelOccupancy::Unmapped) {
            return true;
        }
    }
    return false;
}

bool MappingAlgorithmImpl::hasUnknownMooreNeighbor(const GridKey& key) const {
    constexpr int radius = 2;
    for (int dz = -radius; dz <= radius; ++dz) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const GridKey n{key.x + dx, key.y + dy, key.z + dz};
                if (isInsideMissionBounds(n) && at(n) == types::VoxelOccupancy::Unmapped) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool MappingAlgorithmImpl::isBfsGoal(const GridKey& key, BfsGoalMode mode) const {
    if (at(key) != types::VoxelOccupancy::Empty) {
        return false;
    }
    if (visited_positions_.contains(std::make_tuple(key.x, key.y, key.z))) {
        return false;
    }
    return mode == BfsGoalMode::Frontier6 ? hasUnknownOrthogonalNeighbor(key) : hasUnknownMooreNeighbor(key);
}

std::vector<MappingAlgorithmImpl::GridKey> MappingAlgorithmImpl::bfsToGoal(BfsGoalMode mode) const {
    const GridKey start = toGrid(current_position_);
    std::queue<GridKey> q;
    std::set<std::tuple<int, int, int>> visited;
    std::map<GridKey, GridKey> parent;

    q.push(start);
    visited.insert({start.x, start.y, start.z});

    std::optional<GridKey> goal;
    while (!q.empty()) {
        const GridKey cur = q.front();
        q.pop();

        if (!(cur == start) && isBfsGoal(cur, mode)) {
            goal = cur;
            break;
        }

        for (const auto& d : kBfsNeighbors) {
            const GridKey next{cur.x + d.dx, cur.y + d.dy, cur.z + d.dz};
            const auto key_tuple = std::make_tuple(next.x, next.y, next.z);
            if (visited.contains(key_tuple) || !isNavigable(next)) {
                continue;
            }
            visited.insert(key_tuple);
            parent[next] = cur;
            q.push(next);
        }
    }

    std::vector<GridKey> path;
    if (!goal.has_value()) {
        return path;
    }

    GridKey cur = *goal;
    while (!(cur == start)) {
        path.push_back(cur);
        cur = parent.at(cur);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void MappingAlgorithmImpl::enqueueCommandsForStep(const GridKey& target) {
    const GridKey here = toGrid(current_position_);
    const double cell = cellSizeCm();

    if (target.z != here.z) {
        const double max_elevate = has_drone_config_ ? std::abs(drone_.max_elevate.force_numerical_value_in(cm)) : cell;
        appendChunked(pending_commands_, static_cast<double>(target.z - here.z) * cell, max_elevate, makeElevate);
    }

    if (target.x != here.x || target.y != here.y) {
        double desired = 0.0;
        if (target.x > here.x) {
            desired = 0.0;
        } else if (target.x < here.x) {
            desired = 180.0;
        } else if (target.y > here.y) {
            desired = 90.0;
        } else {
            desired = 270.0;
        }
        const double turn = smallestTurn(orientation_.horizontal.force_numerical_value_in(deg), desired);
        const double max_rotate = has_drone_config_ ? std::abs(drone_.max_rotate.force_numerical_value_in(deg)) : 90.0;
        appendChunked(pending_commands_, turn, max_rotate, makeRotate);

        const double dx = static_cast<double>(target.x - here.x) * cell;
        const double dy = static_cast<double>(target.y - here.y) * cell;
        const double dist = std::sqrt(dx * dx + dy * dy);
        const double max_advance = has_drone_config_ ? std::abs(drone_.max_advance.force_numerical_value_in(cm)) : cell;
        appendChunked(pending_commands_, dist, max_advance, makeAdvance);
    }
}

bool MappingAlgorithmImpl::enqueueSweepMove() {
    const GridKey here = toGrid(current_position_);
    for (const auto& d : kSweepDirections) {
        const GridKey next{here.x + d.dx, here.y + d.dy, here.z + d.dz};
        if (visited_positions_.contains(std::make_tuple(next.x, next.y, next.z))) {
            continue;
        }
        if (!isNavigable(next)) {
            continue;
        }
        enqueueCommandsForStep(next);
        return !pending_commands_.empty();
    }
    return false;
}

types::MovementCommand MappingAlgorithmImpl::nextMovingCommand() {
    if (!pending_commands_.empty()) {
        auto cmd = pending_commands_.front();
        pending_commands_.pop_front();
        if (pending_commands_.empty()) {
            state_ = ExplorationState::Planning;
        }
        return cmd;
    }
    state_ = ExplorationState::Planning;
    return nextPlanningCommand();
}

types::MovementCommand MappingAlgorithmImpl::nextPlanningCommand() {
    markCurrentVisited();

    while (!current_path_.empty()) {
        const GridKey next = current_path_.front();
        current_path_.erase(current_path_.begin());
        if (!isNavigable(next)) {
            current_path_.clear();
            break;
        }
        enqueueCommandsForStep(next);
        if (!pending_commands_.empty()) {
            state_ = ExplorationState::Moving;
            return nextMovingCommand();
        }
    }

    if (enqueueSweepMove()) {
        state_ = ExplorationState::Moving;
        return nextMovingCommand();
    }

    auto path = bfsToGoal(BfsGoalMode::Frontier6);
    if (path.empty()) {
        path = bfsToGoal(BfsGoalMode::Frontier26);
    }
    if (!path.empty()) {
        current_path_ = std::move(path);
        return nextPlanningCommand();
    }

    state_ = ExplorationState::Finished;
    return makeHover();
}

types::MovementCommand MappingAlgorithmImpl::nextMove(const types::DroneState& state,
                                                      const types::LidarScanResult& latest_scan) {
    current_position_ = state.position;
    orientation_ = state.heading;
    ingestScan(state, latest_scan);

    if (state_ == ExplorationState::Finished) {
        return makeHover();
    }
    if (state_ == ExplorationState::Moving) {
        return nextMovingCommand();
    }
    return nextPlanningCommand();
}

} // namespace drone_mapper
