#include <drone_mapper/MappingAlgorithmImpl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>

#include <mp-units/systems/si/math.h>

namespace drone_mapper {
namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────
constexpr double kHalfCircleDeg = 180.0;
constexpr double kFullCircleDeg = 360.0;
constexpr double kEpsilon       = 1e-6;

struct GridOffset { int dx; int dy; int dz; };

constexpr std::array<GridOffset, 6> kBfsNeighbors{{
    {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
}};
constexpr std::array<GridOffset, 6> kSweepDirections{{
    {1,0,0},{0,1,0},{-1,0,0},{0,-1,0},{0,0,1},{0,0,-1},
}};

// ─────────────────────────────────────────────────────────────────────────────
// Angle helpers — FIX 5: use HorizontalAngle strong type, not raw double
// ─────────────────────────────────────────────────────────────────────────────
[[nodiscard]] double normalizeDeg(double d) {
    d = std::fmod(d, kFullCircleDeg);
    return d < 0.0 ? d + kFullCircleDeg : d;
}

[[nodiscard]] HorizontalAngle smallestTurn(HorizontalAngle from, HorizontalAngle to) {
    double diff = normalizeDeg(to.numerical_value_in(deg)) -
                  normalizeDeg(from.numerical_value_in(deg));
    if (diff >  kHalfCircleDeg) diff -= kFullCircleDeg;
    if (diff < -kHalfCircleDeg) diff += kFullCircleDeg;
    return diff * horizontal_angle[deg];
}

// ─────────────────────────────────────────────────────────────────────────────
// MovementCommand builders — FIX 5: PhysicalLength, HorizontalAngle params
// ─────────────────────────────────────────────────────────────────────────────
[[nodiscard]] types::MovementCommand makeHover() {
    return types::MovementCommand{};
}

[[nodiscard]] types::MovementCommand makeRotate(HorizontalAngle angle) {
    types::MovementCommand cmd{};
    cmd.type     = types::MovementCommandType::Rotate;
    cmd.rotation = angle.numerical_value_in(deg) >= 0.0
                       ? types::RotationDirection::Left
                       : types::RotationDirection::Right;
    cmd.angle    = (angle.numerical_value_in(deg) >= 0.0) ? angle : -angle;
    return cmd;
}

[[nodiscard]] types::MovementCommand makeAdvance(PhysicalLength distance) {
    types::MovementCommand cmd{};
    cmd.type     = types::MovementCommandType::Advance;
    cmd.distance = distance;
    return cmd;
}

[[nodiscard]] types::MovementCommand makeElevate(PhysicalLength distance) {
    types::MovementCommand cmd{};
    cmd.type     = types::MovementCommandType::Elevate;
    cmd.distance = distance;
    return cmd;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Grid / world conversion helpers
// ─────────────────────────────────────────────────────────────────────────────
PhysicalLength MappingAlgorithmImpl::cellSize() const {
    const double r = mission_config_.gps_resolution.force_numerical_value_in(cm);
    return (r > 0.0 ? r : 1.0) * cm;
}

PhysicalLength MappingAlgorithmImpl::maxTraceDistance() const {
    const auto& b = mission_config_.boundaries;
    const bool unset = b.min_x.force_numerical_value_in(cm) == 0.0 &&
                       b.max_x.force_numerical_value_in(cm) == 0.0 &&
                       b.min_y.force_numerical_value_in(cm) == 0.0 &&
                       b.max_y.force_numerical_value_in(cm) == 0.0 &&
                       b.min_height.force_numerical_value_in(cm) == 0.0 &&
                       b.max_height.force_numerical_value_in(cm) == 0.0;
    if (!unset) {
        const double dx = b.max_x.force_numerical_value_in(cm) - b.min_x.force_numerical_value_in(cm);
        const double dy = b.max_y.force_numerical_value_in(cm) - b.min_y.force_numerical_value_in(cm);
        const double dz = b.max_height.force_numerical_value_in(cm) - b.min_height.force_numerical_value_in(cm);
        const double diag = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (std::isfinite(diag) && diag > 0.0)
            return std::min(diag, 10000.0) * cm;
    }
    return std::max(10.0 * cellSize().force_numerical_value_in(cm), 100.0) * cm;
}

MappingAlgorithmImpl::GridKey MappingAlgorithmImpl::toGrid(const Position3D& p) const {
    const double cell = cellSize().force_numerical_value_in(cm);
    return GridKey{
        static_cast<int>(std::llround(p.x.force_numerical_value_in(cm) / cell)),
        static_cast<int>(std::llround(p.y.force_numerical_value_in(cm) / cell)),
        static_cast<int>(std::llround(p.z.force_numerical_value_in(cm) / cell)),
    };
}

Position3D MappingAlgorithmImpl::toPosition(const GridKey& k) const {
    const double cell = cellSize().force_numerical_value_in(cm);
    return Position3D{
        static_cast<double>(k.x) * cell * x_extent[cm],
        static_cast<double>(k.y) * cell * y_extent[cm],
        static_cast<double>(k.z) * cell * z_extent[cm],
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// Known-voxel map helpers
// ─────────────────────────────────────────────────────────────────────────────
types::VoxelOccupancy MappingAlgorithmImpl::at(const GridKey& k) const {
    const auto it = known_voxels_.find(k);
    return it == known_voxels_.end() ? types::VoxelOccupancy::Unmapped : it->second;
}

void MappingAlgorithmImpl::setKnown(const GridKey& k, types::VoxelOccupancy v) {
    if (at(k) == types::VoxelOccupancy::Empty && v == types::VoxelOccupancy::Unmapped)
        return;
    known_voxels_[k] = v;
}

void MappingAlgorithmImpl::markCurrentVisited() {
    const GridKey here = toGrid(current_position_);
    visited_positions_.insert({here.x, here.y, here.z});
    setKnown(here, types::VoxelOccupancy::Empty);
}

// ─────────────────────────────────────────────────────────────────────────────
// Bounds helpers
// ─────────────────────────────────────────────────────────────────────────────
bool MappingAlgorithmImpl::isInsideMissionBounds(const GridKey& k) const {
    const auto& b = mission_config_.boundaries;
    const bool unset = b.min_x.force_numerical_value_in(cm) == 0.0 &&
                       b.max_x.force_numerical_value_in(cm) == 0.0 &&
                       b.min_y.force_numerical_value_in(cm) == 0.0 &&
                       b.max_y.force_numerical_value_in(cm) == 0.0 &&
                       b.min_height.force_numerical_value_in(cm) == 0.0 &&
                       b.max_height.force_numerical_value_in(cm) == 0.0;
    if (unset) return true;
    const Position3D pos = toPosition(k);
    return pos.x.force_numerical_value_in(cm) >= b.min_x.force_numerical_value_in(cm) &&
           pos.x.force_numerical_value_in(cm) <= b.max_x.force_numerical_value_in(cm) &&
           pos.y.force_numerical_value_in(cm) >= b.min_y.force_numerical_value_in(cm) &&
           pos.y.force_numerical_value_in(cm) <= b.max_y.force_numerical_value_in(cm) &&
           pos.z.force_numerical_value_in(cm) >= b.min_height.force_numerical_value_in(cm) &&
           pos.z.force_numerical_value_in(cm) <= b.max_height.force_numerical_value_in(cm);
}

bool MappingAlgorithmImpl::isNavigable(const GridKey& k) const {
    if (!isInsideMissionBounds(k)) return false;
    const auto v = at(k);
    return v == types::VoxelOccupancy::Empty || v == types::VoxelOccupancy::Unmapped;
}

// ─────────────────────────────────────────────────────────────────────────────
// FIX 4: ingestScan broken into two focused helpers
// ─────────────────────────────────────────────────────────────────────────────
void MappingAlgorithmImpl::markFreeRay(const Position3D& origin,
                                       double dx, double dy, double dz,
                                       PhysicalLength distance) {
    const PhysicalLength step = std::max(1.0, cellSize().force_numerical_value_in(cm) * 0.5) * cm;
    constexpr std::size_t kMaxSamples = 4096;
    std::size_t n = 0;
    for (PhysicalLength t = 0.0 * cm; t < distance && n < kMaxSamples; t += step, ++n) {
        const double t_cm = t.force_numerical_value_in(cm);
        const Position3D sample{
            origin.x + dx * t_cm * x_extent[cm],
            origin.y + dy * t_cm * y_extent[cm],
            origin.z + dz * t_cm * z_extent[cm],
        };
        setKnown(toGrid(sample), types::VoxelOccupancy::Empty);
    }
}

void MappingAlgorithmImpl::processHit(const types::DroneState& state,
                                      const types::LidarHit& hit) {
    const Orientation abs_or{
        HorizontalAngle{state.heading.horizontal + hit.angle.horizontal},
        AltitudeAngle{state.heading.altitude   + hit.angle.altitude},
    };
    const auto cos_alt = mp_units::si::cos(abs_or.altitude);
    const double dx = (cos_alt * mp_units::si::cos(abs_or.horizontal)).force_numerical_value_in(mp_units::one);
    const double dy = (cos_alt * mp_units::si::sin(abs_or.horizontal)).force_numerical_value_in(mp_units::one);
    const double dz = mp_units::si::sin(abs_or.altitude).force_numerical_value_in(mp_units::one);

    double dist_cm = hit.distance.force_numerical_value_in(cm);
    if (!std::isfinite(dist_cm) || dist_cm < 0.0) return;

    const PhysicalLength cap = maxTraceDistance();
    const bool has_real_hit  = dist_cm <= cap.force_numerical_value_in(cm);
    if (!has_real_hit) dist_cm = cap.force_numerical_value_in(cm);

    markFreeRay(state.position, dx, dy, dz, dist_cm * cm);

    if (has_real_hit) {
        const Position3D hit_pos{
            state.position.x + dx * dist_cm * x_extent[cm],
            state.position.y + dy * dist_cm * y_extent[cm],
            state.position.z + dz * dist_cm * z_extent[cm],
        };
        setKnown(toGrid(hit_pos), types::VoxelOccupancy::Occupied);
    }
}

void MappingAlgorithmImpl::ingestScan(const types::DroneState& state,
                                      const types::LidarScanResult& scan) {
    setKnown(toGrid(state.position), types::VoxelOccupancy::Empty);
    for (const auto& hit : scan) {
        processHit(state, hit);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Frontier / BFS helpers — FIX 4: each check is its own method
// ─────────────────────────────────────────────────────────────────────────────
bool MappingAlgorithmImpl::hasUnknownOrthogonalNeighbor(const GridKey& k) const {
    for (const auto& d : kBfsNeighbors) {
        const GridKey n{k.x+d.dx, k.y+d.dy, k.z+d.dz};
        if (isInsideMissionBounds(n) && at(n) == types::VoxelOccupancy::Unmapped)
            return true;
    }
    return false;
}

bool MappingAlgorithmImpl::hasUnknownMooreNeighbor(const GridKey& k) const {
    constexpr int R = 2;
    for (int dz = -R; dz <= R; ++dz)
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                const GridKey n{k.x+dx, k.y+dy, k.z+dz};
                if (isInsideMissionBounds(n) && at(n) == types::VoxelOccupancy::Unmapped)
                    return true;
            }
    return false;
}

bool MappingAlgorithmImpl::isBfsGoal(const GridKey& k, BfsGoalMode mode) const {
    if (at(k) != types::VoxelOccupancy::Empty) return false;
    if (visited_positions_.contains({k.x, k.y, k.z})) return false;
    return mode == BfsGoalMode::Frontier6
               ? hasUnknownOrthogonalNeighbor(k)
               : hasUnknownMooreNeighbor(k);
}

// FIX 4: BFS path reconstruction as a private member (GridKey is private)
std::vector<MappingAlgorithmImpl::GridKey>
MappingAlgorithmImpl::reconstructPath(const GridKey& goal,
                                       const GridKey& start,
                                       const std::map<GridKey, GridKey>& parent) const {
    std::vector<MappingAlgorithmImpl::GridKey> path;
    MappingAlgorithmImpl::GridKey cur = goal;
    while (!(cur == start)) {
        path.push_back(cur);
        cur = parent.at(cur);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<MappingAlgorithmImpl::GridKey>
MappingAlgorithmImpl::bfsToGoal(BfsGoalMode mode) const {
    const GridKey start = toGrid(current_position_);
    std::queue<GridKey> q;
    std::set<std::tuple<int,int,int>> visited;
    std::map<GridKey, GridKey> parent;

    q.push(start);
    visited.insert({start.x, start.y, start.z});

    while (!q.empty()) {
        const GridKey cur = q.front(); q.pop();

        if (!(cur == start) && isBfsGoal(cur, mode))
            return reconstructPath(cur, start, parent);

        for (const auto& d : kBfsNeighbors) {
            const GridKey next{cur.x+d.dx, cur.y+d.dy, cur.z+d.dz};
            const auto kt = std::make_tuple(next.x, next.y, next.z);
            if (visited.contains(kt) || !isInsideMissionBounds(next) ||
                at(next) != types::VoxelOccupancy::Empty)
                continue;
            visited.insert(kt);
            parent[next] = cur;
            q.push(next);
            if (visited.size() > 20000U) return {};
        }
    }
    return {};
}

// ─────────────────────────────────────────────────────────────────────────────
// Movement planning — FIX 5: PhysicalLength / HorizontalAngle strong types
// ─────────────────────────────────────────────────────────────────────────────
void MappingAlgorithmImpl::enqueueCommandsForStep(const GridKey& target) {
    const GridKey here = toGrid(current_position_);
    const PhysicalLength cell = cellSize();



    // Elevate
    if (target.z != here.z) {
        const PhysicalLength max_elev = drone_config_.max_elevate;
        const PhysicalLength eff_elev = max_elev.force_numerical_value_in(cm) > kEpsilon
                                            ? max_elev : cell;
        const PhysicalLength dz_dist =
            static_cast<double>(target.z - here.z) * cell.force_numerical_value_in(cm) * cm;
        PhysicalLength remaining = dz_dist;
        while (std::abs(remaining.force_numerical_value_in(cm)) > kEpsilon) {
            const double mag = std::min(std::abs(remaining.force_numerical_value_in(cm)),
                                        eff_elev.force_numerical_value_in(cm));
            const PhysicalLength chunk =
                (remaining.force_numerical_value_in(cm) >= 0.0 ? mag : -mag) * cm;
            pending_commands_.push_back(makeElevate(chunk));
            remaining = remaining - chunk;
        }
    }

    // Rotate + Advance
    if (target.x != here.x || target.y != here.y) {
        double desired_deg = 0.0;
        if      (target.x > here.x) desired_deg = 0.0;
        else if (target.x < here.x) desired_deg = 180.0;
        else if (target.y > here.y) desired_deg = 90.0;
        else                         desired_deg = 270.0;

        const HorizontalAngle turn = smallestTurn(
            orientation_.horizontal, desired_deg * horizontal_angle[deg]);
        const HorizontalAngle max_rot = drone_config_.max_rotate;
        const HorizontalAngle eff_rot = max_rot.force_numerical_value_in(deg) > kEpsilon
                                            ? max_rot
                                            : 90.0 * horizontal_angle[deg];
        HorizontalAngle rem_turn = turn;
        while (std::abs(rem_turn.force_numerical_value_in(deg)) > kEpsilon) {
            const double mag = std::min(std::abs(rem_turn.force_numerical_value_in(deg)),
                                        eff_rot.force_numerical_value_in(deg));
            const HorizontalAngle chunk =
                (rem_turn.force_numerical_value_in(deg) >= 0.0 ? mag : -mag) *
                horizontal_angle[deg];
            pending_commands_.push_back(makeRotate(chunk));
            rem_turn = rem_turn - chunk;
        }

        // Advance
        const double dx = static_cast<double>(target.x - here.x) * cell.force_numerical_value_in(cm);
        const double dy = static_cast<double>(target.y - here.y) * cell.force_numerical_value_in(cm);
        const PhysicalLength dist = std::sqrt(dx*dx + dy*dy) * cm;
        const PhysicalLength max_adv = drone_config_.max_advance;
        const PhysicalLength eff_adv = max_adv.force_numerical_value_in(cm) > kEpsilon
                                           ? max_adv : cell;
        PhysicalLength rem_adv = dist;
        while (rem_adv.force_numerical_value_in(cm) > kEpsilon) {
            const PhysicalLength chunk =
                std::min(rem_adv.force_numerical_value_in(cm),
                         eff_adv.force_numerical_value_in(cm)) * cm;
            pending_commands_.push_back(makeAdvance(chunk));
            rem_adv = rem_adv - chunk;
        }
    }
}

bool MappingAlgorithmImpl::enqueueSweepMove() {
    const GridKey here = toGrid(current_position_);
    for (const auto& d : kSweepDirections) {
        const GridKey next{here.x+d.dx, here.y+d.dy, here.z+d.dz};
        if (visited_positions_.contains({next.x, next.y, next.z})) continue;
        if (!isNavigable(next)) continue;
        enqueueCommandsForStep(next);
        if (!pending_commands_.empty()) return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// State-machine steps
// ─────────────────────────────────────────────────────────────────────────────
types::MappingStepCommand MappingAlgorithmImpl::nextMovingStep() {
    if (pending_commands_.empty()) {
        state_ = ExplorationState::Planning;
        return nextPlanningStep();
    }
    auto cmd = pending_commands_.front();
    pending_commands_.pop_front();
    if (pending_commands_.empty())
        state_ = ExplorationState::Planning;

    types::MappingStepCommand step{};
    step.movement        = cmd;
    step.scan_orientation = Orientation{0.0 * horizontal_angle[deg],
                                        0.0 * altitude_angle[deg]};
    step.status = types::AlgorithmStatus::Working;
    return step;
}

types::MappingStepCommand MappingAlgorithmImpl::finishedCommand() {
    types::MappingStepCommand done{};
    done.movement = makeHover();
    done.status   = types::AlgorithmStatus::Finished;
    return done;
}

types::MappingStepCommand MappingAlgorithmImpl::nextPlanningStep() {
    markCurrentVisited();

    while (!current_path_.empty()) {
        const GridKey next = current_path_.front();
        current_path_.erase(current_path_.begin());
        if (!isNavigable(next)) { current_path_.clear(); break; }
        enqueueCommandsForStep(next);
        if (!pending_commands_.empty()) {
            state_ = ExplorationState::Moving;
            return nextMovingStep();
        }
    }

    if (enqueueSweepMove()) {
        state_ = ExplorationState::Moving;
        return nextMovingStep();
    }

    auto path = bfsToGoal(BfsGoalMode::Frontier6);
    if (path.empty()) path = bfsToGoal(BfsGoalMode::Frontier26);
    if (!path.empty()) {
        current_path_ = std::move(path);
        return nextPlanningStep();
    }

    state_ = ExplorationState::Finished;
    return finishedCommand();
}

// ─────────────────────────────────────────────────────────────────────────────
// Public entry point
// ─────────────────────────────────────────────────────────────────────────────
types::MappingStepCommand MappingAlgorithmImpl::nextStep(
    const types::DroneState& state,
    const types::LidarScanResult* latest_scan) {

    current_position_ = state.position;
    orientation_      = state.heading;

    if (latest_scan != nullptr) ingestScan(state, *latest_scan);

    if (state_ == ExplorationState::Finished) return finishedCommand();
    if (state_ == ExplorationState::Moving)   return nextMovingStep();
    return nextPlanningStep();
}

} // namespace drone_mapper
