#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>

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

// ── Constants (HW1 names kept where possible) ────────────────────────────────
constexpr double kHalfCircleDeg  = 180.0;
constexpr double kFullCircleDeg  = 360.0;
constexpr double kEpsilon        = 1e-6;
constexpr double kDdaEpsilon     = 1e-9;

// Fixed scan batch: 8 horizontal directions at 3 elevations.
constexpr double kScanForwardDeg   = 0.0;
constexpr double kScanRightDeg     = 90.0;
constexpr double kScanBackDeg      = 180.0;
constexpr double kScanLeftDeg      = -90.0;
constexpr double kElevationUpDeg   = 30.0;
constexpr double kElevationDownDeg = -30.0;

struct GridOffset { int dx; int dy; int dz; };

// HW1 BFS_NEIGHBOR_DIRECTIONS
constexpr std::array<GridOffset, 6> kBfsNeighbors{{
    {-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1},
}};
// HW1 SWEEP_DIRECTIONS
constexpr std::array<GridOffset, 6> kSweepDirections{{
    {1,0,0},{0,1,0},{0,-1,0},{-1,0,0},{0,0,1},{0,0,-1},
}};

// ── Angle helpers (HW1 normalize_angle / smallest_turn) ──────────────────────
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

// ── Movement command helpers ─────────────────────────────────────────────────
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

// ── World / grid conversion ──────────────────────────────────────────────────
// HW1 used a 1 cm internal grid; HW2's grid cell is the GPS resolution.
PhysicalLength MappingAlgorithmImpl::cellSize() const {
    const auto cfg = output_map_.getMapConfig();
    const double map_res = cfg.resolution.force_numerical_value_in(cm);
    if (map_res > 0.0) {
        return map_res * cm;
    }
    const double gps_res = mission_config_.gps_resolution.force_numerical_value_in(cm);
    return (gps_res > 0.0 ? gps_res : 1.0) * cm;
}

// HW1 used lidar z_max as the ray-marking horizon (config_.lidar_z_max_cm).
PhysicalLength MappingAlgorithmImpl::lidarMaxDistance() const {
    const double z_max = lidar_config_.z_max.force_numerical_value_in(cm);
    return (z_max > 0.0 ? z_max : 100.0) * cm;
}

// Round to the nearest map voxel using the map config offset/resolution.
MappingAlgorithmImpl::GridKey MappingAlgorithmImpl::toGrid(const Position3D& p) const {
    const double cell = cellSize().force_numerical_value_in(cm);
    const auto cfg = output_map_.getMapConfig();
    const double ox = cfg.offset.x.force_numerical_value_in(cm);
    const double oy = cfg.offset.y.force_numerical_value_in(cm);
    const double oz = cfg.offset.z.force_numerical_value_in(cm);
    return GridKey{
        static_cast<int>(std::llround((p.x.force_numerical_value_in(cm) - ox) / cell)),
        static_cast<int>(std::llround((p.y.force_numerical_value_in(cm) - oy) / cell)),
        static_cast<int>(std::llround((p.z.force_numerical_value_in(cm) - oz) / cell)),
    };
}

// Voxel corner in world coordinates, aligned with the map config offset.
Position3D MappingAlgorithmImpl::toPosition(const GridKey& k) const {
    const double cell = cellSize().force_numerical_value_in(cm);
    const auto cfg = output_map_.getMapConfig();
    return Position3D{
        (cfg.offset.x.force_numerical_value_in(cm) + static_cast<double>(k.x) * cell) * x_extent[cm],
        (cfg.offset.y.force_numerical_value_in(cm) + static_cast<double>(k.y) * cell) * y_extent[cm],
        (cfg.offset.z.force_numerical_value_in(cm) + static_cast<double>(k.z) * cell) * z_extent[cm],
    };
}

// ── Known-voxel map helpers ──────────────────────────────────────────────────
types::VoxelOccupancy MappingAlgorithmImpl::at(const GridKey& k) const {
    const auto it = known_voxels_.find(k);
    return it == known_voxels_.end() ? types::VoxelOccupancy::Unmapped : it->second;
}

void MappingAlgorithmImpl::setKnown(const GridKey& k, types::VoxelOccupancy v) {
    known_voxels_[k] = v;
}

// HW1 markCurrentPositionVisited + preserveVisitedPositionsAsFree.
void MappingAlgorithmImpl::markCurrentVisited() {
    const GridKey here = toGrid(current_position_);
    visited_positions_.insert({here.x, here.y, here.z});
    preserveVisitedPositionsAsFree();
}

void MappingAlgorithmImpl::preserveVisitedPositionsAsFree() {
    for (const auto& position : visited_positions_) {
        const auto [x, y, z] = position;
        known_voxels_[GridKey{x, y, z}] = types::VoxelOccupancy::Empty;
    }
    known_voxels_[toGrid(current_position_)] = types::VoxelOccupancy::Empty;
}

// ── Mission bounds ───────────────────────────────────────────────────────────
// HW1 pre-filled OUT_OF_BOUNDS voxels in its map; here we test on demand.
bool MappingAlgorithmImpl::isInsideMissionBounds(const GridKey& k) const {
    const auto& b = mission_config_.mission_bounds;
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

// ── Safety (HW1 sphereAreaIsFree / isNavigableVoxel) ─────────────────────────
// HW1 iterated 1 cm cells inside the sphere and required all of them FREE.
// Here we iterate 1 cm sample points inside the sphere around the voxel
// CENTER and require the grid cell of every sample to be Empty — the same
// conservative rule: Unmapped or Occupied anywhere inside the footprint means
// the target is not safe.
bool MappingAlgorithmImpl::sphereAreaIsFree(const GridKey& k) const {
    const double r_cm = drone_config_.radius.force_numerical_value_in(cm);
    if (r_cm <= 0.0) {
        return at(k) == types::VoxelOccupancy::Empty;
    }

    const Position3D center = toPosition(k);
    const int    r    = static_cast<int>(std::ceil(r_cm));
    const double r2   = r_cm * r_cm;
    const double cell = cellSize().force_numerical_value_in(cm);
    const double cx   = center.x.force_numerical_value_in(cm);
    const double cy   = center.y.force_numerical_value_in(cm);
    const double cz   = center.z.force_numerical_value_in(cm);

    // Keep the same conservative 1 cm sphere sampling rule, but avoid the very
    // expensive path that created a Position3D and ran mp-units conversions for
    // every sample point. Many 1 cm samples fall in the same coarse grid voxel
    // (gps_resolution is often 10 cm), so check each sampled voxel only once.
    std::set<GridKey> sampled_cells;
    for (int dz = -r; dz <= r; ++dz) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (static_cast<double>(dx*dx + dy*dy + dz*dz) >= r2) continue;
                GridKey sk{
                    static_cast<int>(std::floor((cx + static_cast<double>(dx)) / cell)),
                    static_cast<int>(std::floor((cy + static_cast<double>(dy)) / cell)),
                    static_cast<int>(std::floor((cz + static_cast<double>(dz)) / cell)),
                };
                if (!sampled_cells.insert(sk).second) continue;
                if (!isInsideMissionBounds(sk)) return false;
                if (at(sk) != types::VoxelOccupancy::Empty) return false;
            }
        }
    }
    return true;
}

bool MappingAlgorithmImpl::isNavigable(const GridKey& k) const {
    if (!isInsideMissionBounds(k)) return false;
    return sphereAreaIsFree(k);
}

// ── Scan ingestion (HW1 markScanRay: DDA voxel traversal) ────────────────────
void MappingAlgorithmImpl::markScanRay(const Position3D& origin,
                                       double dx, double dy, double dz,
                                       double reported_distance_cm) {
    const double len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len == 0.0) return;
    dx /= len; dy /= len; dz /= len;

    const double cell    = cellSize().force_numerical_value_in(cm);
    const double max_cm  = lidarMaxDistance().force_numerical_value_in(cm);
    const double pos_x   = origin.x.force_numerical_value_in(cm);
    const double pos_y   = origin.y.force_numerical_value_in(cm);
    const double pos_z   = origin.z.force_numerical_value_in(cm);

    // HW1 special case: distance 0 means the obstacle is closer than z_min.
    // Mark the first cell outside the current one, along the beam, Occupied.
    if (reported_distance_cm <= 0.0) {
        const GridKey current = toGrid(origin);
        double test_dist = 0.0;
        GridKey hit = current;
        while (hit == current && test_dist <= max_cm) {
            test_dist += cell;
            hit = toGrid(Position3D{
                (pos_x + dx * test_dist) * x_extent[cm],
                (pos_y + dy * test_dist) * y_extent[cm],
                (pos_z + dz * test_dist) * z_extent[cm],
            });
        }
        if (!(hit == current) && isInsideMissionBounds(hit)) {
            setKnown(hit, types::VoxelOccupancy::Occupied);
        }
        return;
    }

    // HW1: a reading at/over max range is open air, not a wall at the end.
    bool is_valid_hit = true;
    if (!std::isfinite(reported_distance_cm) || reported_distance_cm >= max_cm) {
        reported_distance_cm = max_cm;
        is_valid_hit = false;
    }
    const double effective_dist = reported_distance_cm;

    // DDA setup on the coarse grid (HW1 used 1 cm cells; here cell = gps res).
    int x = static_cast<int>(std::floor(pos_x / cell));
    int y = static_cast<int>(std::floor(pos_y / cell));
    int z = static_cast<int>(std::floor(pos_z / cell));

    const int step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
    const int step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
    const int step_z = dz > 0.0 ? 1 : (dz < 0.0 ? -1 : 0);

    constexpr double kInf = std::numeric_limits<double>::infinity();
    auto boundary_t = [cell](double pos, double dir, int step, int idx) {
        if (step == 0) return kInf;
        const double edge = (step > 0 ? (idx + 1) * cell : idx * cell);
        return (edge - pos) / dir;
    };
    double t_max_x = boundary_t(pos_x, dx, step_x, x);
    double t_max_y = boundary_t(pos_y, dy, step_y, y);
    double t_max_z = boundary_t(pos_z, dz, step_z, z);

    const double t_delta_x = (step_x == 0) ? kInf : std::abs(cell / dx);
    const double t_delta_y = (step_y == 0) ? kInf : std::abs(cell / dy);
    const double t_delta_z = (step_z == 0) ? kInf : std::abs(cell / dz);

    while (true) {
        const GridKey cur{x, y, z};
        if (!isInsideMissionBounds(cur)) return;                    // HW1: OUT_OF_BOUNDS stops the ray

        double next_t;
        enum class StepAxis { X, Y, Z };
        StepAxis axis;
        if (t_max_x < t_max_y && t_max_x < t_max_z) { next_t = t_max_x; axis = StepAxis::X; }
        else if (t_max_y < t_max_z)                 { next_t = t_max_y; axis = StepAxis::Y; }
        else                                        { next_t = t_max_z; axis = StepAxis::Z; }

        // No further cell boundary before the reported distance (or none at
        // all): the reported distance lands inside `cur` - it is the
        // terminal cell, not a pass-through. Mark it Occupied (valid hit) or
        // Empty (miss/max-range) and stop. This must be checked before ever
        // marking `cur` Empty, or a hit landing inside a cell gets recorded
        // as empty space and the actual obstacle is silently lost.
        if (!std::isfinite(next_t) || next_t > effective_dist + kDdaEpsilon) {
            if (is_valid_hit) {
                setKnown(cur, types::VoxelOccupancy::Occupied);
            } else if (at(cur) != types::VoxelOccupancy::Occupied) {
                setKnown(cur, types::VoxelOccupancy::Empty);
            }
            return;
        }

        if (!is_valid_hit && at(cur) == types::VoxelOccupancy::Occupied) return; // HW1 miss rule
        setKnown(cur, types::VoxelOccupancy::Empty);

        if      (axis == StepAxis::X) { x += step_x; t_max_x += t_delta_x; }
        else if (axis == StepAxis::Y) { y += step_y; t_max_y += t_delta_y; }
        else                          { z += step_z; t_max_z += t_delta_z; }
    }
}

// HW1 processScanResultsForExecutedScan. MockLidar reports every beam
// (misses use max double), so there is no need for HW1's expected-beam
// reconstruction; the hit angle is relative to the drone heading.
void MappingAlgorithmImpl::ingestScan(const types::DroneState& state,
                                      const types::LidarScanResult& scan) {
    for (const auto& hit : scan) {
        const Orientation abs_or{
            HorizontalAngle{state.heading.horizontal + hit.angle.horizontal},
            AltitudeAngle{state.heading.altitude   + hit.angle.altitude},
        };
        const auto cos_alt = mp_units::si::cos(abs_or.altitude);
        const double dx = (cos_alt * mp_units::si::cos(abs_or.horizontal)).force_numerical_value_in(mp_units::one);
        const double dy = (cos_alt * mp_units::si::sin(abs_or.horizontal)).force_numerical_value_in(mp_units::one);
        const double dz = mp_units::si::sin(abs_or.altitude).force_numerical_value_in(mp_units::one);

        const double dist_cm = hit.distance.force_numerical_value_in(cm);
        if (std::isfinite(dist_cm) && dist_cm < 0.0) continue;
        markScanRay(state.position, dx, dy, dz, dist_cm);
    }
    preserveVisitedPositionsAsFree(); // HW1 does this after every processed scan
}

// ── Frontier helpers (HW1 verbatim logic) ────────────────────────────────────
bool MappingAlgorithmImpl::hasUnknownOrthogonalNeighbor(const GridKey& k) const {
    for (const auto& d : kBfsNeighbors) {
        const GridKey n{k.x+d.dx, k.y+d.dy, k.z+d.dz};
        if (isInsideMissionBounds(n) && at(n) == types::VoxelOccupancy::Unmapped)
            return true;
    }
    return false;
}

// HW1 hasLineOfSightToUnknown: a diagonal unknown does not count as a frontier
// if a known wall blocks the straight line to it.
bool MappingAlgorithmImpl::hasLineOfSightToUnknown(const GridKey& from,
                                                   int dx, int dy, int dz) const {
    const int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
    if (steps <= 1) return true;
    for (int step = 1; step < steps; ++step) {
        const GridKey mid{
            from.x + static_cast<int>(std::round(static_cast<double>(dx) * step / steps)),
            from.y + static_cast<int>(std::round(static_cast<double>(dy) * step / steps)),
            from.z + static_cast<int>(std::round(static_cast<double>(dz) * step / steps)),
        };
        if (!isInsideMissionBounds(mid)) return false;
        if (at(mid) == types::VoxelOccupancy::Occupied) return false;
    }
    return true;
}

bool MappingAlgorithmImpl::hasUnknownMooreNeighbor(const GridKey& k) const {
    constexpr int R = 2; // HW1 search_radius
    for (int dz = -R; dz <= R; ++dz)
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                const GridKey n{k.x+dx, k.y+dy, k.z+dz};
                if (isInsideMissionBounds(n) &&
                    at(n) == types::VoxelOccupancy::Unmapped &&
                    hasLineOfSightToUnknown(k, dx, dy, dz))
                    return true;
            }
    return false;
}

// HW1 isBfsGoal: FREE, not visited, and a frontier of the requested kind.
bool MappingAlgorithmImpl::isBfsGoal(const GridKey& k, BfsGoalMode mode) const {
    if (at(k) != types::VoxelOccupancy::Empty) return false;
    if (visited_positions_.contains({k.x, k.y, k.z})) return false;
    return mode == BfsGoalMode::Frontier6
               ? hasUnknownOrthogonalNeighbor(k)
               : hasUnknownMooreNeighbor(k);
}

// ── Targeted scans (HW1 makeTargetedScanCommand) ─────────────────────────────
std::optional<Orientation> MappingAlgorithmImpl::targetedScanOrientation(const GridKey& target) const {
    if (!isInsideMissionBounds(target) ||
        at(target) != types::VoxelOccupancy::Unmapped) {
        return std::nullopt;
    }
    const Position3D tp = toPosition(target);
    const double vx = tp.x.force_numerical_value_in(cm) - current_position_.x.force_numerical_value_in(cm);
    const double vy = tp.y.force_numerical_value_in(cm) - current_position_.y.force_numerical_value_in(cm);
    const double vz = tp.z.force_numerical_value_in(cm) - current_position_.z.force_numerical_value_in(cm);
    const double horizontal = std::hypot(vx, vy);

    if (horizontal < kEpsilon && std::abs(vz) < kEpsilon) return std::nullopt;

    const double abs_azimuth = std::atan2(vy, vx) * kHalfCircleDeg / M_PI;
    const HorizontalAngle relative =
        smallestTurn(orientation_.horizontal, abs_azimuth * horizontal_angle[deg]);
    const double elevation = std::atan2(vz, horizontal) * kHalfCircleDeg / M_PI;
    // The simulator applies scan angles relative to the current heading (HW1 note).
    return Orientation{relative, elevation * altitude_angle[deg]};
}

// HW1 enqueueTargetedScanForTarget: the target is blocked because its safety
// sphere contains UNKNOWN cells — scan one of them directly.
bool MappingAlgorithmImpl::enqueueTargetedScanForTarget(const GridKey& target) {
    const double r_cm = drone_config_.radius.force_numerical_value_in(cm);
    const double cell = cellSize().force_numerical_value_in(cm);
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(r_cm / cell)));
    const double radius_sq = static_cast<double>(radius_cells) * radius_cells;

    for (int dz = -radius_cells; dz <= radius_cells; ++dz)
        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if (static_cast<double>(dx*dx + dy*dy + dz*dz) > radius_sq) continue;
                if (auto o = targetedScanOrientation(GridKey{target.x+dx, target.y+dy, target.z+dz})) {
                    pending_scans_.push_back(*o);
                    return true;
                }
            }
    return false;
}

// HW1 enqueueTargetedScanAroundCurrentPosition, widened: HW1 only looked at
// the 6 orthogonal neighbors, but with a narrow-FOV lidar those are already
// covered by the batch's central beams while diagonal cells (which block
// sphereAreaIsFree) stay unknown. Search the nearest Unmapped cell within
// lidar range instead, and never re-target the same cell from the same
// position so the scan loop always terminates.
bool MappingAlgorithmImpl::enqueueTargetedScanAroundCurrentPosition() {
    const GridKey here = toGrid(current_position_);
    const double cell  = cellSize().force_numerical_value_in(cm);
    const int R = std::max(1, static_cast<int>(std::ceil(
        lidarMaxDistance().force_numerical_value_in(cm) / cell)));

    // Search outward in expanding Chebyshev shells instead of building the
    // full R-radius cube up front: R is derived from the lidar's max range,
    // which for long-range configs can be hundreds of cells, making an
    // upfront O(R^3) scan of every cell in the cube prohibitively expensive
    // (measured: minutes per call). A shell's closest possible point is
    // exactly shell*shell away, so once no remaining shell can beat the best
    // candidate found so far, further shells are skipped - same globally
    // nearest not-yet-targeted Unmapped cell as scanning the whole cube,
    // just without ever visiting cells farther away than necessary. Each
    // shell visits only its own surface (six trimmed faces, O(shell^2)), not
    // the sub-cube filtered down to it - visiting the sub-cube would make
    // the "nothing left to find" case (every shell run to completion) an
    // O(R^4) scan instead of the O(R^3) worst case this replaces.
    std::optional<GridKey> best;
    int best_dist_sq = std::numeric_limits<int>::max();

    auto tryCell = [&](int dx, int dy, int dz) {
        const int dist_sq = dx*dx + dy*dy + dz*dz;
        if (dist_sq == 0 || dist_sq > R*R || dist_sq >= best_dist_sq) return;
        const GridKey n{here.x+dx, here.y+dy, here.z+dz};
        if (!isInsideMissionBounds(n)) return;
        if (at(n) != types::VoxelOccupancy::Unmapped) return;
        if (targeted_scans_done_.contains({here.x, here.y, here.z, n.x, n.y, n.z})) return;
        best = n;
        best_dist_sq = dist_sq;
    };

    for (int shell = 1; shell <= R; ++shell) {
        if (best.has_value() && shell * shell > best_dist_sq) break;

        // Faces where x = +-shell (full y,z square).
        for (int dy = -shell; dy <= shell; ++dy)
            for (int dz = -shell; dz <= shell; ++dz) {
                tryCell(shell, dy, dz);
                tryCell(-shell, dy, dz);
            }
        // Faces where y = +-shell (x trimmed to skip the x=+-shell edges above).
        for (int dx = -shell + 1; dx <= shell - 1; ++dx)
            for (int dz = -shell; dz <= shell; ++dz) {
                tryCell(dx, shell, dz);
                tryCell(dx, -shell, dz);
            }
        // Faces where z = +-shell (x,y trimmed to skip edges already visited above).
        for (int dx = -shell + 1; dx <= shell - 1; ++dx)
            for (int dy = -shell + 1; dy <= shell - 1; ++dy) {
                tryCell(dx, dy, shell);
                tryCell(dx, dy, -shell);
            }
    }

    if (!best.has_value()) return false;
    if (auto o = targetedScanOrientation(*best)) {
        targeted_scans_done_.insert({here.x, here.y, here.z,
                                     best->x, best->y, best->z});
        pending_scans_.push_back(*o);
        return true;
    }
    return false;
}

// ── BFS (HW1 bfs_to_goal over sphere-safe cells) ─────────────────────────────
std::vector<MappingAlgorithmImpl::GridKey>
MappingAlgorithmImpl::reconstructPath(const GridKey& goal,
                                       const GridKey& start,
                                       const std::map<GridKey, GridKey>& parent) const {
    std::vector<GridKey> path;
    GridKey cur = goal;
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
    // HW1 required the start cell to be FREE before searching.
    if (at(start) != types::VoxelOccupancy::Empty) return {};

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
            if (visited.contains(kt) || !isNavigable(next)) continue;
            visited.insert(kt);
            parent[next] = cur;
            q.push(next);
        }
    }
    return {};
}

// ── Movement planning (HW1 buildCommandsForStep) ─────────────────────────────
// Order preserved from HW1: Elevate first, then Rotate (smallest turn,
// chunked), then Advance (chunked).
void MappingAlgorithmImpl::enqueueCommandsForStep(const GridKey& target) {
    const GridKey here = toGrid(current_position_);
    const double cell  = cellSize().force_numerical_value_in(cm);

    if (target.z != here.z) {
        const double max_elev = drone_config_.max_elevate.force_numerical_value_in(cm);
        const double eff_elev = max_elev > kEpsilon ? max_elev : cell;
        double remaining = static_cast<double>(target.z - here.z) * cell;
        while (std::abs(remaining) > kEpsilon) {
            double step = std::min(std::abs(remaining), eff_elev);
            if (remaining < 0.0) step = -step;
            pending_commands_.push_back(makeElevate(step * cm));
            remaining -= step;
        }
    }

    if (target.x != here.x || target.y != here.y) {
        // HW1 FORWARD/RIGHT/BACK/LEFT mapping: 0=+x, 90=+y, 180=-x, 270=-y.
        double desired_deg = 0.0;
        if      (target.x > here.x) desired_deg = 0.0;
        else if (target.x < here.x) desired_deg = 180.0;
        else if (target.y > here.y) desired_deg = 90.0;
        else                        desired_deg = 270.0;

        const double turn = smallestTurn(orientation_.horizontal,
                                         desired_deg * horizontal_angle[deg])
                                .numerical_value_in(deg);
        const double max_rot = drone_config_.max_rotate.force_numerical_value_in(deg);
        const double eff_rot = max_rot > kEpsilon ? max_rot : 90.0;
        double rem_turn = turn;
        while (std::abs(rem_turn) > kEpsilon) {
            double step = std::min(std::abs(rem_turn), eff_rot);
            if (rem_turn < 0.0) step = -step;
            pending_commands_.push_back(makeRotate(step * horizontal_angle[deg]));
            rem_turn -= step;
        }

        const double ddx = static_cast<double>(target.x - here.x) * cell;
        const double ddy = static_cast<double>(target.y - here.y) * cell;
        const double dist = std::sqrt(ddx*ddx + ddy*ddy);
        const double max_adv = drone_config_.max_advance.force_numerical_value_in(cm);
        const double eff_adv = max_adv > kEpsilon ? max_adv : cell;
        double rem_adv = dist;
        while (rem_adv > kEpsilon) {
            const double step = std::min(rem_adv, eff_adv);
            pending_commands_.push_back(makeAdvance(step * cm));
            rem_adv -= step;
        }
    }
}

// HW1 enqueueSweepMove: cheap adjacent step before running BFS.
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

// Broader sweep inspired by the student solution: 8 horizontal directions at
// 3 elevations. This gives the planner more complete local coverage before it
// commits to the next move.
void MappingAlgorithmImpl::enqueueScanBatch() {
    constexpr std::array<double, 8> kAzimuths{
        0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0};
    constexpr std::array<double, 3> kElevations{
        kElevationDownDeg, 0.0, kElevationUpDeg};
    for (const double elevation : kElevations) {
        for (const double azimuth : kAzimuths) {
            pending_scans_.push_back(Orientation{azimuth * horizontal_angle[deg],
                                                 elevation * altitude_angle[deg]});
        }
    }
    pending_scans_.push_back(Orientation{kScanForwardDeg * horizontal_angle[deg], kElevationUpDeg   * altitude_angle[deg]});
    pending_scans_.push_back(Orientation{kScanForwardDeg * horizontal_angle[deg], kElevationDownDeg * altitude_angle[deg]});
}

// ── State machine (HW1 nextScanningCommand / nextPlanningCommand / nextMovingCommand)
types::MappingStepCommand MappingAlgorithmImpl::nextScanningStep() {
    // Batch done and this position already visited -> plan (HW1).
    if (pending_scans_.empty()) {
        const GridKey here = toGrid(current_position_);
        if (visited_positions_.contains({here.x, here.y, here.z})) {
            state_ = ExplorationState::Planning;
            return nextPlanningStep();
        }
        enqueueScanBatch();
    }

    types::MappingStepCommand step{};
    step.scan_orientation = pending_scans_.front();
    pending_scans_.pop_front();
    step.status = types::AlgorithmStatus::Working;

    if (pending_scans_.empty()) {
        // HW1 finishCurrentScanBatch: mark visited; the next call plans.
        markCurrentVisited();
        state_ = ExplorationState::Planning;
    }
    return step;
}

types::MappingStepCommand MappingAlgorithmImpl::nextMovingStep() {
    if (pending_commands_.empty()) {
        state_ = ExplorationState::Scanning; // HW1: always re-scan after arriving
        return nextScanningStep();
    }
    types::MappingStepCommand step{};
    step.movement = pending_commands_.front();
    pending_commands_.pop_front();
    step.status = types::AlgorithmStatus::Working;
    if (pending_commands_.empty()) {
        // Arrival at the leg's target cell. The API allows a single step to
        // carry both a movement and a scan (movement runs first) — pack this
        // final movement chunk together with the first scan of the arrival
        // batch instead of spending a separate bare step on it. Same fixed
        // batch, same order, no change to what is decided or when — one
        // fewer step consumed per waypoint.
        state_ = ExplorationState::Scanning;
        if (pending_scans_.empty()) enqueueScanBatch();
        step.scan_orientation = pending_scans_.front();
        pending_scans_.pop_front();
    }
    return step;
}

types::MappingStepCommand MappingAlgorithmImpl::finishedCommand() {
    types::MappingStepCommand done{};
    done.status = types::AlgorithmStatus::Finished;
    return done;
}

types::MappingStepCommand MappingAlgorithmImpl::nextPlanningStep() {
    // 1. Continue an existing BFS path if the next step is still safe (HW1).
    //    Keep the path granular on coarse grids: one voxel at a time is
    //    safer than collapsing several cells into a longer leg.
    while (!current_path_.empty()) {
        const GridKey next = current_path_.front();
        if (!isNavigable(next)) {
            if (enqueueTargetedScanForTarget(next)) {
                current_path_.clear();
                state_ = ExplorationState::Scanning;
                return nextScanningStep();
            }
            current_path_.clear();
            break;
        }
        current_path_.erase(current_path_.begin());
        enqueueCommandsForStep(next);
        if (!pending_commands_.empty()) {
            state_ = ExplorationState::Moving;
            return nextMovingStep();
        }
        current_path_.clear();
        break;
    }

    // 2. BFS recovery: nearest strict frontier, then relaxed frontier (HW1).
    auto path = bfsToGoal(BfsGoalMode::Frontier6);
    if (path.empty()) path = bfsToGoal(BfsGoalMode::Frontier26);
    if (!path.empty()) {
        current_path_ = std::move(path);
        return nextPlanningStep();
    }

    // 3. Before giving up, scan an adjacent unknown cell (HW1).
    if (enqueueTargetedScanAroundCurrentPosition()) {
        state_ = ExplorationState::Scanning;
        return nextScanningStep();
    }

    state_ = ExplorationState::Finished;
    return finishedCommand();
}

// ── Public entry point ───────────────────────────────────────────────────────
types::MappingStepCommand MappingAlgorithmImpl::nextStep(
    const types::DroneState& state,
    const types::LidarScanResult* latest_scan) {

    current_position_ = state.position;
    orientation_      = state.heading;
    if (latest_scan != nullptr) ingestScan(state, *latest_scan);

    switch (state_) {
        case ExplorationState::Scanning: return nextScanningStep();
        case ExplorationState::Planning: return nextPlanningStep();
        case ExplorationState::Moving:   return nextMovingStep();
        case ExplorationState::Finished: return finishedCommand();
    }
    state_ = ExplorationState::Finished;
    return finishedCommand();
}

} // namespace drone_mapper
