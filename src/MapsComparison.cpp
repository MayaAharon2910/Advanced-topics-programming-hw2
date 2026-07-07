#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Units.h>

#include <algorithm>
#include <numeric>
#include <cmath>

namespace drone_mapper {

// Compare each target against the overlapping world-space region of the original map.
std::vector<double> MapsComparison::compare(const IMap3D& original,
                               const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    try {
        const drone_mapper::types::MapConfig orig_cfg = original.getMapConfig();

        for (const auto* target : targets) {
            if (target == nullptr) {
                scores.push_back(-1.0);
                continue;
            }
            const drone_mapper::types::MapConfig tgt_cfg = target->getMapConfig();

            // Determine overlapping world-space bounds
            const double ox_min = orig_cfg.boundaries.min_x.numerical_value_in(drone_mapper::cm);
            const double oy_min = orig_cfg.boundaries.min_y.numerical_value_in(drone_mapper::cm);
            const double oz_min = orig_cfg.boundaries.min_height.numerical_value_in(drone_mapper::cm);
            const double ox_max = orig_cfg.boundaries.max_x.numerical_value_in(drone_mapper::cm);
            const double oy_max = orig_cfg.boundaries.max_y.numerical_value_in(drone_mapper::cm);
            const double oz_max = orig_cfg.boundaries.max_height.numerical_value_in(drone_mapper::cm);

            const double tx_min = tgt_cfg.boundaries.min_x.numerical_value_in(drone_mapper::cm);
            const double ty_min = tgt_cfg.boundaries.min_y.numerical_value_in(drone_mapper::cm);
            const double tz_min = tgt_cfg.boundaries.min_height.numerical_value_in(drone_mapper::cm);
            const double tx_max = tgt_cfg.boundaries.max_x.numerical_value_in(drone_mapper::cm);
            const double ty_max = tgt_cfg.boundaries.max_y.numerical_value_in(drone_mapper::cm);
            const double tz_max = tgt_cfg.boundaries.max_height.numerical_value_in(drone_mapper::cm);

            const double ix_min = std::max(ox_min, tx_min);
            const double iy_min = std::max(oy_min, ty_min);
            const double iz_min = std::max(oz_min, tz_min);

            const double ix_max = std::min(ox_max, tx_max);
            const double iy_max = std::min(oy_max, ty_max);
            const double iz_max = std::min(oz_max, tz_max);

            if (ix_min >= ix_max || iy_min >= iy_max || iz_min >= iz_max) {
                // No overlap -> score 0
                scores.push_back(0.0);
                continue;
            }

            // BONUS FEATURE: Cross-Resolution Support
            // When the two maps have different resolutions (in cm), compute a sampling
            // step that aligns with both maps where possible. We compute the greatest
            // common divisor (in integer cm) of the two resolutions and use that as
            // the sampling step. If resolutions are non-integer or gcd is zero,
            // fall back to the finer (smaller) resolution.
            const double res1 = orig_cfg.resolution.force_numerical_value_in(drone_mapper::cm);
            const double res2 = tgt_cfg.resolution.force_numerical_value_in(drone_mapper::cm);
            double step = 1.0;
            if (res1 > 0.0 && res2 > 0.0) {
                const long r1 = static_cast<long>(std::lround(res1));
                const long r2 = static_cast<long>(std::lround(res2));
                if (r1 > 0 && r2 > 0) {
                    const long g = std::gcd(r1, r2);
                    if (g > 0) step = static_cast<double>(g);
                    else step = std::min(res1, res2);
                } else {
                    step = std::min(res1, res2);
                }
            } else {
                step = std::min(res1 > 0.0 ? res1 : 1.0, res2 > 0.0 ? res2 : 1.0);
            }

            size_t total = 0;
            size_t matches = 0;

            for (double x = ix_min; x < ix_max; x += step) {
                for (double y = iy_min; y < iy_max; y += step) {
                    for (double z = iz_min; z < iz_max; z += step) {
                        drone_mapper::Position3D p{
                            drone_mapper::XLength{x * drone_mapper::cm},
                            drone_mapper::YLength{y * drone_mapper::cm},
                            drone_mapper::ZLength{z * drone_mapper::cm}
                        };
                        const auto a = original.atVoxel(p);
                        const auto b = target->atVoxel(p);
                        if (a == b) ++matches;
                        ++total;
                    }
                }
            }

            if (total == 0) {
                scores.push_back(0.0);
            } else {
                double sc = 100.0 * static_cast<double>(matches) / static_cast<double>(total);
                scores.push_back(sc);
            }
        }
    } catch (const std::exception& ex) {
        (void)ex;
        // On any exception, return a -1 score per target.
        scores.clear();
        for (size_t i = 0; i < targets.size(); ++i) scores.push_back(-1.0);
    }

    return scores;
}

} // namespace drone_mapper
