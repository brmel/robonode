#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "robonode/core/limits.hpp"
#include "robonode/core/settings.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// Refuses a trajectory BEFORE anything moves. Without this the governor is the
// only guard, and it enforces limits by clamping mid-motion — which silently
// changes the executed path instead of reporting that the plan was wrong.
class TrajectoryValidator {
public:
    explicit TrajectoryValidator(Settings::Motion cfg = {}) : cfg_{cfg} {}

    Status check(const std::vector<std::vector<double>>& waypoints,
                 const std::vector<AxisLimits>& limits,
                 const std::vector<std::string>& ids = {}) const {
        if (waypoints.size() != limits.size()) return Status::failure("waypoints/axes mismatch");
        double worst_s = 0.0;
        for (std::size_t i = 0; i < waypoints.size(); ++i) {
            if (const auto st = check_axis(waypoints[i], limits[i], name(ids, i)); !st.ok()) {
                return st;
            }
            worst_s = std::max(worst_s, travel_time(waypoints[i], limits[i]));
        }
        if (worst_s > cfg_.max_duration_s) {
            return Status::failure("trajectory would take " + std::to_string(worst_s) +
                                   " s — refusing (planning bug, not a long move)");
        }
        return Status::success();
    }

private:
    static std::string name(const std::vector<std::string>& ids, std::size_t i) {
        return i < ids.size() ? ids[i] : ("axis " + std::to_string(i));
    }

    // A physical axis can settle a hair past its soft limit; the plan starts at
    // that measured value. Tolerate a configured fraction of travel so
    // measurement noise is not reported as a planning error.
    double tolerance(const AxisLimits& lim) const {
        return cfg_.limit_tolerance * std::abs(lim.position_max - lim.position_min);
    }

    Status check_axis(const std::vector<double>& wp, const AxisLimits& lim,
                      const std::string& id) const {
        if (wp.empty()) return Status::failure(id + ": empty waypoint list");
        const double tol = tolerance(lim);
        for (const double q : wp) {
            if (!std::isfinite(q)) return Status::failure(id + ": non-finite waypoint");
            if (q < lim.position_min - tol || q > lim.position_max + tol) {
                return Status::failure(id + ": waypoint " + std::to_string(q) +
                                       " outside travel [" + std::to_string(lim.position_min) +
                                       ", " + std::to_string(lim.position_max) + "]");
            }
        }
        return Status::success();
    }

    static double travel_time(const std::vector<double>& wp, const AxisLimits& lim) {
        if (lim.velocity_max <= 0.0) return 0.0;
        double distance = 0.0;
        for (std::size_t k = 1; k < wp.size(); ++k) distance += std::abs(wp[k] - wp[k - 1]);
        return distance / lim.velocity_max;
    }

    Settings::Motion cfg_;
};

}  // namespace robonode
