#pragma once

#include <utility>
#include <vector>

#include "robonode/core/state.hpp"
#include "robonode/motion/setpoint_source.hpp"

namespace robonode {

// A planner's output (#54). Two forms behind one type:
//   - waypoints[joint][step]: bare via-points the blend layer re-times (the v0
//     JointPlanner / CartesianLinePlanner form);
//   - a time-parameterized trajectory (times + q[joint][sample]): already
//     dynamically feasible, played directly — what cuRobo (#39) and Crocoddyl
//     (#41) produce, and which waypoint re-timing would throw away.
struct PlannedTrajectory {
    std::vector<std::vector<double>> waypoints;  // [joint][step]; empty if timed
    std::vector<double> times;                   // sample times (s); empty if waypoint-form
    std::vector<std::vector<double>> q;          // [joint][sample]; paired with times

    [[nodiscard]] bool is_timed() const { return !times.empty(); }

    static PlannedTrajectory from_waypoints(std::vector<std::vector<double>> wp) {
        return {std::move(wp), {}, {}};
    }
    static PlannedTrajectory from_timed(std::vector<double> t, std::vector<std::vector<double>> qq) {
        return {{}, std::move(t), std::move(qq)};
    }
};

// Plays one joint's timed samples into the executive's SetpointSource slot,
// linearly interpolating position (velocity = segment slope). A forward cursor
// keeps next() O(1) amortized for the monotonic clock. So a timed planner's
// output drives an axis through the same governor/executive as everything else.
class TimedAxisSource final : public SetpointSource {
public:
    TimedAxisSource(std::vector<double> times, std::vector<double> q)
        : t_{std::move(times)}, q_{std::move(q)} {}

    State next(double t, double /*dt*/) noexcept override {
        if (t_.empty()) return {};
        if (t <= t_.front()) return {q_.front(), 0.0, 0.0};
        if (t >= t_.back()) return {q_.back(), 0.0, 0.0};
        while (i_ + 1 < t_.size() && t_[i_ + 1] < t) ++i_;
        const double span = t_[i_ + 1] - t_[i_];
        const double u = span > 0.0 ? (t - t_[i_]) / span : 0.0;
        const double pos = q_[i_] + u * (q_[i_ + 1] - q_[i_]);
        const double vel = span > 0.0 ? (q_[i_ + 1] - q_[i_]) / span : 0.0;
        return {pos, vel, 0.0};
    }

    [[nodiscard]] double duration_s() const noexcept override {
        return t_.empty() ? 0.0 : t_.back();
    }

private:
    std::vector<double> t_, q_;
    std::size_t i_{0};
};

}  // namespace robonode
