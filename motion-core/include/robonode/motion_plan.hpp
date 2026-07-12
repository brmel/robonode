#pragma once

#include <variant>

#include "trajlib/scurve.hpp"
#include "trajlib/trapezoidal.hpp"

namespace robonode {

// Planned rest-to-rest motion for one axis, sampled per RT cycle. Profile
// math comes from trajlib (trajectory-lab). jerk == 0 in the wire
// MotionProfile selects trapezoidal explicitly (common.proto).
//
// This is the M0 stand-in for the OTG slot (SPEC §3.1): sampling a
// precomputed profile. The Ruckig-class OTG replaces it in M1+ to make
// motion retargetable mid-flight (FR-2.6); the executive's per-cycle
// interface is already shaped for that swap.
class MotionPlan {
public:
    static MotionPlan trapezoid(double start_mm, double goal_mm, trajlib::Limits lim) {
        return MotionPlan{trajlib::TrapezoidalProfile{start_mm, goal_mm, lim}};
    }

    static MotionPlan scurve(double start_mm, double goal_mm, trajlib::JerkLimits lim) {
        return MotionPlan{trajlib::SCurveProfile{start_mm, goal_mm, lim}};
    }

    [[nodiscard]] double duration_s() const {
        return std::visit([](const auto& p) { return p.duration(); }, profile_);
    }

    [[nodiscard]] trajlib::State sample(double t_s) const {
        return std::visit([t_s](const auto& p) { return p.sample(t_s); }, profile_);
    }

private:
    using Profile = std::variant<trajlib::TrapezoidalProfile, trajlib::SCurveProfile>;
    explicit MotionPlan(Profile p) : profile_{std::move(p)} {}
    Profile profile_;
};

}  // namespace robonode
