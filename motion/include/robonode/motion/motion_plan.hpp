#pragma once

#include <variant>

#include "robonode/core/limits.hpp"
#include "robonode/core/state.hpp"

// trajlib is an implementation detail of the motion module — nothing
// outside motion/ may include it (enforced by scripts/check-boundaries.sh).
#include "trajlib/scurve.hpp"
#include "trajlib/trapezoidal.hpp"

namespace robonode {

// Planned rest-to-rest motion for one axis, sampled per RT cycle. Profile
// math comes from trajlib (motion-private, include/trajlib/); the public
// surface speaks only core types. profile.jerk == 0 selects trapezoidal explicitly
// (robonode-idl/common.proto semantics: jerk honored, not decorative).
//
// This is the M0/M1 stand-in for the OTG slot (SPEC §3.1): sampling a
// precomputed profile. The Ruckig-class OTG replaces it to make motion
// retargetable mid-flight (FR-2.6); the executive's per-cycle interface is
// already shaped for that swap.
class MotionPlan {
public:
    // Factory throws std::invalid_argument on non-positive limits — creation
    // is a configuration-surface verb (see core/status.hpp discipline; the
    // celld-facing wrapper converts to Status at the module boundary).
    static MotionPlan move(double start, double goal, const MotionProfile& profile) {
        if (profile.jerk > 0.0) {
            return MotionPlan{trajlib::SCurveProfile{
                start, goal, {profile.velocity, profile.acceleration, profile.jerk}}};
        }
        return MotionPlan{
            trajlib::TrapezoidalProfile{start, goal, {profile.velocity, profile.acceleration}}};
    }

    [[nodiscard]] double duration_s() const {
        return std::visit([](const auto& p) { return p.duration(); }, profile_);
    }

    [[nodiscard]] State sample(double t_s) const {
        const trajlib::State s =
            std::visit([t_s](const auto& p) { return p.sample(t_s); }, profile_);
        return {s.position, s.velocity, s.acceleration};
    }

private:
    using Profile = std::variant<trajlib::TrapezoidalProfile, trajlib::SCurveProfile>;
    explicit MotionPlan(Profile p) : profile_{std::move(p)} {}
    Profile profile_;
};

}  // namespace robonode
