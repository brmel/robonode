#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "trajlib/trapezoidal.hpp"  // State

namespace trajlib {

struct JerkLimits {
    double max_velocity;      // > 0
    double max_acceleration;  // > 0
    double max_jerk;          // > 0
};

// Seven-phase jerk-limited ("S-curve") rest-to-rest profile:
//   [+j, 0, -j]  accelerate,  [0]  cruise,  [-j, 0, +j]  decelerate.
// Acceleration is continuous (C2 position), which is the whole point.
//
// Case tree (Biagiotti & Melchiorri ch. 3.4):
//   (a) is a_max reachable?   v_max * j >= a_max^2  → yes, else accel is
//       triangular with peak a = sqrt(v_max * j)
//   (b) is v_max reachable?   if cruise time goes negative, recompute the
//       accel phase for the actual peak velocity.
//
// Implementation strategy: derive the phase DURATIONS from the case tree,
// then integrate constant-jerk segments analytically. The segment list is
// the single source of truth — no per-phase closed forms to get wrong.
class SCurveProfile {
public:
    SCurveProfile(double start, double goal, JerkLimits lim)
        : start_{start}, sign_{goal >= start ? 1.0 : -1.0} {
        if (lim.max_velocity <= 0.0 || lim.max_acceleration <= 0.0 || lim.max_jerk <= 0.0) {
            throw std::invalid_argument{"limits must be positive"};
        }
        const double d = std::abs(goal - start);
        const double j = lim.max_jerk;
        const double a = lim.max_acceleration;
        const double v = lim.max_velocity;

        if (d == 0.0) {
            segments_ = {};
            n_segments_ = 0;
            return;
        }

        // Shape of the accel phase, assuming v_max is reached.
        double tj, ta;  // tj: jerk ramp time, ta: total accel phase time
        if (v * j >= a * a) {
            tj = a / j;          // a_max reached, trapezoidal acceleration
            ta = tj + v / a;
        } else {
            tj = std::sqrt(v / j);  // a_max never reached
            ta = 2.0 * tj;
        }

        // Cruise length; negative means v_max is never reached.
        double tv = d / v - ta;  // accel covers v*ta/2, decel mirrors → d = v*(ta + tv)
        if (tv < 0.0) {
            tv = 0.0;
            tj = a / j;
            ta = tj / 2.0 + std::sqrt(tj * tj / 4.0 + d / a);
            if (ta < 2.0 * tj) {
                // a_max not reachable either: pure jerk phases
                tj = std::cbrt(d / (2.0 * j));
                ta = 2.0 * tj;
            }
        }

        const double tconst = ta - 2.0 * tj;  // time at constant acceleration
        segments_ = {{
            {tj, +j}, {tconst, 0.0}, {tj, -j},   // accelerate
            {tv, 0.0},                            // cruise
            {tj, -j}, {tconst, 0.0}, {tj, +j},   // decelerate
        }};
        n_segments_ = segments_.size();
    }

    [[nodiscard]] double duration() const {
        double T = 0.0;
        for (std::size_t i = 0; i < n_segments_; ++i) T += segments_[i].dt;
        return T;
    }

    [[nodiscard]] State sample(double t) const {
        t = std::clamp(t, 0.0, duration());
        double p = 0.0, v = 0.0, a = 0.0;
        for (std::size_t i = 0; i < n_segments_; ++i) {
            const double dt = std::min(t, segments_[i].dt);
            const double j = segments_[i].jerk;
            p += v * dt + 0.5 * a * dt * dt + j * dt * dt * dt / 6.0;
            v += a * dt + 0.5 * j * dt * dt;
            a += j * dt;
            t -= dt;
            if (t <= 0.0) break;
        }
        return {start_ + sign_ * p, sign_ * v, sign_ * a};
    }

private:
    struct Segment {
        double dt;
        double jerk;
    };

    double start_;
    double sign_;
    std::array<Segment, 7> segments_{};
    std::size_t n_segments_{};
};

}  // namespace trajlib
