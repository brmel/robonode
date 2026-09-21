#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace trajlib {

// Kinematic state sampled at time t along a 1-DOF profile.
struct State {
    double position{};
    double velocity{};
    double acceleration{};
};

struct Limits {
    double max_velocity;      // > 0
    double max_acceleration;  // > 0
};

// Trapezoidal velocity profile: accelerate at a_max, cruise at v_max,
// decelerate at a_max. Degenerates to a triangular profile when the
// distance is too short to reach v_max.
//
// Reference implementation for Exercise 1 — re-derive the phase math on
// paper before reading. Flaw to remember: acceleration steps between
// ±a_max and 0, i.e. unbounded jerk. See scurve.hpp for the fix.
class TrapezoidalProfile {
public:
    // Rest-to-rest move from `start` to `goal`.
    TrapezoidalProfile(double start, double goal, Limits limits)
        : start_{start}, sign_{goal >= start ? 1.0 : -1.0} {
        if (limits.max_velocity <= 0.0 || limits.max_acceleration <= 0.0) {
            throw std::invalid_argument{"limits must be positive"};
        }
        const double d = std::abs(goal - start);
        const double a = limits.max_acceleration;

        // Triangular if both ramps alone would overshoot: v_max^2 / a >= d.
        v_peak_ = std::min(limits.max_velocity, std::sqrt(a * d));
        accel_ = a;
        t_accel_ = v_peak_ / a;
        t_cruise_ = v_peak_ > 0.0 ? (d - v_peak_ * v_peak_ / a) / v_peak_ : 0.0;
    }

    [[nodiscard]] double duration() const { return 2.0 * t_accel_ + t_cruise_; }

    [[nodiscard]] State sample(double t) const {
        t = std::clamp(t, 0.0, duration());
        const double d_ramp = 0.5 * accel_ * t_accel_ * t_accel_;

        double p, v, a;
        if (t < t_accel_) {  // accelerate
            p = 0.5 * accel_ * t * t;
            v = accel_ * t;
            a = accel_;
        } else if (t < t_accel_ + t_cruise_) {  // cruise
            p = d_ramp + v_peak_ * (t - t_accel_);
            v = v_peak_;
            a = 0.0;
        } else {  // decelerate
            const double tau = t - t_accel_ - t_cruise_;
            p = d_ramp + v_peak_ * t_cruise_ + v_peak_ * tau - 0.5 * accel_ * tau * tau;
            v = v_peak_ - accel_ * tau;
            a = -accel_;
        }
        return {start_ + sign_ * p, sign_ * v, sign_ * a};
    }

private:
    double start_;
    double sign_;
    double v_peak_{};
    double accel_{};
    double t_accel_{};
    double t_cruise_{};
};

}  // namespace trajlib
