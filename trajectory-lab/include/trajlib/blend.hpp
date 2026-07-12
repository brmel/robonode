#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace trajlib {

struct Vec2 {
    double x{}, y{};

    Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    [[nodiscard]] double norm() const { return std::hypot(x, y); }
};

struct State2 {
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;
};

// Parabolic blend through a via point (LSPB corner, 2-DOF).
//
// Path p0 → via → p1 traversed at constant speed `speed` along each linear
// segment; the corner at `via` is rounded by a constant-acceleration
// parabolic arc of duration t_b = |v2 - v1| / accel centered on the via
// arrival time.
//
// Known result to verify (and quote in interviews): the blend's deviation
// from the via point is |v2 - v1| * t_b / 8 = accel * t_b^2 / 8 — i.e. the
// "blend radius" knob users get (UR blend radius, Fanuc CNT) maps directly
// to t_b. Shorter blend = tighter corner = harder deceleration.
//
// Simplification: end segments are cruise-only (no ramp-up from rest);
// real stacks compose this with rest-to-rest profiles at both ends.
class ParabolicBlend {
public:
    ParabolicBlend(Vec2 p0, Vec2 via, Vec2 p1, double speed, double accel)
        : p0_{p0}, via_{via} {
        if (speed <= 0.0 || accel <= 0.0) {
            throw std::invalid_argument{"speed and accel must be positive"};
        }
        const Vec2 d1 = via - p0;
        const Vec2 d2 = p1 - via;
        if (d1.norm() == 0.0 || d2.norm() == 0.0) {
            throw std::invalid_argument{"degenerate segment"};
        }
        v1_ = d1 * (speed / d1.norm());
        v2_ = d2 * (speed / d2.norm());

        t_via_ = d1.norm() / speed;       // nominal via arrival time
        t_end_ = t_via_ + d2.norm() / speed;
        t_blend_ = (v2_ - v1_).norm() / accel;

        // Blend must fit inside both segments.
        if (t_blend_ / 2.0 > t_via_ || t_via_ + t_blend_ / 2.0 > t_end_) {
            throw std::invalid_argument{"blend does not fit segments; reduce speed or raise accel"};
        }
        accel_vec_ = t_blend_ > 0.0 ? (v2_ - v1_) * (1.0 / t_blend_) : Vec2{};
    }

    [[nodiscard]] double duration() const { return t_end_; }

    // Peak distance between the blended path and the via point.
    [[nodiscard]] double corner_deviation() const {
        return (v2_ - v1_).norm() * t_blend_ / 8.0;
    }

    [[nodiscard]] State2 sample(double t) const {
        t = std::clamp(t, 0.0, t_end_);
        const double tb0 = t_via_ - t_blend_ / 2.0;  // blend start
        const double tb1 = t_via_ + t_blend_ / 2.0;  // blend end

        if (t <= tb0) {  // first linear segment
            return {p0_ + v1_ * t, v1_, {}};
        }
        if (t < tb1) {  // parabolic corner
            const double tau = t - tb0;
            const Vec2 pb = p0_ + v1_ * tb0;
            return {pb + v1_ * tau + accel_vec_ * (0.5 * tau * tau),
                    v1_ + accel_vec_ * tau, accel_vec_};
        }
        // second linear segment: by symmetry the blend exits where the
        // nominal path would be at tb1 had it turned exactly at the via.
        return {via_ + v2_ * (t - t_via_), v2_, {}};
    }

private:
    Vec2 p0_, via_;
    Vec2 v1_, v2_, accel_vec_;
    double t_via_{}, t_end_{}, t_blend_{};
};

}  // namespace trajlib
