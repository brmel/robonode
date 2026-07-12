#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "trajlib/scurve.hpp"

using namespace trajlib;
using Catch::Matchers::WithinAbs;

namespace {
constexpr JerkLimits kLimits{.max_velocity = 0.25, .max_acceleration = 1.0, .max_jerk = 10.0};
constexpr double kDt = 1e-4;
}

TEST_CASE("scurve reaches goal at rest with zero acceleration") {
    const SCurveProfile p{0.0, 0.5, kLimits};
    const State end = p.sample(p.duration());
    CHECK_THAT(end.position, WithinAbs(0.5, 1e-6));
    CHECK_THAT(end.velocity, WithinAbs(0.0, 1e-6));
    CHECK_THAT(end.acceleration, WithinAbs(0.0, 1e-6));
}

TEST_CASE("scurve respects all three limits") {
    const SCurveProfile p{0.0, 0.5, kLimits};
    State prev = p.sample(0.0);
    for (double t = kDt; t <= p.duration(); t += kDt) {
        const State s = p.sample(t);
        CHECK(std::abs(s.velocity) <= kLimits.max_velocity + 1e-9);
        CHECK(std::abs(s.acceleration) <= kLimits.max_acceleration + 1e-9);
        // jerk via finite difference of acceleration
        CHECK(std::abs(s.acceleration - prev.acceleration) / kDt <= kLimits.max_jerk + 1e-3);
        prev = s;
    }
}

TEST_CASE("scurve acceleration is continuous (the point of jerk limiting)") {
    const SCurveProfile p{0.0, 0.5, kLimits};
    State prev = p.sample(0.0);
    double max_accel_step = 0.0;
    for (double t = kDt; t <= p.duration(); t += kDt) {
        const State s = p.sample(t);
        max_accel_step = std::max(max_accel_step, std::abs(s.acceleration - prev.acceleration));
        prev = s;
    }
    // bounded by j_max * dt — a trapezoid would jump by a_max = 1.0 here
    CHECK(max_accel_step <= kLimits.max_jerk * kDt + 1e-9);
}

TEST_CASE("scurve short move: v_max never reached") {
    const SCurveProfile p{0.0, 0.01, kLimits};
    double peak_v = 0.0;
    for (double t = 0.0; t <= p.duration(); t += kDt) {
        peak_v = std::max(peak_v, p.sample(t).velocity);
    }
    CHECK(peak_v < kLimits.max_velocity);
    CHECK_THAT(p.sample(p.duration()).position, WithinAbs(0.01, 1e-6));
}

TEST_CASE("scurve tiny move: a_max never reached either") {
    // pure jerk-bounded: d small enough that accel stays triangular
    const SCurveProfile p{0.0, 1e-4, kLimits};
    double peak_a = 0.0;
    for (double t = 0.0; t <= p.duration(); t += kDt) {
        peak_a = std::max(peak_a, std::abs(p.sample(t).acceleration));
    }
    CHECK(peak_a < kLimits.max_acceleration);
    CHECK_THAT(p.sample(p.duration()).position, WithinAbs(1e-4, 1e-9));
}

TEST_CASE("scurve negative displacement mirrors") {
    const SCurveProfile p{0.5, 0.0, kLimits};
    CHECK_THAT(p.sample(p.duration()).position, WithinAbs(0.0, 1e-6));
    CHECK(p.sample(p.duration() / 2.0).velocity < 0.0);
}

TEST_CASE("scurve is slower than trapezoid by ~one jerk ramp per transition") {
    const TrapezoidalProfile trap{0.0, 0.5, {.max_velocity = 0.25, .max_acceleration = 1.0}};
    const SCurveProfile s{0.0, 0.5, kLimits};
    CHECK(s.duration() > trap.duration());
    CHECK(s.duration() < trap.duration() + 4.0 * (kLimits.max_acceleration / kLimits.max_jerk) + 1e-9);
}
