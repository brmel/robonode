#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "trajlib/trapezoidal.hpp"

using namespace trajlib;
using Catch::Matchers::WithinAbs;

namespace {
constexpr Limits kLimits{.max_velocity = 0.25, .max_acceleration = 1.0};
}

TEST_CASE("reaches goal at rest") {
    const TrapezoidalProfile p{0.0, 0.5, kLimits};
    const State end = p.sample(p.duration());
    CHECK_THAT(end.position, WithinAbs(0.5, 1e-9));
    CHECK_THAT(end.velocity, WithinAbs(0.0, 1e-9));
}

TEST_CASE("respects velocity limit") {
    const TrapezoidalProfile p{0.0, 0.5, kLimits};
    for (double t = 0.0; t <= p.duration(); t += 0.001) {
        CHECK(std::abs(p.sample(t).velocity) <= kLimits.max_velocity + 1e-9);
    }
}

TEST_CASE("triangular when distance too short to cruise") {
    // d = 0.01 m, a = 1 → peak velocity sqrt(0.01) = 0.1 < v_max
    const TrapezoidalProfile p{0.0, 0.01, kLimits};
    double peak = 0.0;
    for (double t = 0.0; t <= p.duration(); t += 0.0005) {
        peak = std::max(peak, p.sample(t).velocity);
    }
    CHECK_THAT(peak, WithinAbs(0.1, 1e-3));
}

TEST_CASE("negative displacement mirrors") {
    const TrapezoidalProfile p{0.5, 0.0, kLimits};
    CHECK_THAT(p.sample(p.duration()).position, WithinAbs(0.0, 1e-9));
    CHECK(p.sample(p.duration() / 2).velocity < 0.0);
}

TEST_CASE("sample clamps outside [0, duration]") {
    const TrapezoidalProfile p{0.0, 0.5, kLimits};
    CHECK_THAT(p.sample(-1.0).position, WithinAbs(0.0, 1e-9));
    CHECK_THAT(p.sample(p.duration() + 1.0).position, WithinAbs(0.5, 1e-9));
}
