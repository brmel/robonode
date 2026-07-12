#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "trajlib/blend.hpp"

using namespace trajlib;
using Catch::Matchers::WithinAbs;

namespace {
// 90-degree corner: (0,0) -> (1,0) -> (1,1) at 0.5 m/s, 2 m/s^2
const ParabolicBlend kCorner{{0, 0}, {1, 0}, {1, 1}, 0.5, 2.0};
}

TEST_CASE("blend starts at p0 and ends at p1") {
    const State2 a = kCorner.sample(0.0);
    const State2 b = kCorner.sample(kCorner.duration());
    CHECK_THAT(a.position.x, WithinAbs(0.0, 1e-9));
    CHECK_THAT(a.position.y, WithinAbs(0.0, 1e-9));
    CHECK_THAT(b.position.x, WithinAbs(1.0, 1e-9));
    CHECK_THAT(b.position.y, WithinAbs(1.0, 1e-9));
}

TEST_CASE("velocity is continuous through the corner") {
    const double dt = 1e-4;
    State2 prev = kCorner.sample(0.0);
    for (double t = dt; t <= kCorner.duration(); t += dt) {
        const State2 s = kCorner.sample(t);
        const double dv = (s.velocity - prev.velocity).norm();
        CHECK(dv <= 2.0 * dt + 1e-6);  // bounded by accel * dt
        prev = s;
    }
}

TEST_CASE("corner deviation matches |v2-v1| * tb / 8") {
    // measure actual minimum distance to via point
    double min_dist = 1e9;
    for (double t = 0.0; t <= kCorner.duration(); t += 1e-5) {
        const Vec2 d = kCorner.sample(t).position - Vec2{1, 0};
        min_dist = std::min(min_dist, d.norm());
    }
    CHECK_THAT(min_dist, WithinAbs(kCorner.corner_deviation(), 1e-4));
}

TEST_CASE("speed constant outside the blend") {
    const State2 s = kCorner.sample(0.1);
    CHECK_THAT(s.velocity.norm(), WithinAbs(0.5, 1e-9));
    const State2 e = kCorner.sample(kCorner.duration() - 0.1);
    CHECK_THAT(e.velocity.norm(), WithinAbs(0.5, 1e-9));
}

TEST_CASE("blend that cannot fit throws") {
    // segments 1 m, speed high + accel low -> huge blend time
    CHECK_THROWS_AS((ParabolicBlend{{0, 0}, {1, 0}, {1, 1}, 10.0, 0.1}),
                    std::invalid_argument);
}
