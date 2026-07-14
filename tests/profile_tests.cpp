// Profile-math tests — trapezoid + S-curve — driven through motion's PUBLIC
// surface (robonode::MotionPlan), never trajlib directly. trajlib is a
// motion-private implementation detail (scripts/check-boundaries.sh); these
// assert the same properties the profiles must hold, via the seam the rest of
// the system actually uses. MotionProfile.jerk > 0 selects S-curve, jerk == 0
// selects trapezoidal (robonode-idl/common.proto semantics).
//
// Units are arbitrary and self-consistent (MotionPlan is unit-agnostic); the
// small SI-ish numbers match the profiles' original coverage.

#include <cmath>
#include <cstdio>

#include "check.hpp"
#include "robonode/motion/motion_plan.hpp"

namespace {

constexpr robonode::MotionProfile kTrap{.velocity = 0.25, .acceleration = 1.0, .jerk = 0.0};
constexpr robonode::MotionProfile kScurve{.velocity = 0.25, .acceleration = 1.0, .jerk = 10.0};
constexpr double kDt = 1e-4;

bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

// --- trapezoid ---------------------------------------------------------------

void test_trapezoid_reaches_goal_at_rest() {
    const auto p = robonode::MotionPlan::move(0.0, 0.5, kTrap);
    const auto end = p.sample(p.duration_s());
    CHECK(near(end.position, 0.5, 1e-9));
    CHECK(near(end.velocity, 0.0, 1e-9));
}

void test_trapezoid_respects_velocity_limit() {
    const auto p = robonode::MotionPlan::move(0.0, 0.5, kTrap);
    for (double t = 0.0; t <= p.duration_s(); t += 1e-3) {
        CHECK(std::abs(p.sample(t).velocity) <= kTrap.velocity + 1e-9);
    }
}

void test_trapezoid_triangular_when_too_short_to_cruise() {
    // d = 0.01, a = 1 → peak velocity sqrt(0.01) = 0.1 < v_max.
    const auto p = robonode::MotionPlan::move(0.0, 0.01, kTrap);
    double peak = 0.0;
    for (double t = 0.0; t <= p.duration_s(); t += 5e-4) {
        peak = std::max(peak, p.sample(t).velocity);
    }
    CHECK(near(peak, 0.1, 1e-3));
}

void test_trapezoid_negative_displacement_mirrors() {
    const auto p = robonode::MotionPlan::move(0.5, 0.0, kTrap);
    CHECK(near(p.sample(p.duration_s()).position, 0.0, 1e-9));
    CHECK(p.sample(p.duration_s() / 2.0).velocity < 0.0);
}

void test_trapezoid_sample_clamps_outside_domain() {
    const auto p = robonode::MotionPlan::move(0.0, 0.5, kTrap);
    CHECK(near(p.sample(-1.0).position, 0.0, 1e-9));
    CHECK(near(p.sample(p.duration_s() + 1.0).position, 0.5, 1e-9));
}

// --- S-curve -----------------------------------------------------------------

void test_scurve_reaches_goal_at_rest_zero_accel() {
    const auto p = robonode::MotionPlan::move(0.0, 0.5, kScurve);
    const auto end = p.sample(p.duration_s());
    CHECK(near(end.position, 0.5, 1e-6));
    CHECK(near(end.velocity, 0.0, 1e-6));
    CHECK(near(end.acceleration, 0.0, 1e-6));  // the point of jerk limiting
}

void test_scurve_respects_all_three_limits() {
    const auto p = robonode::MotionPlan::move(0.0, 0.5, kScurve);
    auto prev = p.sample(0.0);
    for (double t = kDt; t <= p.duration_s(); t += kDt) {
        const auto s = p.sample(t);
        CHECK(std::abs(s.velocity) <= kScurve.velocity + 1e-9);
        CHECK(std::abs(s.acceleration) <= kScurve.acceleration + 1e-9);
        CHECK(std::abs(s.acceleration - prev.acceleration) / kDt <= kScurve.jerk + 1e-3);
        prev = s;
    }
}

void test_scurve_acceleration_is_continuous() {
    const auto p = robonode::MotionPlan::move(0.0, 0.5, kScurve);
    auto prev = p.sample(0.0);
    double max_step = 0.0;
    for (double t = kDt; t <= p.duration_s(); t += kDt) {
        const auto s = p.sample(t);
        max_step = std::max(max_step, std::abs(s.acceleration - prev.acceleration));
        prev = s;
    }
    // Bounded by j_max * dt — a trapezoid would jump by a_max = 1.0 here.
    CHECK(max_step <= kScurve.jerk * kDt + 1e-9);
}

void test_scurve_short_move_never_reaches_vmax() {
    const auto p = robonode::MotionPlan::move(0.0, 0.01, kScurve);
    double peak_v = 0.0;
    for (double t = 0.0; t <= p.duration_s(); t += kDt) {
        peak_v = std::max(peak_v, p.sample(t).velocity);
    }
    CHECK(peak_v < kScurve.velocity);
    CHECK(near(p.sample(p.duration_s()).position, 0.01, 1e-6));
}

void test_scurve_tiny_move_never_reaches_amax() {
    // Pure jerk-bounded: d small enough that accel stays triangular.
    const auto p = robonode::MotionPlan::move(0.0, 1e-4, kScurve);
    double peak_a = 0.0;
    for (double t = 0.0; t <= p.duration_s(); t += kDt) {
        peak_a = std::max(peak_a, std::abs(p.sample(t).acceleration));
    }
    CHECK(peak_a < kScurve.acceleration);
    CHECK(near(p.sample(p.duration_s()).position, 1e-4, 1e-9));
}

void test_scurve_negative_displacement_mirrors() {
    const auto p = robonode::MotionPlan::move(0.5, 0.0, kScurve);
    CHECK(near(p.sample(p.duration_s()).position, 0.0, 1e-6));
    CHECK(p.sample(p.duration_s() / 2.0).velocity < 0.0);
}

void test_scurve_slower_than_trapezoid_by_jerk_ramps() {
    const auto trap = robonode::MotionPlan::move(0.0, 0.5, kTrap);
    const auto scurve = robonode::MotionPlan::move(0.0, 0.5, kScurve);
    CHECK(scurve.duration_s() > trap.duration_s());
    // At most ~one jerk ramp (a_max/j_max) per accel transition slower.
    CHECK(scurve.duration_s() < trap.duration_s() + 4.0 * (kScurve.acceleration / kScurve.jerk) + 1e-9);
}

}  // namespace

int main() {
    test_trapezoid_reaches_goal_at_rest();
    test_trapezoid_respects_velocity_limit();
    test_trapezoid_triangular_when_too_short_to_cruise();
    test_trapezoid_negative_displacement_mirrors();
    test_trapezoid_sample_clamps_outside_domain();
    test_scurve_reaches_goal_at_rest_zero_accel();
    test_scurve_respects_all_three_limits();
    test_scurve_acceleration_is_continuous();
    test_scurve_short_move_never_reaches_vmax();
    test_scurve_tiny_move_never_reaches_amax();
    test_scurve_negative_displacement_mirrors();
    test_scurve_slower_than_trapezoid_by_jerk_ramps();
    std::puts("robonode motion profiles: all tests passed");
    return 0;
}
