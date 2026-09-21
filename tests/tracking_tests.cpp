// The tracking capability: predicting where a moving workpiece will be. This is
// the seam a user replaces to solve the moving-target picking problem, so its
// failure mode (a tracker with no motion model) is pinned here deliberately.

#include <cmath>
#include <cstdio>

#include "check.hpp"
#include "robonode/sandbox/compiler.hpp"
#include "robonode/vision/sandboxed_tracker.hpp"
#include "robonode/vision/tracker.hpp"

namespace {

constexpr double kSpeed = 0.08;  // m/s along -x, like the belt

// A part sighted every 100 ms travelling at a constant speed.
template <typename T>
void feed(T& tracker, int samples, double t0 = 0.0) {
    for (int i = 0; i < samples; ++i) {
        const double t = t0 + 0.1 * i;
        tracker.observe({t, {1.4 - kSpeed * t, 0.25, 0.37}});
    }
}

// The naive tracker reports where the part WAS. Reaching for that is how a
// robot misses a moving target, and the platform ships it so the failure is
// something a user can see rather than a claim.
void test_snapshot_tracker_does_not_predict() {
    robonode::SnapshotTracker tracker;
    feed(tracker, 10);
    const auto predicted = tracker.predict(2.0);
    CHECK(predicted.has_value());
    CHECK(std::abs(predicted->x - (1.4 - kSpeed * 0.9)) < 1e-9);  // the last sighting, unchanged
    CHECK(tracker.velocity().norm() < 1e-9);
}

void test_constant_velocity_tracker_extrapolates() {
    robonode::ConstantVelocityTracker tracker{12};
    CHECK(!tracker.predict(1.0).has_value());  // nothing seen yet: say so, do not guess
    feed(tracker, 10);

    CHECK(std::abs(tracker.velocity().x + kSpeed) < 1e-6);  // travelling in -x
    CHECK(std::abs(tracker.velocity().y) < 1e-6);

    const double horizon = 2.0;
    const auto predicted = tracker.predict(horizon);
    CHECK(predicted.has_value());
    CHECK(std::abs(predicted->x - (1.4 - kSpeed * horizon)) < 1e-6);
    CHECK(std::abs(predicted->z - 0.37) < 1e-9);
}

// The smoothed estimator converges on a steady target and, unlike the window
// fit, cannot be knocked into a wild velocity by one bad sighting — it takes
// only a fraction of each disagreement.
void test_smoothed_tracker_converges_and_survives_a_jump() {
    robonode::SmoothedTracker tracker{0.4, 0.25, 0.15};
    CHECK(!tracker.predict(1.0).has_value());
    feed(tracker, 20);
    CHECK(std::abs(tracker.velocity().x + kSpeed) < 5e-3);  // converged on the belt speed

    const double horizon = 2.0;
    const auto predicted = tracker.predict(horizon);
    CHECK(predicted.has_value());
    CHECK(std::abs(predicted->x - (1.4 - kSpeed * horizon)) < 0.02);

    // A sighting far from the prediction is a different part: the track starts
    // over rather than claiming the velocity that would join the two.
    tracker.observe({2.1, {0.5, 0.25, 0.37}});
    CHECK(tracker.velocity().norm() < 1e-9);
    const auto after = tracker.predict(3.0);
    CHECK(after.has_value());
    CHECK(std::abs(after->x - 0.5) < 1e-9);
}

// A stale or repeated sighting must not poison the estimate.
void test_tracker_ignores_non_monotonic_observations() {
    robonode::ConstantVelocityTracker tracker{12};
    feed(tracker, 6);
    const auto before = tracker.velocity();
    tracker.observe({0.2, {99.0, 99.0, 99.0}});  // an old timestamp
    CHECK(std::abs(tracker.velocity().x - before.x) < 1e-12);
}

// The user-authored slot: the host measures, the program predicts.
void test_sandboxed_tracker_runs_user_prediction() {
    robonode::sandbox::Program program;
    CHECK(robonode::sandbox::Compiler::compile("x + vx * dt\ny\nz",
                                               {"x", "y", "z", "vx", "vy", "vz", "dt"}, 3, program)
              .ok());
    robonode::SandboxedTracker tracker{program, {}, 12, /*gate_m=*/0.25, /*max_travel_m=*/1.0};
    feed(tracker, 10);

    const auto predicted = tracker.predict(2.0);
    CHECK(predicted.has_value());
    CHECK(std::abs(predicted->x - (1.4 - kSpeed * 2.0)) < 1e-6);
}

// A part that jumps is a different part: the line reset, or a new one arrived.
// Folding it into the same track would invent a velocity that never happened.
void test_tracker_gates_a_new_part_into_a_fresh_track() {
    robonode::ConstantVelocityTracker tracker{12, /*gate_m=*/0.25};
    feed(tracker, 10);
    CHECK(std::abs(tracker.velocity().x + kSpeed) < 1e-6);

    // A sighting far from where this part should be by now: another object.
    tracker.observe({1.1, {0.5, 0.25, 0.37}});
    CHECK(tracker.velocity().norm() < 1e-9);  // one sighting claims no velocity
    const auto predicted = tracker.predict(2.0);
    CHECK(predicted.has_value());
    CHECK(std::abs(predicted->x - 0.5) < 1e-9);

    // A sighting where the part actually is keeps the track.
    robonode::ConstantVelocityTracker kept{12, 0.25};
    feed(kept, 10);
    kept.observe({1.0, {1.4 - kSpeed * 1.0, 0.25, 0.37}});
    CHECK(std::abs(kept.velocity().x + kSpeed) < 1e-6);
}

// A prediction is a claim about the near future, not a teleport: an absurd
// answer is refused before anything is planned against it.
void test_sandboxed_tracker_rejects_absurd_predictions() {
    robonode::sandbox::Program program;
    CHECK(robonode::sandbox::Compiler::compile("x + 1000\ny\nz",
                                               {"x", "y", "z", "vx", "vy", "vz", "dt"}, 3, program)
              .ok());
    robonode::SandboxedTracker tracker{program, {}, 12, /*gate_m=*/0.25, /*max_travel_m=*/1.0};
    feed(tracker, 5);
    CHECK(!tracker.predict(1.0).has_value());
}

}  // namespace

// A speed estimate needs a baseline in time. Sightings crowded into one instant
// fit a slope through noise and report a part crossing the cell — an
// interception then aims at a pose no arm can reach, which is how this surfaced.
void test_speed_needs_a_time_baseline() {
    robonode::ConstantVelocityTracker t{12, 0.25, 0.15};
    t.observe({0.100, {1.00, 0.25, 0.37}});
    t.observe({0.101, {1.02, 0.25, 0.37}});  // 2 cm in 1 ms is noise, not 20 m/s
    CHECK(t.velocity().norm() < 1e-9);

    t.observe({0.300, {0.98, 0.25, 0.37}});
    t.observe({0.500, {0.96, 0.25, 0.37}});
    CHECK(t.velocity().norm() > 0.0);  // spanning real time, it reports again
    CHECK(std::abs(t.velocity().x) < 1.0);
}

int main() {
    test_speed_needs_a_time_baseline();
    test_snapshot_tracker_does_not_predict();
    test_constant_velocity_tracker_extrapolates();
    test_smoothed_tracker_converges_and_survives_a_jump();
    test_tracker_ignores_non_monotonic_observations();
    test_tracker_gates_a_new_part_into_a_fresh_track();
    test_sandboxed_tracker_runs_user_prediction();
    test_sandboxed_tracker_rejects_absurd_predictions();
    std::puts("robonode tracking: all tests passed");
    return 0;
}
