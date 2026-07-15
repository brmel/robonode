// robonode::motion module tests.

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "check.hpp"
#include "robonode/motion/executive.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/motion_plan.hpp"
#include "robonode/motion/otg.hpp"
#include "robonode/motion/planner.hpp"
#include "robonode/motion/sim_axis.hpp"
#include "robonode/motion/sync_blend.hpp"
#include "robonode/motion/sync_executive.hpp"

namespace {

constexpr robonode::AxisLimits kLimits{
    .position_min = 0.0,
    .position_max = 1450.0,
    .velocity_max = 1200.0,
    .acceleration_max = 8000.0,
    .jerk_max = 120000.0,
};

constexpr robonode::MotionProfile kScurveProfile{
    .velocity = kLimits.velocity_max,
    .acceleration = kLimits.acceleration_max,
    .jerk = kLimits.jerk_max,
};

void test_governor_holds_position_envelope() {
    robonode::Governor gov{kLimits};
    gov.reset(1449.0);
    // Command far outside; governed output must never exceed the limit and
    // must approach it at <= vel_max.
    double p = 1449.0;
    for (int i = 0; i < 100; ++i) {
        const auto r = gov.apply(2000.0, 1e-3);
        CHECK(r.position <= kLimits.position_max + 1e-9);
        CHECK(r.position - p <= kLimits.velocity_max * 1e-3 + 1e-9);
        p = r.position;
    }
    CHECK(gov.position_clamps() > 0);
    CHECK(std::abs(p - kLimits.position_max) < 1e-6);
}

void test_governor_rate_limits_jumps() {
    robonode::Governor gov{kLimits};
    gov.reset(0.0);
    const auto r = gov.apply(100.0, 1e-3);  // 100 mm jump in 1 ms = 100 m/s
    CHECK(std::abs(r.position - kLimits.velocity_max * 1e-3) < 1e-6);
    CHECK(gov.velocity_clamps() == 1);
}

void test_governor_rejects_nonfinite_and_bad_dt() {
    robonode::Governor gov{kLimits};
    gov.reset(100.0);
    const auto nan = gov.apply(std::nan(""), 1e-3);
    CHECK(nan.clamped);
    CHECK(nan.position == 100.0);  // holds previous, never propagates NaN
    const auto inf = gov.apply(std::numeric_limits<double>::infinity(), 1e-3);
    CHECK(inf.position == 100.0);
    const auto bad_dt = gov.apply(200.0, 0.0);
    CHECK(bad_dt.position == 100.0);
    CHECK(gov.rejected_setpoints() == 3);
    // Recovers: next finite setpoint governed normally.
    const auto ok = gov.apply(100.5, 1e-3);
    CHECK(std::abs(ok.position - 100.5) < 1e-9);
}

void test_adapter_lifecycle_defaults() {
    robonode::SimAxis axis{"t", 0.0, 0.005};
    CHECK(axis.lifecycle() == robonode::Lifecycle::kUnconfigured);
    CHECK(axis.configure().ok());
    CHECK(axis.lifecycle() == robonode::Lifecycle::kInactive);
    CHECK(axis.activate().ok());
    CHECK(axis.lifecycle() == robonode::Lifecycle::kActive);
    CHECK(axis.deactivate().ok());
    CHECK(axis.lifecycle() == robonode::Lifecycle::kInactive);
}

void test_executive_completes_scurve_move() {
    robonode::SimAxis axis{"t", 0.0, 0.005};
    robonode::Governor gov{kLimits};
    robonode::Executive exec{axis, gov, 1000.0};
    std::vector<robonode::TelemetryRow> rows;
    const auto plan = robonode::MotionPlan::move(0.0, 500.0, kScurveProfile);
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.1);

    CHECK(stats.cycles == rows.size());
    // In-envelope plan: governor must not have intervened.
    CHECK(gov.position_clamps() == 0);
    CHECK(gov.velocity_clamps() == 0);
    // Axis settled on target within 0.5 mm.
    CHECK(std::abs(rows.back().actual_position - 500.0) < 0.5);
    // Governed setpoints respect the velocity envelope cycle-to-cycle.
    for (std::size_t i = 1; i < rows.size(); ++i) {
        const double dp = rows[i].governed_position - rows[i - 1].governed_position;
        CHECK(std::abs(dp) <= kLimits.velocity_max * 1e-3 + 1e-6);
    }
}

// Wraps SimAxis and trips PROTECTIVE_STOP for executive cycles
// [trip_at, clear_at) — the fault-injection shape sim-gate suites use.
class FaultTimedAxis final : public robonode::AxisAdapter {
public:
    FaultTimedAxis(std::uint64_t trip_at, std::uint64_t clear_at)
        : trip_{trip_at}, clear_{clear_at} {}

    void write_setpoint(double p) noexcept override { inner_.write_setpoint(p); }
    [[nodiscard]] robonode::AxisState read() const noexcept override { return inner_.read(); }
    void step(double dt) noexcept override {
        ++cycle_;
        inner_.set_safety(cycle_ >= trip_ && cycle_ < clear_
                              ? robonode::SafetyState::kProtectiveStop
                              : robonode::SafetyState::kNormal);
        inner_.step(dt);
    }
    [[nodiscard]] std::string name() const override { return "fault-timed"; }

private:
    robonode::SimAxis inner_{"inner", 0.0, 0.005};
    std::uint64_t cycle_{}, trip_, clear_;
};

void test_executive_holds_on_protective_stop_and_recovers() {
    FaultTimedAxis axis{100, 300};  // stop trips at cycle 100, clears at 300
    robonode::Governor gov{kLimits};
    robonode::Executive exec{axis, gov, 1000.0};
    std::vector<robonode::TelemetryRow> rows;
    const auto plan = robonode::MotionPlan::move(0.0, 500.0, kScurveProfile);
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.5);

    CHECK(stats.safety_hold_cycles == 200);
    // Command frozen across the whole hold window.
    for (std::size_t i = 101; i < 300; ++i) {
        CHECK(rows[i].governed_position == rows[100].governed_position);
    }
    // Catch-up after recovery stays inside the velocity envelope...
    for (std::size_t i = 300; i < rows.size(); ++i) {
        const double dp = rows[i].governed_position - rows[i - 1].governed_position;
        CHECK(std::abs(dp) <= kLimits.velocity_max * 1e-3 + 1e-6);
    }
    // ...and the move still completes.
    CHECK(std::abs(rows.back().actual_position - 500.0) < 0.5);
}

constexpr robonode::AxisLimits kAuxLimits{
    .position_min = -10.0,
    .position_max = 100.0,
    .velocity_max = 300.0,
    .acceleration_max = 2000.0,
    .jerk_max = 0.0,
};

// Monotonic waypoints: the true pass-through case — small corner cut,
// nonzero velocity at the via (the Vention gap: no stop-and-go).
void test_sync_plan_passes_through_monotonic_via() {
    const std::vector<std::vector<double>> wp = {{0.0, 500.0, 800.0}, {0.0, 45.0, 90.0}};
    const auto plan = robonode::SyncBlendPlan::plan(wp, {kLimits, kAuxLimits});

    const double T = plan.duration();
    const robonode::AxisLimits lims[2] = {kLimits, kAuxLimits};
    double best_d[2] = {1e9, 1e9};
    double v_at_best[2] = {0.0, 0.0};
    for (double t = 0.0; t <= T; t += 1e-4) {
        for (std::size_t i = 0; i < 2; ++i) {
            const auto s = plan.sample(i, t);
            CHECK(std::abs(s.velocity) <= lims[i].velocity_max * 1.001);
            CHECK(std::abs(s.acceleration) <= lims[i].acceleration_max * 1.001);
            const double d = std::abs(s.position - wp[i][1]);
            if (d < best_d[i]) {
                best_d[i] = d;
                v_at_best[i] = s.velocity;
            }
        }
    }
    for (std::size_t i = 0; i < 2; ++i) {
        const auto end = plan.sample(i, T);
        CHECK(std::abs(end.position - wp[i][2]) < 1e-6);      // exact arrival
        CHECK(std::abs(end.velocity) < 1e-9);                 // at rest
        CHECK(best_d[i] < 0.05 * std::abs(wp[i][1] - wp[i][0]));  // tight corner
        CHECK(std::abs(v_at_best[i]) > 0.1 * lims[i].velocity_max);  // moving
    }
}

// Direction-reversal corner: deviation is inherently |Δv|·t_b/8 (large) and
// velocity crosses zero at closest approach — assert limits + endpoints
// only; the corner-cut is the physics, not a bug.
void test_sync_plan_reversal_corner_respects_limits() {
    const std::vector<std::vector<double>> wp = {{0.0, 500.0, 300.0}, {0.0, 90.0, 45.0}};
    const auto plan = robonode::SyncBlendPlan::plan(wp, {kLimits, kAuxLimits});
    const double T = plan.duration();
    const robonode::AxisLimits lims[2] = {kLimits, kAuxLimits};
    for (double t = 0.0; t <= T; t += 1e-4) {
        for (std::size_t i = 0; i < 2; ++i) {
            const auto s = plan.sample(i, t);
            CHECK(std::abs(s.velocity) <= lims[i].velocity_max * 1.001);
            CHECK(std::abs(s.acceleration) <= lims[i].acceleration_max * 1.001);
        }
    }
    for (std::size_t i = 0; i < 2; ++i) {
        const auto end = plan.sample(i, T);
        CHECK(std::abs(end.position - wp[i][2]) < 1e-6);
        CHECK(std::abs(end.velocity) < 1e-9);
    }
}

void test_otg_reaches_target_jerk_limited() {
    const double dt = 1e-3;
    robonode::Otg otg{1000.0, kLimits, 0.0};
    otg.retarget(500.0);
    double prev_a = 0.0;
    double t = 0.0;
    int guard = 0;
    while (!otg.done() && guard++ < 5000) {
        const auto s = otg.next(t, dt);
        CHECK(std::abs(s.velocity) <= kLimits.velocity_max * 1.001);
        CHECK(std::abs(s.acceleration) <= kLimits.acceleration_max * 1.001);
        // Jerk-limited: acceleration changes at most jerk·dt per cycle.
        CHECK(std::abs(s.acceleration - prev_a) <= kLimits.jerk_max * dt * 1.01);
        prev_a = s.acceleration;
        t += dt;
    }
    const auto end = otg.next(t, dt);
    CHECK(std::abs(end.position - 500.0) < 1e-3);
    CHECK(std::abs(end.velocity) < 1e-6);
}

void test_otg_retargets_mid_flight_smoothly() {
    const double dt = 1e-3;
    robonode::Otg otg{1000.0, kLimits, 0.0};
    otg.retarget(500.0);
    double t = 0.0;
    robonode::State before{};
    for (int i = 0; i < 200; ++i) {  // 0.2 s toward 500
        before = otg.next(t, dt);
        t += dt;
    }
    CHECK(std::abs(before.velocity) > 100.0);  // genuinely mid-flight
    otg.retarget(200.0);                       // reverse!
    const auto after = otg.next(t, dt);
    // Continuity across the retarget: velocity cannot jump more than a·dt.
    CHECK(std::abs(after.velocity - before.velocity) <=
          kLimits.acceleration_max * dt * 1.05);
    int guard = 0;
    while (!otg.done() && guard++ < 10000) {
        otg.next(t, dt);
        t += dt;
    }
    const auto end = otg.next(t, dt);
    CHECK(std::abs(end.position - 200.0) < 1e-3);
}

// Test utility: applies scheduled retargets on the shared clock — the shape
// a jog session or Tier B stream takes through the same slot.
class ScheduledRetargets final : public robonode::SetpointSource {
public:
    ScheduledRetargets(robonode::Otg& otg, std::vector<std::pair<double, double>> schedule)
        : otg_{otg}, schedule_{std::move(schedule)} {}

    robonode::State next(double t, double dt) noexcept override {
        while (i_ < schedule_.size() && t >= schedule_[i_].first) {
            otg_.retarget(schedule_[i_++].second);
        }
        return otg_.next(t, dt);
    }
    [[nodiscard]] double duration_s() const noexcept override { return otg_.duration_s(); }

private:
    robonode::Otg& otg_;
    std::vector<std::pair<double, double>> schedule_;
    std::size_t i_{};
};

void test_executive_streams_retargeted_otg_through_governor() {
    robonode::SimAxis axis{"t", 0.0, 0.005};
    robonode::Governor gov{kLimits};
    robonode::Executive exec{axis, gov, 1000.0};
    robonode::Otg otg{1000.0, kLimits, 0.0};
    ScheduledRetargets stream{otg, {{0.0, 500.0}, {0.2, 200.0}, {0.4, 800.0}}};

    std::vector<robonode::TelemetryRow> rows;
    const auto stats = exec.execute(stream, rows, /*run_for_s=*/1.6);

    CHECK(stats.safety_hold_cycles == 0);
    // OTG output respects the same limits the governor enforces → silent.
    CHECK(gov.position_clamps() == 0);
    CHECK(gov.velocity_clamps() == 0);
    CHECK(std::abs(rows.back().actual_position - 800.0) < 0.5);
}

void test_sync_executive_two_axes_settle_together() {
    robonode::SimAxis rail{"rail-x", 0.0, 0.005};
    robonode::SimAxis aux{"turret-a", 0.0, 0.005};
    robonode::Governor g0{kLimits}, g1{kAuxLimits};
    robonode::SyncExecutive exec{{&rail, &aux}, {&g0, &g1}, 1000.0};

    const auto plan = robonode::SyncBlendPlan::plan({{0.0, 500.0, 300.0}, {0.0, 90.0, 45.0}},
                                                    {kLimits, kAuxLimits});
    std::vector<std::vector<robonode::TelemetryRow>> rows;
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.1);

    CHECK(rows.size() == 2);
    CHECK(rows[0].size() == stats.cycles);
    CHECK(stats.safety_hold_cycles == 0);
    // In-envelope plan: governors silent.
    CHECK(g0.position_clamps() == 0 && g0.velocity_clamps() == 0);
    CHECK(g1.position_clamps() == 0 && g1.velocity_clamps() == 0);
    CHECK(std::abs(rows[0].back().actual_position - 300.0) < 0.5);
    CHECK(std::abs(rows[1].back().actual_position - 45.0) < 0.5);
}

// Cell-coherent safety (FR-8.2): when ANY axis in a synchronized group goes
// non-NORMAL, EVERY axis holds — a group that keeps moving while one member
// is stopped is how gantries rack themselves. The property #3 requires for
// the 7-DOF arm, proven here generically at the motion level.
void test_sync_executive_coherent_hold() {
    robonode::SimAxis normal_axis{"a", 0.0, 0.005};
    FaultTimedAxis faulting{100, 300};  // trips protective stop, cycles [100,300)
    robonode::Governor g0{kLimits}, g1{kLimits};
    robonode::SyncExecutive exec{{&normal_axis, &faulting}, {&g0, &g1}, 1000.0};

    const auto plan = robonode::SyncBlendPlan::plan({{0.0, 500.0}, {0.0, 500.0}}, {kLimits, kLimits});
    std::vector<std::vector<robonode::TelemetryRow>> rows;
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.5);

    CHECK(stats.safety_hold_cycles == 200);  // whole trip window
    // The NORMAL axis freezes too during the fault window — coherence.
    for (std::size_t i = 101; i < 300; ++i) {
        CHECK(rows[0][i].governed_position == rows[0][100].governed_position);
    }
    // Both still complete after recovery.
    CHECK(std::abs(rows[0].back().actual_position - 500.0) < 0.5);
    CHECK(std::abs(rows[1].back().actual_position - 500.0) < 0.5);
}

void test_joint_planner_produces_reaching_waypoints() {
    robonode::JointPlanner planner;
    const std::vector<double> start{0.0, 0.0, 0.0};
    robonode::Goal goal;
    goal.kind = robonode::Goal::kJoint;
    goal.joint = {0.5, -0.3, 1.0};

    std::vector<std::vector<double>> wp;
    CHECK(planner.plan(start, goal, wp).ok());
    CHECK(wp.size() == 3);
    for (std::size_t k = 0; k < 3; ++k) {
        CHECK(wp[k].front() == start[k]);       // starts where we are
        CHECK(wp[k].back() == goal.joint[k]);   // ends at the goal
    }
    // Wrong goal kind fails closed.
    goal.kind = robonode::Goal::kCartesianPosition;
    CHECK(!planner.plan(start, goal, wp).ok());
}

// #54: a time-parameterized trajectory plays through the SAME executive/
// governor as everything else — a timed planner's output drives the axis to
// its final sample, no re-timing.
void test_timed_trajectory_drives_axis_through_executive() {
    robonode::TimedAxisSource src{{0.0, 0.5, 1.0}, {0.0, 250.0, 500.0}};  // rail 0→500 mm / 1 s
    robonode::SimAxis axis{"rail", 0.0, 0.005};
    robonode::Governor gov{kLimits};
    robonode::Executive exec{axis, gov, 1000.0};
    std::vector<robonode::TelemetryRow> rows;
    exec.execute(src, rows, /*run_for_s=*/1.3);
    CHECK(gov.position_clamps() == 0);  // in-envelope
    CHECK(std::abs(rows.back().actual_position - 500.0) < 1.0);
}

// The default plan_trajectory wraps bare waypoints (waypoint-form planners).
void test_planner_trajectory_wraps_waypoints() {
    robonode::JointPlanner planner;
    robonode::Goal goal;
    goal.kind = robonode::Goal::kJoint;
    goal.joint = {0.5, -0.3, 1.0};
    robonode::PlannedTrajectory traj;
    CHECK(planner.plan_trajectory({0.0, 0.0, 0.0}, goal, traj).ok());
    CHECK(!traj.is_timed());
    CHECK(traj.waypoints.size() == 3);
    CHECK(traj.waypoints[0].back() == 0.5);
}

}  // namespace

int main() {
    test_governor_holds_position_envelope();
    test_timed_trajectory_drives_axis_through_executive();
    test_planner_trajectory_wraps_waypoints();
    test_governor_rate_limits_jumps();
    test_governor_rejects_nonfinite_and_bad_dt();
    test_adapter_lifecycle_defaults();
    test_executive_completes_scurve_move();
    test_executive_holds_on_protective_stop_and_recovers();
    test_otg_reaches_target_jerk_limited();
    test_otg_retargets_mid_flight_smoothly();
    test_executive_streams_retargeted_otg_through_governor();
    test_sync_plan_passes_through_monotonic_via();
    test_sync_plan_reversal_corner_respects_limits();
    test_sync_executive_two_axes_settle_together();
    test_sync_executive_coherent_hold();
    test_joint_planner_produces_reaching_waypoints();
    std::puts("robonode motion: all tests passed");
    return 0;
}
