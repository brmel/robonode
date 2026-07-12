// M0 smoke tests — no framework dependency yet. Grows into Catch2 (same as
// trajectory-lab) when the sim-gate suites land (FR-3.3).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

// assert() vanishes under NDEBUG (Release); tests must fail in every build type.
#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::fprintf(stderr, "CHECK failed %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                                 \
            std::abort();                                                        \
        }                                                                        \
    } while (0)

#include "robonode/executive.hpp"
#include "robonode/governor.hpp"
#include "robonode/mcap_recorder.hpp"
#include "robonode/motion_plan.hpp"
#include "robonode/sim_axis.hpp"
#include "robonode/sync_blend.hpp"
#include "robonode/sync_executive.hpp"

namespace {

constexpr robonode::AxisLimits kLimits{
    .position_min_mm = 0.0,
    .position_max_mm = 1450.0,
    .velocity_max_mm_s = 1200.0,
    .acceleration_max_mm_s2 = 8000.0,
    .jerk_max_mm_s3 = 120000.0,
};

void test_governor_holds_position_envelope() {
    robonode::Governor gov{kLimits};
    gov.reset(1449.0);
    // Command far outside; governed output must never exceed the limit and
    // must approach it at <= vel_max.
    double p = 1449.0;
    for (int i = 0; i < 100; ++i) {
        const auto r = gov.apply(2000.0, 1e-3);
        CHECK(r.position_mm <= kLimits.position_max_mm + 1e-9);
        CHECK(r.position_mm - p <= kLimits.velocity_max_mm_s * 1e-3 + 1e-9);
        p = r.position_mm;
    }
    CHECK(gov.position_clamps() > 0);
    CHECK(std::abs(p - kLimits.position_max_mm) < 1e-6);
}

void test_governor_rate_limits_jumps() {
    robonode::Governor gov{kLimits};
    gov.reset(0.0);
    const auto r = gov.apply(100.0, 1e-3);  // 100 mm jump in 1 ms = 100 m/s
    CHECK(std::abs(r.position_mm - kLimits.velocity_max_mm_s * 1e-3) < 1e-6);
    CHECK(gov.velocity_clamps() == 1);
}

void test_governor_rejects_nonfinite_and_bad_dt() {
    robonode::Governor gov{kLimits};
    gov.reset(100.0);
    const auto nan = gov.apply(std::nan(""), 1e-3);
    CHECK(nan.clamped);
    CHECK(nan.position_mm == 100.0);  // holds previous, never propagates NaN
    const auto inf = gov.apply(std::numeric_limits<double>::infinity(), 1e-3);
    CHECK(inf.position_mm == 100.0);
    const auto bad_dt = gov.apply(200.0, 0.0);
    CHECK(bad_dt.position_mm == 100.0);
    CHECK(gov.rejected_setpoints() == 3);
    // Recovers: next finite setpoint governed normally.
    const auto ok = gov.apply(100.5, 1e-3);
    CHECK(std::abs(ok.position_mm - 100.5) < 1e-9);
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
    const auto plan = robonode::MotionPlan::scurve(
        0.0, 500.0, {kLimits.velocity_max_mm_s, kLimits.acceleration_max_mm_s2, kLimits.jerk_max_mm_s3});
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.5);

    CHECK(stats.safety_hold_cycles == 200);
    // Command frozen across the whole hold window.
    for (std::size_t i = 101; i < 300; ++i) {
        CHECK(rows[i].governed_position_mm == rows[100].governed_position_mm);
    }
    // Catch-up after recovery stays inside the velocity envelope...
    for (std::size_t i = 300; i < rows.size(); ++i) {
        const double dp = rows[i].governed_position_mm - rows[i - 1].governed_position_mm;
        CHECK(std::abs(dp) <= kLimits.velocity_max_mm_s * 1e-3 + 1e-6);
    }
    // ...and the move still completes.
    CHECK(std::abs(rows.back().actual_position_mm - 500.0) < 0.5);
}

void test_executive_completes_scurve_move() {
    robonode::SimAxis axis{"t", 0.0, 0.005};
    robonode::Governor gov{kLimits};
    robonode::Executive exec{axis, gov, 1000.0};
    std::vector<robonode::TelemetryRow> rows;
    const auto plan = robonode::MotionPlan::scurve(
        0.0, 500.0, {kLimits.velocity_max_mm_s, kLimits.acceleration_max_mm_s2, kLimits.jerk_max_mm_s3});
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.1);

    CHECK(stats.cycles == rows.size());
    // In-envelope plan: governor must not have intervened.
    CHECK(gov.position_clamps() == 0);
    CHECK(gov.velocity_clamps() == 0);
    // Axis settled on target within 0.5 mm.
    CHECK(std::abs(rows.back().actual_position_mm - 500.0) < 0.5);
    // Governed setpoints respect the velocity envelope cycle-to-cycle.
    for (std::size_t i = 1; i < rows.size(); ++i) {
        const double dp = rows[i].governed_position_mm - rows[i - 1].governed_position_mm;
        CHECK(std::abs(dp) <= kLimits.velocity_max_mm_s * 1e-3 + 1e-6);
    }
}

constexpr robonode::AxisLimits kAuxLimits{
    .position_min_mm = -10.0,
    .position_max_mm = 100.0,
    .velocity_max_mm_s = 300.0,
    .acceleration_max_mm_s2 = 2000.0,
    .jerk_max_mm_s3 = 0.0,
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
            CHECK(std::abs(s.velocity) <= lims[i].velocity_max_mm_s * 1.001);
            CHECK(std::abs(s.acceleration) <= lims[i].acceleration_max_mm_s2 * 1.001);
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
        CHECK(std::abs(v_at_best[i]) > 0.1 * lims[i].velocity_max_mm_s);  // moving
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
            CHECK(std::abs(s.velocity) <= lims[i].velocity_max_mm_s * 1.001);
            CHECK(std::abs(s.acceleration) <= lims[i].acceleration_max_mm_s2 * 1.001);
        }
    }
    for (std::size_t i = 0; i < 2; ++i) {
        const auto end = plan.sample(i, T);
        CHECK(std::abs(end.position - wp[i][2]) < 1e-6);
        CHECK(std::abs(end.velocity) < 1e-9);
    }
}

void test_mcap_recorder_writes_valid_file() {
    std::vector<std::vector<robonode::TelemetryRow>> rows{
        {{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, {0.001, 1.1, 2.1, 3.1, 4.1, 5.1, 6.1}}};
    CHECK(robonode::McapRecorder::write("test-recorder.mcap",
                                        {"rn/test/axis/MotionAxis/telemetry"}, rows, 1000.0));
    std::FILE* f = std::fopen("test-recorder.mcap", "rb");
    CHECK(f != nullptr);
    char magic[8]{};
    CHECK(std::fread(magic, 1, 8, f) == 8);
    std::fclose(f);
    std::remove("test-recorder.mcap");
    // MCAP magic: \x89 M C A P 0 \r \n
    CHECK(magic[1] == 'M' && magic[2] == 'C' && magic[3] == 'A' && magic[4] == 'P');
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
    CHECK(std::abs(rows[0].back().actual_position_mm - 300.0) < 0.5);
    CHECK(std::abs(rows[1].back().actual_position_mm - 45.0) < 0.5);
}

}  // namespace

int main() {
    test_governor_holds_position_envelope();
    test_governor_rate_limits_jumps();
    test_governor_rejects_nonfinite_and_bad_dt();
    test_executive_completes_scurve_move();
    test_executive_holds_on_protective_stop_and_recovers();
    test_sync_plan_passes_through_monotonic_via();
    test_sync_plan_reversal_corner_respects_limits();
    test_sync_executive_two_axes_settle_together();
    test_mcap_recorder_writes_valid_file();
    std::puts("motion-core: all tests passed");
    return 0;
}
