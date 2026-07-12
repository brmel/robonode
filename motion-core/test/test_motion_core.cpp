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
#include "robonode/motion_plan.hpp"
#include "robonode/sim_axis.hpp"

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

}  // namespace

int main() {
    test_governor_holds_position_envelope();
    test_governor_rate_limits_jumps();
    test_governor_rejects_nonfinite_and_bad_dt();
    test_executive_completes_scurve_move();
    test_executive_holds_on_protective_stop_and_recovers();
    std::puts("motion-core: all tests passed");
    return 0;
}
