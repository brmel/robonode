// robonode::sim_mujoco module tests — physics twin behind the AxisAdapter
// seam. Built only when ROBONODE_BUILD_MUJOCO is on (heavy MuJoCo build).

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "check.hpp"
#include "robonode/motion/driver_registry.hpp"
#include "robonode/motion/executive.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/motion_plan.hpp"
#include "robonode/sim_mujoco/mujoco_driver.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must be defined for the mujoco test"
#endif

namespace {

constexpr robonode::AxisLimits kRail{
    .position_min_mm = 0.0,
    .position_max_mm = 1450.0,
    .velocity_max_mm_s = 1200.0,
    .acceleration_max_mm_s2 = 8000.0,
    .jerk_max_mm_s3 = 120000.0,
};

robonode::DriverContext rail_ctx() {
    return {"rail-x",
            kRail,
            {{"world", std::string{ROBONODE_WORLDS} + "/rail.xml"},
             {"joint", "rail"},
             {"actuator", "rail_servo"},
             {"units_per_m", "1000"}}};
}

void test_mujoco_world_loads_and_registers() {
    robonode::DriverRegistry reg;
    robonode::register_mujoco_axis(reg);
    CHECK(reg.has("robonode.mujoco-axis"));
    std::unique_ptr<robonode::AxisAdapter> rail;
    CHECK(reg.make("robonode.mujoco-axis", rail_ctx(), rail).ok());
    CHECK(rail != nullptr);
    CHECK(rail->configure().ok());
    CHECK(rail->lifecycle() == robonode::Lifecycle::kInactive);
}

void test_mujoco_bad_config_fails_closed() {
    robonode::DriverRegistry reg;
    robonode::register_mujoco_axis(reg);
    std::unique_ptr<robonode::AxisAdapter> a;
    // Missing world/joint/actuator ⇒ factory returns null ⇒ make fails.
    CHECK(!reg.make("robonode.mujoco-axis", {"x", kRail, {}}, a).ok());
    // Bad world path ⇒ load fails ⇒ null.
    CHECK(!reg.make("robonode.mujoco-axis",
                    {"x", kRail, {{"world", "/no/such.xml"}, {"joint", "rail"}, {"actuator", "rail_servo"}}},
                    a)
               .ok());
}

void test_mujoco_move_tracks_target_under_physics() {
    robonode::DriverRegistry reg;
    robonode::register_mujoco_axis(reg);
    std::unique_ptr<robonode::AxisAdapter> rail;
    CHECK(reg.make("robonode.mujoco-axis", rail_ctx(), rail).ok());
    CHECK(rail->configure().ok());
    CHECK(rail->activate().ok());

    robonode::Governor gov{kRail};
    robonode::Executive exec{*rail, gov, 1000.0};
    const auto plan = robonode::MotionPlan::move(
        0.0, 500.0, {kRail.velocity_max_mm_s, kRail.acceleration_max_mm_s2, kRail.jerk_max_mm_s3});
    std::vector<robonode::TelemetryRow> rows;
    exec.execute(plan, rows, /*settle_s=*/0.5);

    // Physics servo tracks the setpoint to steady state (not exact — that is
    // the point of a real plant): within 5 mm of 500 after settling.
    CHECK(std::abs(rows.back().actual_position_mm - 500.0) < 5.0);
    // In-envelope plan ⇒ governor silent (the OTG/plan respected the limits).
    CHECK(gov.position_clamps() == 0);
    CHECK(gov.velocity_clamps() == 0);
    // Following error is physical: nonzero at some point during the move.
    double peak = 0.0;
    for (const auto& r : rows) peak = std::max(peak, std::abs(r.following_error_mm));
    CHECK(peak > 0.1);
}

}  // namespace

int main() {
    test_mujoco_world_loads_and_registers();
    test_mujoco_bad_config_fails_closed();
    test_mujoco_move_tracks_target_under_physics();
    std::puts("robonode sim_mujoco: all tests passed");
    return 0;
}
