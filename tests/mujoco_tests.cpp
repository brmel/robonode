// robonode::sim_mujoco module tests — physics twin behind the AxisAdapter
// seam. Built only when ROBONODE_BUILD_MUJOCO is on (heavy MuJoCo build).

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "axis_run.hpp"
#include "check.hpp"
#include "robonode/motion/driver_registry.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/motion_plan.hpp"
#include "robonode/sim_mujoco/mujoco_driver.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must be defined for the mujoco test"
#endif

namespace {

constexpr robonode::AxisLimits kRail{
    .position_min = 0.0,
    .position_max = 1450.0,
    .velocity_max = 1200.0,
    .acceleration_max = 8000.0,
    .jerk_max = 120000.0,
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
    const auto plan = robonode::MotionPlan::move(
        0.0, 500.0, {kRail.velocity_max, kRail.acceleration_max, kRail.jerk_max});
    std::vector<robonode::TelemetryRow> rows;
    testing::run_plan(*rail, gov, 1000.0, plan, rows, /*settle_s=*/0.5);

    // Physics servo tracks the setpoint to steady state (not exact — that is
    // the point of a real plant): within 5 mm of 500 after settling.
    CHECK(std::abs(rows.back().actual_position - 500.0) < 5.0);
    // In-envelope plan ⇒ governor silent (the OTG/plan respected the limits).
    CHECK(gov.position_clamps() == 0);
    CHECK(gov.velocity_clamps() == 0);
    // Following error is physical: nonzero at some point during the move.
    double peak = 0.0;
    for (const auto& r : rows) peak = std::max(peak, std::abs(r.following_error));
    CHECK(peak > 0.1);
}


// The model describes the robot once: links, joint axes/types, meshes and the
// named tool site. Serving this is what lets the viewer stop transcribing it.
void test_world_exposes_its_kinematic_chain() {
    std::shared_ptr<robonode::MujocoWorld> world;
    CHECK(robonode::MujocoWorld::load(std::string{ROBONODE_WORLDS} + "/rail_ur10e.xml", world).ok());

    const auto links = world->links();
    CHECK(links.size() > 2);
    CHECK(links.front().parent == 0);  // the world body roots the tree

    bool saw_slide = false, saw_hinge = false, saw_mesh = false;
    for (const auto& l : links) {
        if (l.joint_type == "slide") saw_slide = true;
        if (l.joint_type == "hinge") saw_hinge = true;
        for (const auto& g : l.geoms) saw_mesh = saw_mesh || g.type == "mesh";
        if (!l.joint.empty()) {  // a driven link declares a real axis
            const double n2 = l.axis[0] * l.axis[0] + l.axis[1] * l.axis[1] + l.axis[2] * l.axis[2];
            CHECK(n2 > 0.5);
        }
        CHECK(l.parent >= 0 && static_cast<std::size_t>(l.parent) < links.size());
    }
    CHECK(saw_slide && saw_hinge && saw_mesh);  // rail, arm joints, visual meshes

    bool saw_tcp = false;
    for (const auto& s : world->sites()) {
        if (s.name != "tcp") continue;
        saw_tcp = true;
        CHECK(s.body > 0 && static_cast<std::size_t>(s.body) < links.size());
    }
    CHECK(saw_tcp);
}

}  // namespace

int main() {
    test_mujoco_world_loads_and_registers();
    test_mujoco_bad_config_fails_closed();
    test_mujoco_move_tracks_target_under_physics();
    test_world_exposes_its_kinematic_chain();
    std::puts("robonode sim_mujoco: all tests passed");
    return 0;
}
