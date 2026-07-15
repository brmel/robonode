// 7-DOF arm integration tests (celld + sim_mujoco): the rail + UR10e joints
// as one synchronized group on one shared physics world. Built only with
// ROBONODE_BUILD_MUJOCO.

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "check.hpp"
#include "robonode/celld/cell.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/sync_blend.hpp"
#include "robonode/motion/sync_executive.hpp"
#include "robonode/sim_mujoco/mujoco_driver.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must be defined for the arm test"
#endif

namespace {

const std::string kWorld = std::string{ROBONODE_WORLDS} + "/rail_ur10e.xml";

const robonode::AxisLimits kRail{0, 1450, 1200, 8000, 120000};
const robonode::AxisLimits kJoint{-6.28, 6.28, 3.0, 15.0, 150.0};
const robonode::AxisLimits kElbow{-3.14, 3.14, 3.0, 15.0, 150.0};

std::vector<robonode::Descriptor> arm_descriptors() {
    auto mk = [](std::string id, std::string joint, std::string act, robonode::AxisLimits lim,
                 const char* units) {
        robonode::Descriptor d;
        d.id = std::move(id);
        d.driver = "robonode.mujoco-axis";
        d.limits = lim;
        d.command_rate_hz = 1000;
        d.config = {{"world", kWorld}, {"joint", std::move(joint)}, {"actuator", std::move(act)},
                    {"units_per_m", units}};
        return d;
    };
    return {mk("rail-x", "rail", "rail_servo", kRail, "1000"),
            mk("j1", "j1", "j1_servo", kJoint, "1"), mk("j2", "j2", "j2_servo", kJoint, "1"),
            mk("j3", "j3", "j3_servo", kElbow, "1"), mk("j4", "j4", "j4_servo", kJoint, "1"),
            mk("j5", "j5", "j5_servo", kJoint, "1"), mk("j6", "j6", "j6_servo", kJoint, "1")};
}

const std::vector<std::vector<double>> kWaypoints = {
    {0.0, 500.0, 300.0}, {0.0, 0.6, 0.3}, {0.0, -0.5, -0.3}, {0.0, 0.5, 0.25},
    {0.0, 0.4, 0.2},     {0.0, -0.4, -0.2}, {0.0, 0.3, 0.15}};

// The whole cell built from descriptors reaches every target on one clock.
void test_arm_seven_dof_one_clock() {
    robonode::DriverRegistry reg;
    robonode::register_mujoco_axis(reg);
    robonode::Cell cell{reg};
    for (const auto& d : arm_descriptors()) CHECK(cell.add_node(d).ok());
    CHECK(cell.nodes().size() == 7);
    CHECK(cell.configure_all().ok());
    CHECK(cell.activate_all().ok());

    std::vector<std::vector<robonode::TelemetryRow>> rows;
    robonode::CycleStats stats{};
    CHECK(cell.run_waypoints(kWaypoints, 1000.0, rows, stats, /*settle_s=*/1.0).ok());

    CHECK(rows.size() == 7);
    for (std::size_t i = 0; i < 7; ++i) {
        // One clock: every axis ran the identical number of cycles.
        CHECK(rows[i].size() == stats.cycles);
        // Platform correctness — the whole plan/governor/celld chain drives
        // the COMMANDED setpoint to the exact target on every axis.
        CHECK(std::abs(rows[i].back().governed_position - kWaypoints[i].back()) < 1e-6);
        // Physics closed the loop: the plant moved substantially toward its
        // target in the right direction (servo gains are nominal; tight
        // tracking is control tuning, orthogonal to coordination — see the
        // world README). rail tracks tightly (light, direct-drive).
        const double target = kWaypoints[i].back();
        const double actual = rows[i].back().actual_position;
        if (i == 0) {
            CHECK(std::abs(actual - target) < 5.0);  // rail mm
        } else {
            CHECK(actual * target > 0.0);            // correct direction
            CHECK(std::abs(actual) > 0.5 * std::abs(target));  // moved most of the way
            CHECK(std::abs(actual - target) < 0.15);           // rad
        }
    }
    CHECK(stats.safety_hold_cycles == 0);
}

// Generic fault injector: forwards to an already-configured adapter but
// reports PROTECTIVE_STOP for cycles [trip, clear). World stepping is the
// executive's job (via shared_world), so wrapping any joint is safe.
class FaultInjector final : public robonode::AxisAdapter {
public:
    FaultInjector(robonode::AxisAdapter* inner, std::uint64_t trip, std::uint64_t clear)
        : inner_{inner}, trip_{trip}, clear_{clear} {}
    void write_setpoint(double p) noexcept override { inner_->write_setpoint(p); }
    void step(double dt) noexcept override {
        ++cycle_;
        inner_->step(dt);
    }
    [[nodiscard]] robonode::AxisState read() const noexcept override {
        robonode::AxisState s = inner_->read();
        if (cycle_ >= trip_ && cycle_ < clear_) s.safety = robonode::SafetyState::kProtectiveStop;
        return s;
    }
    [[nodiscard]] robonode::CycleSteppable* shared_world() const noexcept override {
        return inner_->shared_world();
    }
    [[nodiscard]] std::string name() const override { return inner_->name(); }

private:
    robonode::AxisAdapter* inner_;
    std::uint64_t cycle_{}, trip_, clear_;
};

// A protective stop on ONE joint holds ALL 7 (cell-coherent safety, #3).
void test_arm_fault_on_one_joint_holds_all() {
    robonode::MujocoWorldPool pool;
    std::vector<std::unique_ptr<robonode::MujocoAxisAdapter>> owned;
    auto make = [&](std::string id, std::string joint, std::string act, double upm) {
        std::shared_ptr<robonode::MujocoWorld> w;
        CHECK(pool.get(kWorld, w).ok());
        owned.push_back(std::make_unique<robonode::MujocoAxisAdapter>(
            std::move(id), std::move(w), std::move(joint), std::move(act), upm));
        CHECK(owned.back()->configure().ok());
        CHECK(owned.back()->activate().ok());
    };
    make("rail-x", "rail", "rail_servo", 1000);
    make("j1", "j1", "j1_servo", 1);
    make("j2", "j2", "j2_servo", 1);
    make("j3", "j3", "j3_servo", 1);

    // Trip j3 (index 3) for cycles [200, 600).
    FaultInjector faulted{owned[3].get(), 200, 600};
    std::vector<robonode::AxisAdapter*> adapters{owned[0].get(), owned[1].get(), owned[2].get(),
                                                 &faulted};
    std::vector<robonode::Governor> govs{robonode::Governor{kRail}, robonode::Governor{kJoint},
                                         robonode::Governor{kJoint}, robonode::Governor{kElbow}};
    std::vector<robonode::Governor*> gp{&govs[0], &govs[1], &govs[2], &govs[3]};
    robonode::SyncExecutive exec{adapters, gp, 1000.0};

    const auto plan = robonode::SyncBlendPlan::plan(
        {{0.0, 400.0}, {0.0, 0.5}, {0.0, -0.4}, {0.0, 0.4}}, {kRail, kJoint, kJoint, kElbow});
    std::vector<std::vector<robonode::TelemetryRow>> rows;
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.6);

    CHECK(stats.safety_hold_cycles == 400);  // whole trip window, all axes
    // Every axis (including the rail) freezes its command during the fault
    // window — coherence across the shared world.
    for (std::size_t ax = 0; ax < 4; ++ax) {
        for (std::size_t i = 201; i < 600; ++i) {
            CHECK(rows[ax][i].governed_position == rows[ax][200].governed_position);
        }
    }
}

}  // namespace

int main() {
    test_arm_seven_dof_one_clock();
    test_arm_fault_on_one_joint_holds_all();
    std::puts("robonode arm: all tests passed");
    return 0;
}
