// arm_dev — a 7-DOF cell (UR10e-parameterised arm on a rail) built by celld
// from descriptors, run as one synchronized blended program on one clock,
// in MuJoCo physics. The rail + 6 joints share a single world (pool), the
// rail is the clock owner. This is the arm+rail coordination story
// (differentiator D2) in physics.
//
//   ./arm_dev
//
// Writes arm-run.mcap (7 topics, open in Foxglove).

#include <cstdio>
#include <string>
#include <vector>

#include "robonode/celld/cell.hpp"
#include "robonode/motion/sim_driver.hpp"
#include "robonode/recorder/mcap_recorder.hpp"
#include "robonode/sim_mujoco/mujoco_driver.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must point at the sim-mujoco worlds dir"
#endif

namespace {

// Builds the 7 node descriptors of the rail+UR10e cell. Limits are data
// (FR-1.3); rail in mm, joints in rad. All reference one shared world path.
std::vector<robonode::Descriptor> arm_descriptors() {
    const std::string world = std::string{ROBONODE_WORLDS} + "/rail_ur10e.xml";
    auto mk = [&](std::string id, std::string joint, std::string act, robonode::AxisLimits lim,
                  const char* units) {
        robonode::Descriptor d;
        d.id = std::move(id);
        d.driver = "robonode.mujoco-axis";
        d.limits = lim;
        d.command_rate_hz = 1000;
        d.config = {{"world", world}, {"joint", std::move(joint)}, {"actuator", std::move(act)},
                    {"units_per_m", units}};
        return d;
    };
    const robonode::AxisLimits rail{0, 1450, 1200, 8000, 120000};   // mm
    const robonode::AxisLimits jl{-6.28, 6.28, 3.0, 15.0, 150.0};   // rad
    const robonode::AxisLimits j3l{-3.14, 3.14, 3.0, 15.0, 150.0};  // rad (elbow)
    return {mk("rail-x", "rail", "rail_servo", rail, "1000"),
            mk("j1", "j1", "j1_servo", jl, "1"),
            mk("j2", "j2", "j2_servo", jl, "1"),
            mk("j3", "j3", "j3_servo", j3l, "1"),
            mk("j4", "j4", "j4_servo", jl, "1"),
            mk("j5", "j5", "j5_servo", jl, "1"),
            mk("j6", "j6", "j6_servo", jl, "1")};
}

int die(const robonode::Status& st) {
    std::fprintf(stderr, "arm_dev: %s\n", st.message().c_str());
    return 1;
}

}  // namespace

int main() {
    robonode::DriverRegistry registry;
    robonode::register_sim_axis(registry);
    robonode::register_mujoco_axis(registry);  // nodes share the world via the pool

    robonode::Cell cell{registry};
    for (const auto& d : arm_descriptors()) {
        if (const auto st = cell.add_node(d); !st.ok()) return die(st);
    }
    if (const auto st = cell.configure_all(); !st.ok()) return die(st);
    if (const auto st = cell.activate_all(); !st.ok()) return die(st);
    std::printf("arm cell up: %zu nodes on one shared world\n", cell.nodes().size());

    // Coordinated blended program: rail traverses while the arm articulates,
    // interior waypoints passed at speed, all on one clock.
    const std::vector<std::vector<double>> waypoints = {
        {0.0, 500.0, 300.0},   // rail-x mm
        {0.0, 0.6, 0.3},       // j1 rad
        {0.0, -0.5, -0.3},     // j2
        {0.0, 0.5, 0.25},      // j3
        {0.0, 0.4, 0.2},       // j4
        {0.0, -0.4, -0.2},     // j5
        {0.0, 0.3, 0.15},      // j6
    };

    std::vector<std::vector<robonode::TelemetryRow>> rows;
    robonode::CycleStats stats{};
    if (const auto st = cell.run_waypoints(waypoints, 1000.0, rows, stats, /*settle_s=*/2.0);
        !st.ok()) {
        return die(st);
    }

    std::printf("7-DOF coordinated move, %llu cycles on one clock, safety holds %llu:\n",
                static_cast<unsigned long long>(stats.cycles),
                static_cast<unsigned long long>(stats.safety_hold_cycles));
    std::vector<std::string> topics;
    for (std::size_t i = 0; i < cell.nodes().size(); ++i) {
        const auto& n = cell.nodes()[i];
        std::printf("  %-7s actual %.4f | governed %.4f | target %.4f\n", n.id.c_str(),
                    rows[i].back().actual_position_mm, rows[i].back().governed_position_mm,
                    waypoints[i].back());
        topics.push_back("rn/dev-cell/" + n.id + "/MotionAxis/telemetry");
    }

    const auto rec = robonode::McapRecorder::write("arm-run.mcap", topics, rows, 1000.0);
    std::printf("recording: %s\n", rec.ok() ? "arm-run.mcap" : rec.message().c_str());
    (void)cell.deactivate_all();
    return 0;
}
