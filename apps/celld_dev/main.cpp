// celld_dev — the coordination plane in miniature: descriptors from
// robonode-idl/examples are the ONLY source of node identity and limits
// (FR-1.3); celld builds the tree, drives lifecycle (FR-1.2), runs a
// cell-coherent blended program (FR-2.3/2.4), records MCAP, tears down.
//
//   ./celld_dev [descriptor.json ...]      default: rail-x + turret-a examples

#include <cstdio>
#include <string>
#include <vector>

#include "robonode/celld/cell.hpp"
#include "robonode/motion/sim_driver.hpp"
#include "robonode/recorder/mcap_recorder.hpp"

namespace {
int die(const robonode::Status& st) {
    std::fprintf(stderr, "celld: %s\n", st.message().c_str());
    return 1;
}
}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> paths;
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) paths.emplace_back(argv[i]);
    } else {
        paths = {"robonode-idl/examples/rail-x.descriptor.json",
                 "robonode-idl/examples/turret-a.descriptor.json"};
    }

    // Apps own the driver registry — they register exactly the drivers they
    // link. celld_dev links only sim; a hardware app would also
    // register_ur_wrist(registry, ...). celld itself stays vendor-blind.
    robonode::DriverRegistry registry;
    robonode::register_sim_axis(registry);

    robonode::Cell cell{registry};
    double rate_hz = 1000.0;
    for (const auto& p : paths) {
        robonode::Descriptor d;
        if (const auto st = robonode::load_descriptor(p, d); !st.ok()) return die(st);
        if (const auto st = cell.add_node(d); !st.ok()) return die(st);
        rate_hz = d.command_rate_hz;  // v0: one clock, last descriptor wins
        std::printf("node %-10s driver=%s pos[%g, %g] vel %g acc %g jerk %g\n", d.id.c_str(),
                    d.driver.c_str(), d.limits.position_min, d.limits.position_max,
                    d.limits.velocity_max, d.limits.acceleration_max,
                    d.limits.jerk_max);
    }

    if (const auto st = cell.configure_all(); !st.ok()) return die(st);
    if (const auto st = cell.activate_all(); !st.ok()) return die(st);
    std::printf("cell up: %zu nodes active\n", cell.nodes().size());

    // Program: synchronized blended waypoints, one list per node, inside
    // each node's own descriptor envelope.
    std::vector<std::vector<double>> waypoints;
    std::vector<std::string> topics;
    for (const auto& n : cell.nodes()) {
        const auto& l = n.descriptor.limits;
        const double lo = l.position_min;
        const double span = l.position_max - l.position_min;
        waypoints.push_back({lo, lo + 0.6 * span, lo + 0.4 * span});
        topics.push_back("rn/dev-cell/" + n.id + "/MotionAxis/telemetry");
    }

    std::vector<std::vector<robonode::TelemetryRow>> rows;
    robonode::CycleStats stats{};
    if (const auto st = cell.run_waypoints(waypoints, rate_hz, rows, stats); !st.ok()) {
        return die(st);
    }
    for (std::size_t i = 0; i < cell.nodes().size(); ++i) {
        std::printf("%-10s end %.3f (target %.3f) — %llu cycles, one clock\n",
                    cell.nodes()[i].id.c_str(), rows[i].back().actual_position,
                    waypoints[i].back(), static_cast<unsigned long long>(stats.cycles));
    }
    std::printf("jitter mean %.1f us | p99 %.1f | max %.1f | safety holds %llu\n",
                stats.mean_jitter_us, stats.p99_jitter_us, stats.max_jitter_us,
                static_cast<unsigned long long>(stats.safety_hold_cycles));

    const auto rec = robonode::McapRecorder::write("celld-run.mcap", topics, rows, rate_hz);
    std::printf("recording: %s\n", rec.ok() ? "celld-run.mcap" : rec.message().c_str());

    if (const auto st = cell.deactivate_all(); !st.ok()) return die(st);
    std::printf("cell down\n");
    return 0;
}
