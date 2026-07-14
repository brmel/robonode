// rtb_dev — real robot kinematics/planning behind our seams. Plans a straight
// Cartesian moveL on a REAL UR10 (Robotics Toolbox, via the rtb-kinematics
// service) and prints the resulting joint waypoints — the same
// std::vector<std::vector<double>> that SyncBlendPlan/celld consume. No
// hand-rolled kinematics: the math + model come from a mature library.
//
//   1) python3 services/rtb-kinematics/service.py        (needs RTB installed)
//   2) ./rtb_dev [robot=ur10] [host=127.0.0.1] [port=8091]

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "robonode/rtb/rtb_kinematics.hpp"
#include "robonode/rtb/rtb_planner.hpp"

int main(int argc, char** argv) {
    const std::string robot = argc > 1 ? argv[1] : "ur10";
    const std::string host = argc > 2 ? argv[2] : "127.0.0.1";
    const int port = argc > 3 ? std::atoi(argv[3]) : 8091;

    robonode::RtbKinematics kin{robot, 6, host, port};
    robonode::RtbPlanner planner{robot, /*steps=*/25, host, port};

    // Start pose → a reachable Cartesian target (start TCP + a small delta).
    const std::vector<double> q0{0.1, -0.9, 1.2, -0.3, 1.4, 0.0};
    robonode::Vec3 start_tcp;
    try {
        start_tcp = kin.tcp_position(q0);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "cannot reach rtb service (%s) — is service.py running?\n", e.what());
        return 1;
    }
    std::printf("real %s: start TCP = [%.4f %.4f %.4f]\n", robot.c_str(), start_tcp.x, start_tcp.y,
                start_tcp.z);

    robonode::Goal goal;
    goal.kind = robonode::Goal::kCartesianPosition;
    goal.cartesian = start_tcp + robonode::Vec3{0.10, -0.06, 0.05};

    std::vector<std::vector<double>> wp;  // [joint][step]
    if (const auto st = planner.plan(q0, goal, wp); !st.ok()) {
        std::fprintf(stderr, "plan failed: %s\n", st.message().c_str());
        return 1;
    }
    std::printf("planned real moveL to [%.4f %.4f %.4f]: %zu joints x %zu steps\n", goal.cartesian.x,
                goal.cartesian.y, goal.cartesian.z, wp.size(), wp[0].size());

    // Final joint waypoint, and its FK back through the real model — proves
    // the plan lands on the Cartesian target.
    std::vector<double> q_end(wp.size());
    for (std::size_t k = 0; k < wp.size(); ++k) q_end[k] = wp[k].back();
    const auto reached = kin.tcp_position(q_end);
    const double err = (reached - goal.cartesian).norm();
    std::printf("final joints:");
    for (double v : q_end) std::printf(" %.3f", v);
    std::printf("\nFK(final) = [%.4f %.4f %.4f]  |error| = %.5f m\n", reached.x, reached.y,
                reached.z, err);
    std::printf("%s\n", err < 1e-3 ? "OK — real-robot moveL landed on target" : "WARN — large error");
    return err < 1e-3 ? 0 : 1;
}
