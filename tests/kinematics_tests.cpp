// #4 Cartesian layer tests: FK/IK/moveL on the UR10e arm. Pure kinematics
// on the MuJoCo model (no physics servo), so exact and robust. Built with
// ROBONODE_BUILD_MUJOCO.

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "check.hpp"
#include "robonode/motion/cartesian.hpp"
#include "robonode/sim_mujoco/mujoco_kinematics.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must be defined for the kinematics test"
#endif

namespace {

std::unique_ptr<robonode::MujocoKinematics> make_kin() {
    std::unique_ptr<robonode::MujocoKinematics> k;
    const auto st = robonode::MujocoKinematics::create(
        std::string{ROBONODE_WORLDS} + "/rail_ur10e.xml", {"j1", "j2", "j3", "j4", "j5", "j6"},
        "tcp", k);
    CHECK(st.ok());
    return k;
}

void test_fk_responds_to_joints() {
    auto kin = make_kin();
    CHECK(kin->dof() == 6);
    const std::vector<double> zero(6, 0.0);
    const robonode::Vec3 p0 = kin->tcp_position(zero);
    // Moving j2 must move the TCP (the model is actually articulated).
    std::vector<double> q = zero;
    q[1] = 0.5;
    const robonode::Vec3 p1 = kin->tcp_position(q);
    CHECK((p1 - p0).norm() > 0.05);
}

void test_jacobian_matches_finite_difference() {
    auto kin = make_kin();
    const std::vector<double> q{0.2, -0.3, 0.4, 0.1, -0.2, 0.3};
    const auto J = kin->position_jacobian(q);  // 3×6
    const double h = 1e-6;
    for (std::size_t k = 0; k < 6; ++k) {
        std::vector<double> qp = q, qm = q;
        qp[k] += h;
        qm[k] -= h;
        const robonode::Vec3 dp = (kin->tcp_position(qp) - kin->tcp_position(qm)) * (0.5 / h);
        CHECK(std::abs(dp.x - J[0 * 6 + k]) < 1e-4);
        CHECK(std::abs(dp.y - J[1 * 6 + k]) < 1e-4);
        CHECK(std::abs(dp.z - J[2 * 6 + k]) < 1e-4);
    }
}

void test_ik_reaches_reachable_target() {
    auto kin = make_kin();
    // Target = FK of a known pose ⇒ definitely reachable. IK from a different
    // seed must drive the TCP to it (redundant arm: joints may differ, TCP
    // matches — the correct IK property).
    const std::vector<double> q_target{0.3, -0.5, 0.6, 0.2, -0.3, 0.1};
    const robonode::Vec3 target = kin->tcp_position(q_target);

    std::vector<double> q_sol;
    const auto st = robonode::ik_position(*kin, std::vector<double>(6, 0.0), target, q_sol);
    CHECK(st.ok());
    CHECK((kin->tcp_position(q_sol) - target).norm() < 1e-3);
}

void test_ik_reports_unreachable() {
    auto kin = make_kin();
    // Far outside the ~1.3 m reach.
    std::vector<double> q_sol;
    const auto st = robonode::ik_position(*kin, std::vector<double>(6, 0.0),
                                          robonode::Vec3{10.0, 10.0, 10.0}, q_sol);
    CHECK(!st.ok());
}

void test_move_l_traces_straight_line() {
    auto kin = make_kin();
    const std::vector<double> q0{0.1, -0.4, 0.5, 0.0, -0.2, 0.0};
    const robonode::Vec3 p0 = kin->tcp_position(q0);
    const robonode::Vec3 target = p0 + robonode::Vec3{0.12, -0.08, 0.05};  // reachable delta

    std::vector<std::vector<double>> wp;  // [joint][step]
    const auto st = robonode::plan_move_l(*kin, q0, target, 20, wp);
    CHECK(st.ok());
    CHECK(wp.size() == 6);
    CHECK(wp[0].size() == 21);

    // Endpoint reaches target; every intermediate TCP lies near the p0→target
    // line (straight Cartesian path, the point of moveL).
    const robonode::Vec3 dir = target - p0;
    const double len = dir.norm();
    for (std::size_t s = 0; s < wp[0].size(); ++s) {
        std::vector<double> q(6);
        for (std::size_t k = 0; k < 6; ++k) q[k] = wp[k][s];
        const robonode::Vec3 p = kin->tcp_position(q);
        // Perpendicular distance from the line through p0 with direction dir.
        const robonode::Vec3 rel = p - p0;
        const double t = (rel.x * dir.x + rel.y * dir.y + rel.z * dir.z) / (len * len);
        const robonode::Vec3 proj = p0 + dir * t;
        CHECK((p - proj).norm() < 2e-3);  // within 2 mm of the straight line
    }
    std::vector<double> q_end(6);
    for (std::size_t k = 0; k < 6; ++k) q_end[k] = wp[k].back();
    CHECK((kin->tcp_position(q_end) - target).norm() < 1e-3);
}

// Cartesian jog: the joint velocities it returns must reproduce (near) the
// requested TCP velocity when pushed back through the Jacobian.
void test_cartesian_jog_produces_requested_tcp_velocity() {
    auto kin = make_kin();
    const std::vector<double> q{0.2, -0.4, 0.5, 0.1, -0.2, 0.1};
    const robonode::Vec3 v_req{0.1, -0.05, 0.08};  // m/s away from singularity
    std::vector<double> dq;
    CHECK(robonode::cartesian_jog_velocity(*kin, q, v_req, dq).ok());
    CHECK(dq.size() == 6);

    // v_achieved = J · dq
    const auto J = kin->position_jacobian(q);
    robonode::Vec3 v{};
    for (std::size_t k = 0; k < 6; ++k) {
        v.x += J[0 * 6 + k] * dq[k];
        v.y += J[1 * 6 + k] * dq[k];
        v.z += J[2 * 6 + k] * dq[k];
    }
    CHECK((v - v_req).norm() < 0.02);  // DLS tracks the request (small damping bias)
}

}  // namespace

int main() {
    test_fk_responds_to_joints();
    test_jacobian_matches_finite_difference();
    test_ik_reaches_reachable_target();
    test_ik_reports_unreachable();
    test_move_l_traces_straight_line();
    test_cartesian_jog_produces_requested_tcp_velocity();
    std::puts("robonode kinematics: all tests passed");
    return 0;
}
