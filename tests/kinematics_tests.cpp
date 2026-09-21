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
#include "robonode/motion/planner.hpp"
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
    // "did not converge" is not a diagnosis: the message must say how far short
    // the solver got, so an operator can tell "out of reach" from "just short".
    CHECK(st.message().find("mm short") != std::string::npos);
    CHECK(st.message().find("iterations") != std::string::npos);
}

// A planner handed the wrong kind of goal must say WHICH kind it got: 6-DoF
// pose and joint angles are different problems, and one of them has no planner.
void test_planner_names_the_goal_it_was_handed() {
    auto kin = make_kin();
    robonode::JointReachPlanner planner{*kin};
    robonode::Goal pose;
    pose.kind = robonode::Goal::kCartesianPose;
    std::vector<std::vector<double>> wp;
    const auto st = planner.plan(std::vector<double>(6, 0.0), pose, wp);
    CHECK(!st.ok());
    CHECK(st.message().find("Cartesian pose") != std::string::npos);
    CHECK(st.message().find("moveP") != std::string::npos);  // and what to select instead
}

// #39: a planner that can ask "would this be touching something?" can route
// around it. The query itself comes first — a planner is only as honest as it.
void test_kinematics_reports_a_collision() {
    auto kin = make_kin();
    CHECK(kin->can_check_collision());
    // The ready pose is clear; folding the arm into the rail is not.
    CHECK(!kin->in_collision(std::vector<double>{0.0, -1.5708, -1.5708, 1.5708, -1.5708, -1.5708}));
    // Elbow folded back into the arm's own structure: a configuration the
    // solver will happily produce and the machine cannot hold.
    CHECK(kin->in_collision(std::vector<double>{0.0, -0.4, 1.8, 0.0, 0.0, 0.0}));
}

// A planner handed kinematics that cannot answer must refuse rather than plan a
// straight line and call it avoidance.
void test_avoiding_planner_refuses_blind_kinematics() {
    struct BlindKinematics final : robonode::Kinematics {
        [[nodiscard]] std::size_t dof() const override { return 6; }
        [[nodiscard]] robonode::Vec3 tcp_position(const std::vector<double>&) const override {
            return {};
        }
        [[nodiscard]] std::vector<double> position_jacobian(
            const std::vector<double>&) const override {
            return std::vector<double>(18, 0.0);
        }
    } blind;

    robonode::AvoidingPlanner planner{blind, 10, 0.18};
    robonode::Goal goal;
    goal.kind = robonode::Goal::kCartesianPosition;
    goal.cartesian = {0.9, 0.25, 0.5};
    std::vector<std::vector<double>> wp;
    const auto st = planner.plan(std::vector<double>(6, 0.0), goal, wp);
    CHECK(!st.ok());
    CHECK(st.message().find("cannot check collisions") != std::string::npos);
}

// Every configuration the plan passes through is clear — which is the whole
// claim, and the one a straight line cannot make.
void test_avoiding_planner_emits_a_clear_path() {
    auto kin = make_kin();
    robonode::AvoidingPlanner planner{*kin, 20, 0.18};
    robonode::Goal goal;
    goal.kind = robonode::Goal::kCartesianPosition;
    goal.cartesian = {0.9, 0.25, 0.45};

    const std::vector<double> start{0.0, -1.5708, -1.5708, 1.5708, -1.5708, -1.5708};
    std::vector<std::vector<double>> wp;
    CHECK(planner.plan(start, goal, wp).ok());
    CHECK(!wp.empty());

    const std::size_t steps = wp.front().size();
    std::vector<double> q(wp.size());
    for (std::size_t s = 0; s < steps; ++s) {
        for (std::size_t k = 0; k < wp.size(); ++k) q[k] = wp[k][s];
        CHECK(!kin->in_collision(q));
    }
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

void test_cartesian_line_planner_reaches_goal() {
    auto kin = make_kin();
    robonode::CartesianLinePlanner planner{*kin, 25};

    const std::vector<double> q0{0.1, -0.4, 0.5, 0.0, -0.2, 0.0};
    robonode::Goal goal;
    goal.kind = robonode::Goal::kCartesianPosition;
    goal.cartesian = kin->tcp_position(q0) + robonode::Vec3{0.1, -0.06, 0.05};

    std::vector<std::vector<double>> wp;  // [joint][step]
    CHECK(planner.plan(q0, goal, wp).ok());
    CHECK(wp.size() == 6);
    std::vector<double> q_end(6);
    for (std::size_t k = 0; k < 6; ++k) q_end[k] = wp[k].back();
    CHECK((kin->tcp_position(q_end) - goal.cartesian).norm() < 1e-3);

    // Unreachable goal fails closed; wrong goal kind rejected.
    goal.cartesian = robonode::Vec3{10, 10, 10};
    CHECK(!planner.plan(q0, goal, wp).ok());
    goal.kind = robonode::Goal::kJoint;
    CHECK(!planner.plan(q0, goal, wp).ok());
}

// SE(3) seam (#52): a position-only impl gets the full surface via defaults —
// tcp_pose = position + identity orientation; jacobian = 6×dof with the
// position rows filled and the orientation rows zero. Pinocchio (#34)
// The MuJoCo impl answers with the model's real orientation and a real angular
// Jacobian (#92) — the position half still agrees with the position-only path,
// because they are the same columns.
void test_se3_surface_carries_real_orientation() {
    auto kin = make_kin();
    const std::vector<double> q{0.2, -0.3, 0.4, 0.1, -0.2, 0.3};

    const robonode::Pose pose = kin->tcp_pose(q);
    const robonode::Vec3 p = kin->tcp_position(q);
    CHECK((pose.position - p).norm() < 1e-12);
    CHECK(std::abs(pose.orientation.norm() - 1.0) < 1e-9);  // a unit quaternion

    // Turning a wrist joint must turn the TCP: an impl that reported identity
    // would pass every position test and silently make 6-DoF impossible.
    auto turned = q;
    turned[5] += 0.5;
    const auto other = kin->tcp_pose(turned).orientation;
    const double dot = std::abs(pose.orientation.w * other.w + pose.orientation.x * other.x +
                                pose.orientation.y * other.y + pose.orientation.z * other.z);
    CHECK(dot < 0.999);

    const auto J6 = kin->jacobian(q);
    const auto Jp = kin->position_jacobian(q);
    CHECK(J6.size() == 6 * 6);
    const std::size_t n = kin->dof();
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t k = 0; k < n; ++k) CHECK(std::abs(J6[r * n + k] - Jp[r * n + k]) < 1e-12);
    }
    double angular = 0.0;
    for (std::size_t r = 3; r < 6; ++r) {
        for (std::size_t k = 0; k < n; ++k) angular += std::abs(J6[r * n + k]);
    }
    CHECK(angular > 1e-6);
}

// A full pose target: reach the point AND arrive at the angle. Asking for the
// pose the arm is already in must be solvable from a disturbed seed.
void test_ik_pose_reaches_position_and_orientation() {
    auto kin = make_kin();
    const std::vector<double> q_true{0.1, -0.6, 0.8, -0.2, 0.4, 0.3};
    const robonode::Pose target = kin->tcp_pose(q_true);

    robonode::IkOptions opt;
    opt.max_iters = 400;
    std::vector<double> q_sol;
    const auto seed = std::vector<double>{0.0, -0.5, 0.7, 0.0, 0.3, 0.0};
    CHECK(robonode::ik_pose(*kin, seed, target, q_sol, opt).ok());

    const robonode::Pose reached = kin->tcp_pose(q_sol);
    CHECK((reached.position - target.position).norm() < 2e-3);
    const auto turn = robonode::detail::orientation_error(target.orientation, reached.orientation);
    CHECK(turn.norm() < 2e-2);
}

// Orientation is not free: a pose the arm cannot twist into is refused, and
// says how far short it got rather than pretending it arrived.
void test_ik_pose_refuses_an_impossible_orientation() {
    auto kin = make_kin();
    robonode::Pose target = kin->tcp_pose(std::vector<double>(6, 0.0));
    target.position = {10.0, 10.0, 10.0};
    std::vector<double> q_sol;
    const auto st = robonode::ik_pose(*kin, std::vector<double>(6, 0.0), target, q_sol);
    CHECK(!st.ok());
    CHECK(st.message().find("mm short") != std::string::npos);
}

}  // namespace

int main() {
    test_fk_responds_to_joints();
    test_jacobian_matches_finite_difference();
    test_ik_reaches_reachable_target();
    test_ik_reports_unreachable();
    test_planner_names_the_goal_it_was_handed();
    test_kinematics_reports_a_collision();
    test_avoiding_planner_refuses_blind_kinematics();
    test_avoiding_planner_emits_a_clear_path();
    test_move_l_traces_straight_line();
    test_cartesian_jog_produces_requested_tcp_velocity();
    test_cartesian_line_planner_reaches_goal();
    test_se3_surface_carries_real_orientation();
    test_ik_pose_reaches_position_and_orientation();
    test_ik_pose_refuses_an_impossible_orientation();
    std::puts("robonode kinematics: all tests passed");
    return 0;
}
