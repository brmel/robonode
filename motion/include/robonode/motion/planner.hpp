#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/core/settings.hpp"
#include "robonode/core/status.hpp"
#include "robonode/motion/cartesian.hpp"
#include "robonode/motion/kinematics.hpp"
#include "robonode/motion/module_registry.hpp"

namespace robonode {

// The planning seam (SPEC §3.1 planner). A Planner turns a goal into joint
// waypoints for SyncBlendPlan — nothing more. That narrowness is the point:
// v0 ships joint-space and Cartesian-straight-line planners; a collision-
// aware OMPL planner swaps in behind this interface without touching any
// caller (the same trick as the OTG slot). Planners never talk to adapters.
//
// Waypoints are returned as waypoints[joint][step], the SyncBlendPlan shape.

struct Goal {
    enum Kind { kJoint, kCartesianPosition, kCartesianPose } kind{kJoint};
    std::vector<double> joint;  // for kJoint
    Vec3 cartesian;             // for kCartesianPosition (TCP xyz)
    Pose pose;                  // for kCartesianPose (TCP position + orientation; #34 fills the planner)

    // How the target itself is moving (m/s), when it is. A planner that ignores
    // this reaches a point the workpiece has already left; one that uses it can
    // arrive travelling alongside, which is how a moving grasp is made.
    Vec3 velocity;
    [[nodiscard]] bool is_moving() const { return velocity.norm() > 1e-6; }
};

// A planner refusing a goal must say which goal it was handed: "non-Cartesian"
// is the same six words whether the caller sent joint angles or a full 6-DoF
// pose, and those are two different problems — with different answers, since
// only one planner version solves orientation.
inline const char* name_of(Goal::Kind kind) {
    switch (kind) {
        case Goal::kJoint: return "joint";
        case Goal::kCartesianPosition: return "Cartesian position";
        case Goal::kCartesianPose:
            return "Cartesian pose (6-DoF — select the robonode.moveP planner)";
    }
    return "unknown";
}

class Planner {
public:
    virtual ~Planner() = default;
    virtual Status plan(const std::vector<double>& start_q, const Goal& goal,
                        std::vector<std::vector<double>>& waypoints) const = 0;
};

// Joint-space: a direct start→goal move (two waypoints; the blend/OTG layer
// smooths and time-parameterises). Collision-blind — the seam is the point.
class JointPlanner final : public Planner {
public:
    Status plan(const std::vector<double>& start_q, const Goal& goal,
                std::vector<std::vector<double>>& waypoints) const override {
        if (goal.kind != Goal::kJoint) return Status::failure(std::string{"JointPlanner: wants joint, was given a "} + name_of(goal.kind) + " goal");
        if (goal.joint.size() != start_q.size()) {
            return Status::failure("JointPlanner: goal has " + std::to_string(goal.joint.size()) +
                                   " joints, the arm has " + std::to_string(start_q.size()));
        }
        waypoints.assign(start_q.size(), {});
        for (std::size_t k = 0; k < start_q.size(); ++k) {
            waypoints[k] = {start_q[k], goal.joint[k]};
        }
        return Status::success();
    }
};

// Cartesian straight line: TCP travels in a line to the goal position,
// resolved to joint waypoints by DLS IK (moveL). Fails closed if any point
// along the line is unreachable. Collision-blind (world model + OMPL, later).
class CartesianLinePlanner final : public Planner {
public:
    CartesianLinePlanner(const Kinematics& kin, int n_steps, IkOptions ik = {})
        : kin_{kin}, n_steps_{n_steps}, ik_{ik} {}

    Status plan(const std::vector<double>& start_q, const Goal& goal,
                std::vector<std::vector<double>>& waypoints) const override {
        if (goal.kind != Goal::kCartesianPosition) {
            return Status::failure(std::string{"CartesianLinePlanner: wants a Cartesian position, was given a "} + name_of(goal.kind) + " goal");
        }
        return plan_move_l(kin_, start_q, goal.cartesian, n_steps_, waypoints, ik_);
    }

private:
    const Kinematics& kin_;
    int n_steps_;
    IkOptions ik_;
};

// Point-to-point (moveJ): resolve the Cartesian goal to joints once, then a
// direct joint-space move. Faster and always feasible where a straight TCP line
// would clip a singularity — the TCP path curves. The classic moveL/moveJ
// choice, now a swappable version.
class JointReachPlanner final : public Planner {
public:
    explicit JointReachPlanner(const Kinematics& kin, IkOptions ik = {}) : kin_{kin}, ik_{ik} {}

    Status plan(const std::vector<double>& start_q, const Goal& goal,
                std::vector<std::vector<double>>& waypoints) const override {
        if (goal.kind != Goal::kCartesianPosition) {
            return Status::failure(std::string{"JointReachPlanner: wants a Cartesian position, was given a "} + name_of(goal.kind) + " goal");
        }
        std::vector<double> q_goal;
        if (const auto st = ik_position(kin_, start_q, goal.cartesian, q_goal, ik_); !st.ok()) {
            return st;
        }
        waypoints.assign(start_q.size(), {});
        for (std::size_t k = 0; k < start_q.size(); ++k) waypoints[k] = {start_q[k], q_goal[k]};
        return Status::success();
    }

private:
    const Kinematics& kin_;
    IkOptions ik_;
};

// Approach a moving target along its own direction of travel: the last stretch
// runs WITH the workpiece rather than across it, so the tool is moving the same
// way at contact. A straight line to a moving point meets it at an angle and
// swipes; this is the difference between touching a part and taking it.
class RendezvousPlanner final : public Planner {
public:
    RendezvousPlanner(const Kinematics& kin, int n_steps, double lead_in_s, IkOptions ik = {})
        : kin_{kin}, n_steps_{n_steps}, lead_in_s_{lead_in_s}, ik_{std::move(ik)} {}

    Status plan(const std::vector<double>& start_q, const Goal& goal,
                std::vector<std::vector<double>>& waypoints) const override {
        if (goal.kind != Goal::kCartesianPosition) {
            return Status::failure(std::string{"RendezvousPlanner: wants a Cartesian position, was given a "} + name_of(goal.kind) + " goal");
        }
        if (!goal.is_moving()) return plan_move_l(kin_, start_q, goal.cartesian, n_steps_, waypoints, ik_);

        // Meet the part upstream, then run alongside it into the grasp point.
        const Vec3 entry{goal.cartesian.x - goal.velocity.x * lead_in_s_,
                         goal.cartesian.y - goal.velocity.y * lead_in_s_,
                         goal.cartesian.z - goal.velocity.z * lead_in_s_};

        std::vector<std::vector<double>> to_entry;
        if (const auto st = plan_move_l(kin_, start_q, entry, n_steps_, to_entry, ik_); !st.ok()) {
            return st;
        }
        std::vector<double> q_entry;
        q_entry.reserve(to_entry.size());
        for (const auto& j : to_entry) q_entry.push_back(j.empty() ? 0.0 : j.back());

        std::vector<std::vector<double>> alongside;
        if (const auto st = plan_move_l(kin_, q_entry, goal.cartesian, n_steps_, alongside, ik_);
            !st.ok()) {
            return st;
        }
        waypoints = std::move(to_entry);
        for (std::size_t j = 0; j < waypoints.size() && j < alongside.size(); ++j) {
            waypoints[j].insert(waypoints[j].end(), alongside[j].begin(), alongside[j].end());
        }
        return Status::success();
    }

private:
    const Kinematics& kin_;
    int n_steps_;
    double lead_in_s_;
    IkOptions ik_;
};

// A straight line is the shortest path and the one most likely to go through
// something. This planner asks the kinematics whether each waypoint would be
// touching anything (#39); if the direct line is clear it IS the direct line,
// and if it is not, the tool lifts to a clearance height, crosses over, and
// comes down — the manoeuvre an operator makes by hand, made explicit.
//
// It is not a general motion planner: no sampling, no roadmap, no guarantee of
// completeness. What it guarantees is the half that matters — it never emits a
// path it knows is blocked. It either routes round or refuses, and a refusal
// says how much clearance it tried, so the answer is actionable rather than a
// shrug. It lives behind the same seam a cuRobo/OMPL version (#39) will use, so
// replacing it changes one registry line.
class AvoidingPlanner final : public Planner {
public:
    AvoidingPlanner(const Kinematics& kin, int n_steps, double clearance_m, IkOptions ik = {})
        : kin_{kin}, n_steps_{std::max(1, n_steps)}, clearance_m_{clearance_m}, ik_{std::move(ik)} {}

    Status plan(const std::vector<double>& start_q, const Goal& goal,
                std::vector<std::vector<double>>& waypoints) const override {
        if (goal.kind != Goal::kCartesianPosition) {
            return Status::failure(std::string{"AvoidingPlanner: wants a Cartesian position, was "
                                               "given a "} +
                                   name_of(goal.kind) + " goal");
        }
        // An impl that cannot answer "is this clear?" must not be asked to
        // route around anything: say so rather than plan a line and call it
        // avoidance.
        if (!kin_.can_check_collision()) {
            return Status::failure("AvoidingPlanner: these kinematics cannot check collisions");
        }

        const auto direct = plan_move_l(kin_, start_q, goal.cartesian, n_steps_, waypoints, ik_);
        if (direct.ok() && clear(waypoints)) return direct;

        // Over the top: up from where the tool is, across at height, down onto
        // the goal. Clearing the obstacle is not enough — an ELBOW can sweep
        // through what the tool passed over — so the height escalates until the
        // whole arm is clear, or the planner admits it cannot find a way.
        const Vec3 from = kin_.tcp_position(start_q);
        std::string why = "the direct line is blocked";
        for (int attempt = 1; attempt <= kAttempts; ++attempt) {
            const double lift = clearance_m_ * attempt;
            std::vector<std::vector<double>> path;
            if (const auto st = plan_over(start_q, goal.cartesian, from, lift, path); !st.ok()) {
                why = st.message();
                continue;
            }
            if (!clear(path)) {
                why = "a link still sweeps through it at " + metres(lift) + " of clearance";
                continue;
            }
            waypoints = std::move(path);
            return Status::success();
        }
        return Status::failure("AvoidingPlanner: no way round — " + why + " (tried up to " +
                               metres(clearance_m_ * kAttempts) + ")");
    }

private:
    static constexpr int kAttempts = 3;  // clearance, twice it, three times it

    static std::string metres(double m) {
        return std::to_string(static_cast<int>(m * 1000.0)) + " mm";
    }

    // Up, across at height, down — each leg a straight line, so a leg that
    // cannot be solved fails for the same reason moveL would.
    Status plan_over(const std::vector<double>& start_q, const Vec3& goal, const Vec3& from,
                     double lift, std::vector<std::vector<double>>& path) const {
        const double top = std::max(from.z, goal.z) + lift;
        std::vector<double> q = start_q;
        for (const Vec3& leg : {Vec3{from.x, from.y, top}, Vec3{goal.x, goal.y, top}, goal}) {
            std::vector<std::vector<double>> segment;
            if (const auto st = plan_move_l(kin_, q, leg, n_steps_, segment, ik_); !st.ok()) {
                return st;
            }
            append(path, segment);
            for (std::size_t k = 0; k < q.size() && k < segment.size(); ++k) q[k] = segment[k].back();
        }
        return Status::success();
    }

    // waypoints[joint][step]: a configuration is a column.
    [[nodiscard]] bool clear(const std::vector<std::vector<double>>& waypoints) const {
        if (waypoints.empty()) return true;
        const std::size_t steps = waypoints.front().size();
        std::vector<double> q(waypoints.size());
        for (std::size_t s = 0; s < steps; ++s) {
            for (std::size_t k = 0; k < waypoints.size(); ++k) q[k] = waypoints[k][s];
            if (kin_.in_collision(q)) return false;
        }
        return true;
    }

    static void append(std::vector<std::vector<double>>& path,
                       const std::vector<std::vector<double>>& segment) {
        if (path.empty()) {
            path = segment;
            return;
        }
        for (std::size_t k = 0; k < path.size() && k < segment.size(); ++k) {
            path[k].insert(path[k].end(), segment[k].begin(), segment[k].end());
        }
    }

    const Kinematics& kin_;
    int n_steps_;
    double clearance_m_;
    IkOptions ik_;
};

// Reach a full pose: the point AND the angle (#92). A tool that must arrive
// square to a face, or normal to a surface, cannot say so with a position
// goal — the wrist ends wherever the solver happened to leave it. This walks
// the TCP in a straight line like moveL and solves orientation along the way,
// so the approach is predictable rather than a swing at the end.
class CartesianPosePlanner final : public Planner {
public:
    CartesianPosePlanner(const Kinematics& kin, int n_steps, IkOptions ik = {})
        : kin_{kin}, n_steps_{std::max(1, n_steps)}, ik_{std::move(ik)} {}

    Status plan(const std::vector<double>& start_q, const Goal& goal,
                std::vector<std::vector<double>>& waypoints) const override {
        if (goal.kind != Goal::kCartesianPose) {
            return Status::failure(std::string{"CartesianPosePlanner: wants a Cartesian pose, was "
                                               "given a "} +
                                   name_of(goal.kind) + " goal");
        }
        const Pose start = kin_.tcp_pose(start_q);
        waypoints.assign(start_q.size(), {});
        std::vector<double> q = start_q;
        for (std::size_t k = 0; k < q.size(); ++k) waypoints[k].push_back(q[k]);

        for (int step = 1; step <= n_steps_; ++step) {
            const double t = static_cast<double>(step) / n_steps_;
            Pose via;
            via.position = start.position + (goal.pose.position - start.position) * t;
            via.orientation = slerp(start.orientation, goal.pose.orientation, t);
            std::vector<double> q_next;
            if (const auto st = ik_pose(kin_, q, via, q_next, ik_); !st.ok()) {
                return Status::failure("moveP: unreachable at step " + std::to_string(step) + ": " +
                                       st.message());
            }
            q = std::move(q_next);
            for (std::size_t k = 0; k < q.size(); ++k) waypoints[k].push_back(q[k]);
        }
        return Status::success();
    }

private:
    // Shortest-arc interpolation between two orientations. Turning the long way
    // round is not a different answer, it is a machine swinging through poses
    // nobody asked for.
    static Quat slerp(Quat a, const Quat& b, double t) {
        double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        if (dot < 0.0) {
            a = {-a.w, -a.x, -a.y, -a.z};
            dot = -dot;
        }
        if (dot > 0.9995) {  // almost the same: lerp and renormalise
            return Quat{a.w + t * (b.w - a.w), a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                        a.z + t * (b.z - a.z)}
                .normalized();
        }
        const double theta = std::acos(dot);
        const double sin_theta = std::sin(theta);
        const double wa = std::sin((1.0 - t) * theta) / sin_theta;
        const double wb = std::sin(t * theta) / sin_theta;
        return Quat{wa * a.w + wb * b.w, wa * a.x + wb * b.x, wa * a.y + wb * b.y,
                    wa * a.z + wb * b.z}
            .normalized();
    }

    const Kinematics& kin_;
    int n_steps_;
    IkOptions ik_;
};

// The trajectory capability (ADR-11): what a planner version is built with (the
// arm kinematics it solves against) + its registry. cuRobo/OMPL (#39) and user
// planners register here; product code selects a version and calls Planner.
struct PlannerContext {
    const Kinematics* kin;
    Settings::Ik ik;                  // damping, tolerance, iteration + step budget
    std::vector<JointBound> bounds;   // joint travel, so IK cannot wind past a stop
    std::vector<double> weights;      // per-joint mobility (a carrier axis is a last resort)
    Vec3 workspace_lo{-3, -3, -1};    // bound on what a user planner may emit
    Vec3 workspace_hi{3, 3, 3};
    std::map<std::string, std::string> config;
};
using PlannerRegistry = ModuleRegistry<Planner, PlannerContext>;

inline IkOptions ik_options(const PlannerContext& c) {
    return {c.ik.lambda,   c.ik.tol_m, c.ik.max_iters,  c.ik.max_step_rad,
            c.bounds,      c.weights,  c.ik.tol_rad,    c.ik.orientation_weight};
}

inline void register_basic_planners(PlannerRegistry& reg) {
    reg.add("robonode.moveL", [](const PlannerContext& c) -> std::unique_ptr<Planner> {
        return std::make_unique<CartesianLinePlanner>(*c.kin, c.ik.line_steps, ik_options(c));
    });
    reg.add("robonode.moveJ", [](const PlannerContext& c) -> std::unique_ptr<Planner> {
        return std::make_unique<JointReachPlanner>(*c.kin, ik_options(c));
    });
    reg.add("robonode.avoid", [](const PlannerContext& c) -> std::unique_ptr<Planner> {
        return std::make_unique<AvoidingPlanner>(*c.kin, c.ik.line_steps, c.ik.clearance_m,
                                                 ik_options(c));
    });
    reg.add("robonode.moveP", [](const PlannerContext& c) -> std::unique_ptr<Planner> {
        return std::make_unique<CartesianPosePlanner>(*c.kin, c.ik.line_steps, ik_options(c));
    });
    reg.add("robonode.rendezvous", [](const PlannerContext& c) -> std::unique_ptr<Planner> {
        return std::make_unique<RendezvousPlanner>(*c.kin, c.ik.line_steps, c.ik.lead_in_s,
                                                   ik_options(c));
    });
}

}  // namespace robonode
