#pragma once

#include <utility>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/core/status.hpp"
#include "robonode/motion/cartesian.hpp"
#include "robonode/motion/kinematics.hpp"

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
};

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
        if (goal.kind != Goal::kJoint) return Status::failure("JointPlanner: non-joint goal");
        if (goal.joint.size() != start_q.size()) {
            return Status::failure("JointPlanner: goal/start dof mismatch");
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
            return Status::failure("CartesianLinePlanner: non-Cartesian goal");
        }
        return plan_move_l(kin_, start_q, goal.cartesian, n_steps_, waypoints, ik_);
    }

private:
    const Kinematics& kin_;
    int n_steps_;
    IkOptions ik_;
};

}  // namespace robonode
