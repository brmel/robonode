#pragma once

#include <string>
#include <vector>

#include "robonode/core/status.hpp"
#include "robonode/motion/planner.hpp"
#include "robonode/rtb/rtb_client.hpp"

namespace robonode {

// The Planner seam backed by real robot models (Robotics Toolbox). A
// Cartesian goal is planned by the service's real IK (one round-trip →
// joint waypoints for SyncBlendPlan); a joint goal is a direct move. This is
// the mature-library planner dropped in behind our seam — celld and the
// executive are unchanged.
class RtbPlanner final : public Planner {
public:
    RtbPlanner(std::string robot, int steps = 25, std::string host = "127.0.0.1", int port = 8091)
        : robot_{std::move(robot)}, steps_{steps}, client_{std::move(host), port} {}

    Status plan(const std::vector<double>& start_q, const Goal& goal,
                std::vector<std::vector<double>>& waypoints) const override {
        if (goal.kind == Goal::kJoint) {
            if (goal.joint.size() != start_q.size()) {
                return Status::failure("RtbPlanner: joint/start dof mismatch");
            }
            waypoints.assign(start_q.size(), {});
            for (std::size_t k = 0; k < start_q.size(); ++k) {
                waypoints[k] = {start_q[k], goal.joint[k]};
            }
            return Status::success();
        }
        // Cartesian: real IK along a straight TCP line, in the service.
        try {
            const auto j = client_.post(
                "/plan_move_l",
                {{"robot", robot_},
                 {"q_start", start_q},
                 {"target", {goal.cartesian.x, goal.cartesian.y, goal.cartesian.z}},
                 {"steps", steps_}});
            if (!j.value("ok", false)) {
                return Status::failure("RtbPlanner: " + j.value("error", "plan failed"));
            }
            waypoints = j.at("waypoints").get<std::vector<std::vector<double>>>();
            return Status::success();
        } catch (const std::exception& e) {
            return Status::failure(std::string{"RtbPlanner: "} + e.what());
        }
    }

private:
    std::string robot_;
    int steps_;
    rtb::RtbClient client_;
};

}  // namespace robonode
