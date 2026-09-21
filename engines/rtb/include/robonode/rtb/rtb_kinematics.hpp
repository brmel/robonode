#pragma once

#include <string>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/motion/kinematics.hpp"
#include "robonode/rtb/rtb_client.hpp"

namespace robonode {

// The Kinematics seam backed by real robot models (Robotics Toolbox) via the
// rtb-kinematics service — no hand-rolled FK/Jacobian. Throws on service
// failure (non-RT, planning context; callers that need soft failure use
// RtbPlanner, which returns Status).
class RtbKinematics final : public Kinematics {
public:
    RtbKinematics(std::string robot, std::size_t dof, std::string host = "127.0.0.1",
                  int port = 8091)
        : robot_{std::move(robot)}, dof_{dof}, client_{std::move(host), port} {}

    [[nodiscard]] std::size_t dof() const override { return dof_; }

    [[nodiscard]] Vec3 tcp_position(const std::vector<double>& q) const override {
        const auto j = client_.post("/fk", {{"robot", robot_}, {"q", q}});
        const auto& p = j.at("pos");
        return {p[0].get<double>(), p[1].get<double>(), p[2].get<double>()};
    }

    [[nodiscard]] std::vector<double> position_jacobian(
        const std::vector<double>& q) const override {
        const auto j = client_.post("/jacobian", {{"robot", robot_}, {"q", q}});
        return j.at("jac").get<std::vector<double>>();  // 3×dof row-major
    }

private:
    std::string robot_;
    std::size_t dof_;
    rtb::RtbClient client_;
};

}  // namespace robonode
