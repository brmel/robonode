#pragma once

#include <vector>

#include "robonode/core/geometry.hpp"

namespace robonode {

// The kinematics seam (ArmKinematics@1). Units are the model's (metres,
// radians); joint order is the descriptor's node order.
//
// Position (tcp_position + position_jacobian) is what every implementation must
// provide. The SE(3) half defaults to position-only with identity orientation,
// so a position-only implementation stays valid and a full one (Pinocchio, #34)
// fills the seam by overriding rather than by changing it.
class Kinematics {
public:
    virtual ~Kinematics() = default;

    [[nodiscard]] virtual std::size_t dof() const = 0;

    // q has dof() entries.
    [[nodiscard]] virtual Vec3 tcp_position(const std::vector<double>& q) const = 0;

    // 3×dof, row-major, rows x/y/z.
    [[nodiscard]] virtual std::vector<double> position_jacobian(
        const std::vector<double>& q) const = 0;

    [[nodiscard]] virtual Pose tcp_pose(const std::vector<double>& q) const {
        return {tcp_position(q), Quat::identity()};
    }

    // Would the arm be touching something in this configuration? A planner that
    // can ask this can route AROUND an obstacle instead of through it (#39).
    // Default: no — an impl that cannot answer must not claim a path is clear,
    // so it says nothing rather than something false, and the planner that
    // needs it refuses instead of pretending.
    [[nodiscard]] virtual bool can_check_collision() const { return false; }
    [[nodiscard]] virtual bool in_collision(const std::vector<double>&) const { return false; }

    // 6×dof, row-major, rows [vx vy vz wx wy wz].
    [[nodiscard]] virtual std::vector<double> jacobian(const std::vector<double>& q) const {
        const auto jp = position_jacobian(q);
        const std::size_t n = dof();
        std::vector<double> j(6 * n, 0.0);
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t k = 0; k < n; ++k) j[r * n + k] = jp[r * n + k];
        }
        return j;
    }
};

}  // namespace robonode
