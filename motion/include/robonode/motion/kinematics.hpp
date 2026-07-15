#pragma once

#include <vector>

#include "robonode/core/geometry.hpp"

namespace robonode {

// The kinematics seam (ArmKinematics@1). Whatever provides forward kinematics
// + a Jacobian for an articulated chain: today a MuJoCo-backed impl (the model
// IS the reference) and a Robotics Toolbox impl (offline); a Pinocchio impl
// (#34) lands the in-process SE(3) path. The Cartesian layer speaks only this
// interface, so it never depends on any one engine.
//
// Units are the model's (metres, radians). Joint order is fixed by the impl
// and documented by dof()/the descriptor node order.
//
// SE(3) is the full surface (tcp_pose + 6×dof jacobian); the position half
// (tcp_position + position_jacobian) is the minimal contract every impl must
// provide. The SE(3) methods DEFAULT to position-only + identity orientation,
// so existing position-only impls work unchanged; a full-kinematics impl
// (Pinocchio) overrides them with real orientation. This lets #34 fill the
// seam instead of breaking it.
class Kinematics {
public:
    virtual ~Kinematics() = default;

    // Number of actuated joints this chain controls.
    [[nodiscard]] virtual std::size_t dof() const = 0;

    // --- minimal contract: position ---
    // Forward kinematics: joint vector (size dof) → TCP position.
    [[nodiscard]] virtual Vec3 tcp_position(const std::vector<double>& q) const = 0;

    // Position Jacobian d(tcp)/d(q): 3×dof, row-major (rows x/y/z).
    [[nodiscard]] virtual std::vector<double> position_jacobian(
        const std::vector<double>& q) const = 0;

    // --- full SE(3) surface (default: position-only) ---
    // Full pose. Default: tcp_position + identity orientation. Override for
    // real orientation.
    [[nodiscard]] virtual Pose tcp_pose(const std::vector<double>& q) const {
        return {tcp_position(q), Quat::identity()};
    }

    // Geometric Jacobian 6×dof, row-major (rows [vx vy vz wx wy wz]). Default:
    // top 3 rows = position_jacobian, orientation rows = 0. Override for a real
    // angular Jacobian.
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
