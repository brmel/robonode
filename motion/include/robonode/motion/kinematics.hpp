#pragma once

#include <vector>

#include "robonode/core/geometry.hpp"

namespace robonode {

// The kinematics seam (ArmKinematics@1, position half). Whatever provides
// forward kinematics + a position Jacobian for an articulated chain: today a
// MuJoCo-backed impl (the model IS the reference), later a URDF-based one.
// The Cartesian layer (moveL, jog) speaks only this interface, so it never
// depends on MuJoCo.
//
// Units are the model's (metres, radians). Joint order is fixed by the impl
// and documented by dof()/the descriptor node order.
class Kinematics {
public:
    virtual ~Kinematics() = default;

    // Number of actuated joints this chain controls.
    [[nodiscard]] virtual std::size_t dof() const = 0;

    // Forward kinematics: joint vector (size dof) → TCP position.
    [[nodiscard]] virtual Vec3 tcp_position(const std::vector<double>& q) const = 0;

    // Position Jacobian d(tcp)/d(q): 3×dof, row-major (rows x/y/z).
    [[nodiscard]] virtual std::vector<double> position_jacobian(
        const std::vector<double>& q) const = 0;
};

}  // namespace robonode
