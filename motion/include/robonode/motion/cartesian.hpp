#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/core/status.hpp"
#include "robonode/motion/kinematics.hpp"

namespace robonode {

// The v0 Cartesian layer (FR-2.5-lite / ArmKinematics@1 position half):
// damped-least-squares inverse kinematics and moveL. moveL walks the TCP in
// a straight line, resolving joint angles at each step, and emits joint
// waypoints for SyncBlendPlan — so a Cartesian command flows through the
// same blend/OTG/governor pipeline as everything else. Position-only;
// orientation joins with full 6-DOF IK later.

namespace detail {

// Solve the 3×3 system A x = b (row-major A) by Cramer's rule. Returns false
// if |det| is ~0 (caller falls back to a damped/held step).
inline bool solve3x3(const std::array<double, 9>& a, const Vec3& b, Vec3& x) {
    const double det =
        a[0] * (a[4] * a[8] - a[5] * a[7]) - a[1] * (a[3] * a[8] - a[5] * a[6]) +
        a[2] * (a[3] * a[7] - a[4] * a[6]);
    if (std::abs(det) < 1e-12) return false;
    const double inv = 1.0 / det;
    x.x = inv * (b.x * (a[4] * a[8] - a[5] * a[7]) - a[1] * (b.y * a[8] - a[5] * b.z) +
                 a[2] * (b.y * a[7] - a[4] * b.z));
    x.y = inv * (a[0] * (b.y * a[8] - a[5] * b.z) - b.x * (a[3] * a[8] - a[5] * a[6]) +
                 a[2] * (a[3] * b.z - b.y * a[6]));
    x.z = inv * (a[0] * (a[4] * b.z - b.y * a[7]) - a[1] * (a[3] * b.z - b.y * a[6]) +
                 b.x * (a[3] * a[7] - a[4] * a[6]));
    return true;
}

}  // namespace detail

struct IkOptions {
    double lambda = 0.05;       // DLS damping (rad·m); higher = safer near singularities
    double tol_m = 1e-4;        // convergence: TCP error norm (m)
    int max_iters = 200;        // per target
    double max_step_rad = 0.1;  // clamp per-iteration joint step
};

// Resolve joints so fk(q) reaches `target`, seeded at `q_seed`. On success
// `out_q` is the solution; failure Status if it did not converge.
inline Status ik_position(const Kinematics& kin, const std::vector<double>& q_seed, Vec3 target,
                          std::vector<double>& out_q, const IkOptions& opt = {}) {
    const std::size_t n = kin.dof();
    if (q_seed.size() != n) return Status::failure("ik: seed size != dof");
    std::vector<double> q = q_seed;

    for (int it = 0; it < opt.max_iters; ++it) {
        const Vec3 e = target - kin.tcp_position(q);
        if (e.norm() < opt.tol_m) {
            out_q = std::move(q);
            return Status::success();
        }
        const std::vector<double> J = kin.position_jacobian(q);  // 3×n row-major

        // A = J Jᵀ + λ²I   (3×3)
        std::array<double, 9> A{};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                double s = 0.0;
                for (std::size_t k = 0; k < n; ++k) s += J[r * n + k] * J[c * n + k];
                A[r * 3 + c] = s + (r == c ? opt.lambda * opt.lambda : 0.0);
            }
        }
        Vec3 y{};
        if (!detail::solve3x3(A, e, y)) break;  // singular even damped: give up

        // dq = Jᵀ y, clamped.
        for (std::size_t k = 0; k < n; ++k) {
            double dq = J[0 * n + k] * y.x + J[1 * n + k] * y.y + J[2 * n + k] * y.z;
            dq = std::clamp(dq, -opt.max_step_rad, opt.max_step_rad);
            q[k] += dq;
        }
    }
    return Status::failure("ik: did not converge");
}

// Cartesian jog: map a desired TCP velocity to joint velocities at the
// current pose via damped least squares — dq = Jᵀ(JJᵀ+λ²I)⁻¹ v. The FR-2.1
// Cartesian jog primitive; near singularities the damping bounds joint
// speed instead of blowing up. Returns zeros (and reports) if degenerate.
inline Status cartesian_jog_velocity(const Kinematics& kin, const std::vector<double>& q,
                                     Vec3 v_cart, std::vector<double>& dq_out,
                                     double lambda = 0.05) {
    const std::size_t n = kin.dof();
    if (q.size() != n) return Status::failure("jog: q size != dof");
    const std::vector<double> J = kin.position_jacobian(q);

    std::array<double, 9> A{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (std::size_t k = 0; k < n; ++k) s += J[r * n + k] * J[c * n + k];
            A[r * 3 + c] = s + (r == c ? lambda * lambda : 0.0);
        }
    }
    Vec3 y{};
    if (!detail::solve3x3(A, v_cart, y)) {
        dq_out.assign(n, 0.0);
        return Status::failure("jog: singular");
    }
    dq_out.assign(n, 0.0);
    for (std::size_t k = 0; k < n; ++k) {
        dq_out[k] = J[0 * n + k] * y.x + J[1 * n + k] * y.y + J[2 * n + k] * y.z;
    }
    return Status::success();
}

// moveL: straight TCP line from fk(q_start) to `target` in n_steps. Returns
// joint waypoints as waypoints[joint][step] (dof lists of length n_steps+1),
// ready for SyncBlendPlan. Fails closed if any step is unreachable.
inline Status plan_move_l(const Kinematics& kin, const std::vector<double>& q_start, Vec3 target,
                          int n_steps, std::vector<std::vector<double>>& waypoints,
                          const IkOptions& opt = {}) {
    const std::size_t n = kin.dof();
    if (q_start.size() != n) return Status::failure("moveL: start size != dof");
    if (n_steps < 1) return Status::failure("moveL: n_steps < 1");

    const Vec3 p0 = kin.tcp_position(q_start);
    waypoints.assign(n, {});
    for (auto& w : waypoints) w.reserve(n_steps + 1);
    for (std::size_t k = 0; k < n; ++k) waypoints[k].push_back(q_start[k]);

    std::vector<double> q = q_start;
    for (int s = 1; s <= n_steps; ++s) {
        const double u = static_cast<double>(s) / n_steps;
        const Vec3 p = p0 + (target - p0) * u;  // straight-line interpolation
        std::vector<double> q_next;
        if (const auto st = ik_position(kin, q, p, q_next, opt); !st.ok()) {
            return Status::failure("moveL: unreachable at step " + std::to_string(s) + ": " +
                                   st.message());
        }
        q = std::move(q_next);
        for (std::size_t k = 0; k < n; ++k) waypoints[k].push_back(q[k]);
    }
    return Status::success();
}

}  // namespace robonode
