#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
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

// Solve the small dense system A x = b (row-major, m×m, m ≤ 6) by Gaussian
// elimination with partial pivoting. A is damped before it gets here, so a
// pivot that vanishes anyway means a genuine singularity — which the caller
// reports rather than papers over.
inline bool solve_dense(std::vector<double> a, std::vector<double> b, std::vector<double>& x) {
    const std::size_t m = b.size();
    for (std::size_t col = 0; col < m; ++col) {
        std::size_t pivot = col;
        for (std::size_t r = col + 1; r < m; ++r) {
            if (std::abs(a[r * m + col]) > std::abs(a[pivot * m + col])) pivot = r;
        }
        if (std::abs(a[pivot * m + col]) < 1e-12) return false;
        if (pivot != col) {
            for (std::size_t c = 0; c < m; ++c) std::swap(a[col * m + c], a[pivot * m + c]);
            std::swap(b[col], b[pivot]);
        }
        for (std::size_t r = col + 1; r < m; ++r) {
            const double f = a[r * m + col] / a[col * m + col];
            if (f == 0.0) continue;
            for (std::size_t c = col; c < m; ++c) a[r * m + c] -= f * a[col * m + c];
            b[r] -= f * b[col];
        }
    }
    x.assign(m, 0.0);
    for (std::size_t i = m; i-- > 0;) {
        double sum = b[i];
        for (std::size_t c = i + 1; c < m; ++c) sum -= a[i * m + c] * x[c];
        x[i] = sum / a[i * m + i];
    }
    return true;
}

}  // namespace detail

// A joint's travel. Empty bounds mean "unbounded" — accepted only where the
// chain genuinely has none, since unbounded DLS can wind a joint many turns
// past its stop and produce a trajectory that is technically a solution and
// physically nonsense.
struct JointBound {
    double lo, hi;
};

struct IkOptions {
    double lambda = 0.05;       // DLS damping (rad·m); higher = safer near singularities
    double tol_m = 1e-4;        // convergence: TCP error norm (m)
    int max_iters = 200;        // per target
    double max_step_rad = 0.1;  // clamp per-iteration joint step
    std::vector<JointBound> bounds;   // per joint; empty ⇒ unbounded
    std::vector<double> weights;      // per joint mobility; empty ⇒ all 1
    double tol_rad = 5e-3;            // convergence: orientation error (rad)
    double orientation_weight = 0.3;  // how much a radian counts against a metre
};

namespace detail {

inline double weight(std::size_t joint, const IkOptions& opt) {
    return joint < opt.weights.size() ? opt.weights[joint] : 1.0;
}

inline double project(double q, std::size_t joint, const IkOptions& opt) {
    if (joint >= opt.bounds.size()) return q;
    return std::clamp(q, opt.bounds[joint].lo, opt.bounds[joint].hi);
}

}  // namespace detail

using detail::project;

namespace detail {

// The middle of every joint's travel: a seed that belongs to no particular
// pose, and therefore to no particular local minimum.
inline std::vector<double> mid_travel(const std::vector<double>& like, const IkOptions& opt) {
    std::vector<double> q = like;
    for (std::size_t k = 0; k < q.size() && k < opt.bounds.size(); ++k) {
        q[k] = 0.5 * (opt.bounds[k].lo + opt.bounds[k].hi);
    }
    return q;
}

// The residual a solve is driving to zero, and the Jacobian rows that move it.
// Position IK uses three rows; a full pose uses six. ONE iteration either way:
// damping, step clamping and joint travel are the same problem in both.
struct IkTask {
    std::size_t rows{3};
    std::function<std::vector<double>(const std::vector<double>&)> residual;
    std::function<std::vector<double>(const std::vector<double>&)> jacobian;
    std::function<double(const std::vector<double>&)> distance;  // for the failure message
    std::function<bool(const std::vector<double>&)> reached;
};

inline Status ik_from(const Kinematics& kin, std::vector<double> q, const IkTask& task,
                      std::vector<double>& out_q, const IkOptions& opt, double& residual_m,
                      bool& singular);

inline Status ik_two_seeds(const Kinematics& kin, const std::vector<double>& q_seed,
                           const IkTask& task, std::vector<double>& out_q, const IkOptions& opt);

// Where a pose target's orientation error points, as a rotation vector: the
// axis to turn about, scaled by how far. Quaternion difference, shortest arc.
inline Vec3 orientation_error(const Quat& want, const Quat& have) {
    const Quat e{want.w * have.w + want.x * have.x + want.y * have.y + want.z * have.z,
                 -want.w * have.x + want.x * have.w - want.y * have.z + want.z * have.y,
                 -want.w * have.y + want.x * have.z + want.y * have.w - want.z * have.x,
                 -want.w * have.z - want.x * have.y + want.y * have.x + want.z * have.w};
    // The conjugate half of the same rotation: pick the shortest way round, or
    // a 359° turn is chosen over a 1° one.
    const double sign = e.w < 0.0 ? -1.0 : 1.0;
    const Vec3 axis{sign * e.x, sign * e.y, sign * e.z};
    const double sin_half = axis.norm();
    if (sin_half < 1e-9) return {};
    const double angle = 2.0 * std::atan2(sin_half, std::abs(e.w));
    return axis * (angle / sin_half);
}

// "did not converge" is not a diagnosis. How close it got, and whether it
// stopped because the damped system went singular, is the difference between
// "the target is out of reach" and "the arm is folded the wrong way".
// Two reasons, two functions. Behind one flag, `residual` and `iters` were
// arguments the singular branch ignored — which is what a boolean parameter
// selecting between whole messages always turns out to mean.
inline std::string degenerate_pose() { return "ik: singular — the arm is in a degenerate pose"; }

inline std::string short_by(double residual, int iters) {
    return "ik: " + std::to_string(static_cast<int>(residual * 1000.0)) + " mm short after " +
           std::to_string(iters) + " iterations";
}

}  // namespace detail

// Resolve joints so fk(q) reaches `target`, seeded at `q_seed`. Iterative IK
// finds the solution nearest its seed, so a reachable pose can still fail from
// an awkward one — a far carrier position, an arm folded the wrong way. When
// that happens, try again from the middle of the travel before calling the
// target unreachable, which is the difference between "the arm cannot get
// there" and "it could not get there from here".
inline Status ik_position(const Kinematics& kin, const std::vector<double>& q_seed, Vec3 target,
                          std::vector<double>& out_q, const IkOptions& opt = {}) {
    if (q_seed.size() != kin.dof()) return Status::failure("ik: seed size != dof");
    detail::IkTask task;
    task.rows = 3;
    task.residual = [&kin, target](const std::vector<double>& q) {
        const Vec3 e = target - kin.tcp_position(q);
        return std::vector<double>{e.x, e.y, e.z};
    };
    task.jacobian = [&kin](const std::vector<double>& q) { return kin.position_jacobian(q); };
    task.distance = [](const std::vector<double>& e) { return Vec3{e[0], e[1], e[2]}.norm(); };
    task.reached = [tol = opt.tol_m](const std::vector<double>& e) {
        return Vec3{e[0], e[1], e[2]}.norm() < tol;
    };
    return detail::ik_two_seeds(kin, q_seed, task, out_q, opt);
}

// The same solve with orientation in it (#92): six rows instead of three, so a
// tool that must arrive at an ANGLE — a gripper square to a part, a nozzle
// normal to a surface — is expressible at all. Position-only impls of the
// Kinematics seam report identity orientation, so this degrades to a reach
// rather than lying about what it achieved.
inline Status ik_pose(const Kinematics& kin, const std::vector<double>& q_seed, const Pose& target,
                      std::vector<double>& out_q, const IkOptions& opt = {}) {
    if (q_seed.size() != kin.dof()) return Status::failure("ik: seed size != dof");
    const double w = opt.orientation_weight;
    detail::IkTask task;
    task.rows = 6;
    task.residual = [&kin, target, w](const std::vector<double>& q) {
        const Pose now = kin.tcp_pose(q);
        const Vec3 dp = target.position - now.position;
        const Vec3 dr = detail::orientation_error(target.orientation, now.orientation) * w;
        return std::vector<double>{dp.x, dp.y, dp.z, dr.x, dr.y, dr.z};
    };
    task.jacobian = [&kin, w](const std::vector<double>& q) {
        auto j = kin.jacobian(q);
        const std::size_t n = kin.dof();
        for (std::size_t r = 3; r < 6; ++r) {
            for (std::size_t k = 0; k < n; ++k) j[r * n + k] *= w;
        }
        return j;
    };
    task.distance = [](const std::vector<double>& e) { return Vec3{e[0], e[1], e[2]}.norm(); };
    task.reached = [tol_m = opt.tol_m, tol_rad = opt.tol_rad, w](const std::vector<double>& e) {
        const double turn = w > 0.0 ? Vec3{e[3], e[4], e[5]}.norm() / w : 0.0;
        return Vec3{e[0], e[1], e[2]}.norm() < tol_m && turn < tol_rad;
    };
    return detail::ik_two_seeds(kin, q_seed, task, out_q, opt);
}

namespace detail {

// Iterative IK finds the solution nearest its seed, so a reachable target can
// still fail from an awkward one — a far carrier position, an arm folded the
// wrong way. Try the middle of the travel before calling it unreachable: that
// is the difference between "the arm cannot get there" and "not from here".
inline Status ik_two_seeds(const Kinematics& kin, const std::vector<double>& q_seed,
                           const IkTask& task, std::vector<double>& out_q, const IkOptions& opt) {
    double from_seed = 0.0, from_middle = 0.0;
    bool seed_singular = false, middle_singular = false;
    if (const auto st = ik_from(kin, q_seed, task, out_q, opt, from_seed, seed_singular); st.ok()) {
        return st;
    }
    if (const auto st = ik_from(kin, mid_travel(q_seed, opt), task, out_q, opt, from_middle,
                                middle_singular);
        st.ok()) {
        return st;
    }
    // Report the CLOSEST the solver ever got: that is the number that says
    // whether the target is metres away or a millimetre out of travel.
    const bool seed_was_closer = from_seed <= from_middle;
    if (seed_was_closer ? seed_singular : middle_singular) {
        return Status::failure(degenerate_pose());
    }
    return Status::failure(
        short_by(seed_was_closer ? from_seed : from_middle, opt.max_iters));
}

inline Status ik_from(const Kinematics& kin, std::vector<double> q, const IkTask& task,
                      std::vector<double>& out_q, const IkOptions& opt, double& residual_m,
                      bool& singular) {
    const std::size_t n = kin.dof();
    const std::size_t m = task.rows;
    singular = false;
    residual_m = task.distance(task.residual(q));

    for (int it = 0; it < opt.max_iters; ++it) {
        const std::vector<double> e = task.residual(q);
        residual_m = task.distance(e);
        if (task.reached(e)) {
            out_q = std::move(q);
            return Status::success();
        }
        const std::vector<double> J = task.jacobian(q);  // m×n row-major

        // Weighted DLS: A = J W Jᵀ + λ²I. A low weight makes a joint the
        // solver's last resort — how a long carrier axis stays parked unless
        // the arm genuinely cannot reach.
        std::vector<double> A(m * m, 0.0);
        for (std::size_t r = 0; r < m; ++r) {
            for (std::size_t c = 0; c < m; ++c) {
                double sum = 0.0;
                for (std::size_t k = 0; k < n; ++k) {
                    sum += J[r * n + k] * weight(k, opt) * J[c * n + k];
                }
                A[r * m + c] = sum + (r == c ? opt.lambda * opt.lambda : 0.0);
            }
        }
        std::vector<double> y;
        if (!solve_dense(A, e, y)) {  // singular even damped: give up
            singular = true;
            break;
        }

        // dq = W Jᵀ y, step-clamped, then projected back into the joint's travel.
        for (std::size_t k = 0; k < n; ++k) {
            double dq = 0.0;
            for (std::size_t r = 0; r < m; ++r) dq += J[r * n + k] * y[r];
            dq = std::clamp(weight(k, opt) * dq, -opt.max_step_rad, opt.max_step_rad);
            q[k] = project(q[k] + dq, k, opt);
        }
    }
    return Status::failure(singular ? degenerate_pose() : short_by(residual_m, opt.max_iters));
}

}  // namespace detail

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
