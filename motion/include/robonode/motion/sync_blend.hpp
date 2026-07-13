#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "robonode/core/limits.hpp"
#include "robonode/core/state.hpp"

namespace robonode {

// Synchronized multi-axis waypoint plan: linear segments with parabolic
// blends (multi-segment LSPB, Craig §7.4 shape), one shared clock.
//
// This is FR-2.3 + FR-2.4 in one object — the two things Vention's public
// API cannot do on actuators: interior waypoints are passed THROUGH at
// nonzero velocity (parabolic corner, never stopping), and all axes share
// segment timing so they arrive together (slowest axis paces the group).
//
// Construction: segment durations from velocity limits (with headroom for
// blends), blend durations from acceleration limits, then each axis's
// timeline is built by piecewise integration (constant-accel pieces), which
// guarantees C1 position/velocity continuity by construction. The classic
// LSPB corner-cut applies: the path passes within |Δv|·t_b/8 of each
// interior waypoint per axis — that deviation is the blend knob, exactly
// the UR blend-radius / Fanuc CNT semantic.
class SyncBlendPlan {
public:
    // waypoints[axis][k]: same k count for every axis (≥ 2). Motion starts
    // and ends at rest. Limits per axis, read from descriptors (FR-1.3).
    // vel_headroom < 1 reserves velocity margin so blends fit.
    static SyncBlendPlan plan(const std::vector<std::vector<double>>& waypoints,
                              const std::vector<AxisLimits>& limits,
                              double vel_headroom = 0.75, double acc_headroom = 0.9) {
        const std::size_t n_axes = waypoints.size();
        if (n_axes == 0 || limits.size() != n_axes) {
            throw std::invalid_argument{"axes/limits mismatch"};
        }
        const std::size_t m = waypoints[0].size();
        if (m < 2) throw std::invalid_argument{"need at least 2 waypoints"};
        for (const auto& w : waypoints) {
            if (w.size() != m) throw std::invalid_argument{"ragged waypoint lists"};
        }
        for (const auto& l : limits) {
            if (l.velocity_max_mm_s <= 0.0 || l.acceleration_max_mm_s2 <= 0.0) {
                throw std::invalid_argument{"limits must be positive"};
            }
        }

        const std::size_t n_seg = m - 1;

        // Segment durations: slowest axis paces the group (synchronization).
        std::vector<double> T(n_seg, 1e-3);  // floor: never zero-duration
        for (std::size_t k = 0; k < n_seg; ++k) {
            for (std::size_t i = 0; i < n_axes; ++i) {
                const double d = std::abs(waypoints[i][k + 1] - waypoints[i][k]);
                T[k] = std::max(T[k], d / (vel_headroom * limits[i].velocity_max_mm_s));
            }
        }

        // Iterate: velocities from durations, blend times from accel limits,
        // stretch any segment whose blends don't fit, repeat until stable.
        std::vector<std::vector<double>> v(n_axes, std::vector<double>(n_seg));
        std::vector<double> tb(m, 0.0);  // blend duration centered on each waypoint
        for (int iter = 0; iter < 16; ++iter) {
            for (std::size_t i = 0; i < n_axes; ++i) {
                for (std::size_t k = 0; k < n_seg; ++k) {
                    v[i][k] = (waypoints[i][k + 1] - waypoints[i][k]) / T[k];
                }
            }
            for (std::size_t k = 0; k < m; ++k) {
                double t = 1e-9;  // avoid zero-duration blends
                for (std::size_t i = 0; i < n_axes; ++i) {
                    const double v_in = k == 0 ? 0.0 : v[i][k - 1];
                    const double v_out = k == n_seg ? 0.0 : v[i][k];
                    t = std::max(t, std::abs(v_out - v_in) /
                                        (acc_headroom * limits[i].acceleration_max_mm_s2));
                }
                tb[k] = t;
            }
            bool ok = true;
            for (std::size_t k = 0; k < n_seg; ++k) {
                const double needed = (tb[k] + tb[k + 1]) / 2.0;
                if (T[k] < needed) {
                    T[k] = needed * 1.05;  // stretch; velocities shrink next iter
                    ok = false;
                }
            }
            if (ok) break;
        }

        // Build each axis's timeline by integration: constant-accel blend
        // pieces around waypoint times, constant-velocity cruises between.
        std::vector<double> tau(m);
        tau[0] = tb[0] / 2.0;
        for (std::size_t k = 1; k < m; ++k) tau[k] = tau[k - 1] + T[k - 1];

        SyncBlendPlan p;
        p.n_axes_ = n_axes;
        p.duration_ = tau[m - 1] + tb[m - 1] / 2.0;
        p.pieces_.resize(n_axes);
        for (std::size_t i = 0; i < n_axes; ++i) {
            auto& pieces = p.pieces_[i];
            double pos = waypoints[i][0];
            double vel = 0.0;
            double t_cursor = 0.0;
            for (std::size_t k = 0; k < m; ++k) {
                // Blend at waypoint k: [tau_k - tb_k/2, tau_k + tb_k/2]
                const double v_out = k == n_seg ? 0.0 : v[i][k];
                const double a = (v_out - vel) / tb[k];
                pieces.push_back({t_cursor, tb[k], pos, vel, a});
                pos += vel * tb[k] + 0.5 * a * tb[k] * tb[k];
                vel = v_out;
                t_cursor += tb[k];
                if (k < n_seg) {  // cruise to the next blend's start
                    const double cruise = (tau[k + 1] - tb[k + 1] / 2.0) - (tau[k] + tb[k] / 2.0);
                    pieces.push_back({t_cursor, cruise, pos, vel, 0.0});
                    pos += vel * cruise;
                    t_cursor += cruise;
                }
            }
        }
        return p;
    }

    [[nodiscard]] double duration() const { return duration_; }
    [[nodiscard]] std::size_t axes() const { return n_axes_; }

    [[nodiscard]] State sample(std::size_t axis, double t) const {
        const auto& pieces = pieces_[axis];
        if (t <= 0.0) return {pieces.front().p0, 0.0, 0.0};
        for (const auto& pc : pieces) {
            if (t < pc.t0 + pc.dt || &pc == &pieces.back()) {
                const double tau = std::clamp(t - pc.t0, 0.0, pc.dt);
                return {pc.p0 + pc.v0 * tau + 0.5 * pc.a * tau * tau, pc.v0 + pc.a * tau, pc.a};
            }
        }
        return {pieces.back().p0, 0.0, 0.0};  // unreachable
    }

private:
    struct Piece {
        double t0, dt, p0, v0, a;
    };
    std::size_t n_axes_{};
    double duration_{};
    std::vector<std::vector<Piece>> pieces_;
};

}  // namespace robonode
