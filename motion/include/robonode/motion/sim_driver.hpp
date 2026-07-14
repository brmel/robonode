#pragma once

#include <algorithm>
#include <memory>

#include "robonode/motion/driver_registry.hpp"
#include "robonode/motion/sim_axis.hpp"

namespace robonode {

// Registers the built-in "robonode.sim-axis" driver: a first-order SimAxis
// homed at 0 clamped into the node's range (a sensible neutral pose). The
// physics twin (MuJoCo) registers its own driver name the same way — the
// registry doesn't care which is which, and celld never learns the
// difference, so a node's implementation can be swapped behind the seam.
inline void register_sim_axis(DriverRegistry& registry) {
    registry.register_driver("robonode.sim-axis", [](const DriverContext& ctx) {
        const double home =
            std::clamp(0.0, ctx.limits.position_min_mm, ctx.limits.position_max_mm);
        return std::make_unique<SimAxis>(ctx.id, home, /*tau_s=*/0.005);
    });
}

// A second, deliberately more sluggish sim version (larger time constant) —
// a real alternative to swap a node to, so "try each version of a node" has
// something to try. Same AxisAdapter contract; visibly different response.
inline void register_sim_axis_soft(DriverRegistry& registry) {
    registry.register_driver("robonode.sim-axis-soft", [](const DriverContext& ctx) {
        const double home =
            std::clamp(0.0, ctx.limits.position_min_mm, ctx.limits.position_max_mm);
        return std::make_unique<SimAxis>(ctx.id, home, /*tau_s=*/0.025);
    });
}

}  // namespace robonode
