#pragma once

#include <memory>

#include "robonode/motion/driver_registry.hpp"
#include "robonode/motion/sim_axis.hpp"

namespace robonode {

// Registers the built-in "robonode.sim-axis" driver: a first-order SimAxis
// seeded at the node's lower position limit. The physics twin (MuJoCo)
// registers its own driver name the same way — the registry doesn't care
// which is which, and celld never learns the difference.
inline void register_sim_axis(DriverRegistry& registry) {
    registry.register_driver("robonode.sim-axis", [](const DriverContext& ctx) {
        return std::make_unique<SimAxis>(ctx.id, ctx.limits.position_min_mm, /*tau_s=*/0.005);
    });
}

}  // namespace robonode
