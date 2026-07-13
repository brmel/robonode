#pragma once

#include <memory>
#include <string>

#include "robonode/motion/driver_registry.hpp"
#include "robonode/sim_mujoco/mujoco_axis_adapter.hpp"

namespace robonode {

// Registers "robonode.mujoco-axis": a single-axis physics node. Each node
// gets its own world (one joint), so this driver is for standalone axes; an
// arm shares one world across joints and is built by the arm module (#3),
// not through this per-node factory.
//
// Required config: "world" (MJCF path), "joint", "actuator". Optional
// "units_per_m" (default 1000 = descriptor mm). Factory returns null on any
// missing/invalid config or world-load failure (registry reports it).
inline void register_mujoco_axis(DriverRegistry& registry) {
    registry.register_driver(
        "robonode.mujoco-axis", [](const DriverContext& ctx) -> std::unique_ptr<AxisAdapter> {
            const auto world = ctx.config.find("world");
            const auto joint = ctx.config.find("joint");
            const auto actuator = ctx.config.find("actuator");
            if (world == ctx.config.end() || joint == ctx.config.end() ||
                actuator == ctx.config.end()) {
                return nullptr;
            }
            double units_per_m = 1000.0;
            if (const auto u = ctx.config.find("units_per_m"); u != ctx.config.end()) {
                try {
                    units_per_m = std::stod(u->second);
                } catch (...) {
                    return nullptr;
                }
            }
            std::shared_ptr<MujocoWorld> w;
            if (!MujocoWorld::load(world->second, w).ok()) return nullptr;
            return std::make_unique<MujocoAxisAdapter>(ctx.id, std::move(w), joint->second,
                                                       actuator->second, units_per_m,
                                                       /*clock_owner=*/true);
        });
}

}  // namespace robonode
