#pragma once

#include <memory>
#include <string>

#include "robonode/motion/driver_registry.hpp"
#include "robonode/sim_mujoco/mujoco_axis_adapter.hpp"

namespace robonode {

// Registers "robonode.mujoco-axis". Nodes naming the same "world" path share
// one physics world (via a pool captured in the factory), so an arm's 7 DOF
// — 7 descriptors, one world path — drive one body; the executive ticks that
// shared world once per cycle (no node "owns" the clock, #50). celld keeps its
// per-node model and stays vendor-blind; physics is shared underneath.
//
// Required config: "world" (MJCF path), "joint", "actuator". Optional
// "units_per_m" (default 1000 = descriptor mm). Factory returns null on any
// missing/invalid config or world-load failure (registry reports it).
inline void register_mujoco_axis(DriverRegistry& registry) {
    auto pool = std::make_shared<MujocoWorldPool>();
    registry.register_driver(
        "robonode.mujoco-axis",
        [pool](const DriverContext& ctx) -> std::unique_ptr<AxisAdapter> {
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
            if (!pool->get(world->second, w).ok()) return nullptr;
            return std::make_unique<MujocoAxisAdapter>(ctx.id, std::move(w), joint->second,
                                                       actuator->second, units_per_m);
        });
}

}  // namespace robonode
