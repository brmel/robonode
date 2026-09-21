#pragma once

#include <algorithm>
#include <string>

#include "robonode/motion/axis_adapter.hpp"
#include "robonode/motion/driver_registry.hpp"

namespace robonode {

// Bring-your-own node — the copy-me template (#23).
//
// A driver is just an AxisAdapter: implement four methods, register a factory,
// and your version shows up in every node's dropdown, swappable live. Nothing
// here is privileged — this file is what a user copies to plug in their own
// control law. It reads a gain from the descriptor config to show how per-node
// parameters arrive (config is data, not code).
//
// This example is a proportional tracker: each cycle it steps a fraction
// `gain` of the way from the actual position to the commanded setpoint. Higher
// gain tracks faster. Visibly its own response, so "try each version" has a
// real third option to try next to the built-in sim drivers.
class ByoAxis final : public AxisAdapter {
public:
    ByoAxis(std::string name, double initial, double gain)
        : name_{std::move(name)}, gain_{std::clamp(gain, 0.0, 1.0)} {
        state_.position = initial;
    }

    void write_setpoint(double position) noexcept override { setpoint_ = position; }

    void step(double dt_s) noexcept override {
        const double prev = state_.position;
        state_.position += gain_ * (setpoint_ - state_.position);
        state_.velocity = (state_.position - prev) / dt_s;
    }

    [[nodiscard]] AxisState read() const noexcept override { return state_; }
    [[nodiscard]] std::string name() const override { return name_; }

private:
    std::string name_;
    double gain_;
    double setpoint_{};
    AxisState state_{};
};

// Register "robonode.byo-example". Optional config "gain" (0..1, default 0.2).
// Homed at 0 clamped into the node's range, like the built-in sim drivers.
inline void register_byo_axis(DriverRegistry& registry) {
    registry.register_driver("robonode.byo-example", [](const DriverContext& ctx) {
        double gain = 0.2;
        if (const auto it = ctx.config.find("gain"); it != ctx.config.end()) {
            try {
                gain = std::stod(it->second);
            } catch (...) {
                // Bad value → keep the default; the driver never fails on config.
            }
        }
        const double home = std::clamp(0.0, ctx.limits.position_min, ctx.limits.position_max);
        return std::make_unique<ByoAxis>(ctx.id, home, gain);
    });
}

}  // namespace robonode
