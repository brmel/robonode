#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "robonode/core/limits.hpp"
#include "robonode/core/state.hpp"
#include "robonode/motion/module_registry.hpp"

namespace robonode {

// Control seam: turns the plan reference + measured state into the setpoint the
// governor sees. Runs in the 1 kHz loop, so: noexcept, no allocation, no
// blocking (ADR-5). A WASM engine never runs here — only host-validated code.
class Controller {
public:
    virtual ~Controller() = default;
    virtual double command(const State& target, const AxisState& actual, double dt_s) noexcept = 0;
    virtual void reset(double position) noexcept = 0;
};

class DirectController final : public Controller {
public:
    double command(const State& target, const AxisState&, double) noexcept override {
        return target.position;
    }
    void reset(double) noexcept override {}
};

// Critically-damped command shaping (SmoothDamp): eases to the reference with no
// overshoot, trading a little lag for a gentler follow than Direct.
class SmoothController final : public Controller {
public:
    explicit SmoothController(double smooth_time_s) : smooth_time_s_{smooth_time_s} {}

    double command(const State& target, const AxisState&, double dt_s) noexcept override {
        const double omega = 2.0 / smooth_time_s_;
        const double x = omega * dt_s;
        const double decay = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);
        const double change = pos_ - target.position;
        const double temp = (vel_ + omega * change) * dt_s;
        vel_ = (vel_ - omega * temp) * decay;
        pos_ = target.position + (change + temp) * decay;
        return pos_;
    }
    void reset(double position) noexcept override {
        pos_ = position;
        vel_ = 0.0;
    }

private:
    double smooth_time_s_;
    double pos_{};
    double vel_{};
};

struct ControlContext {
    AxisLimits limits;
    std::map<std::string, std::string> config;
};
using ControlRegistry = ModuleRegistry<Controller, ControlContext>;

inline void register_basic_controllers(ControlRegistry& reg) {
    reg.add("robonode.direct", [](const ControlContext&) -> std::unique_ptr<Controller> {
        return std::make_unique<DirectController>();
    });
    reg.add("robonode.smooth", [](const ControlContext& c) -> std::unique_ptr<Controller> {
        const auto it = c.config.find("smooth_time");
        const double t = it == c.config.end() ? 0.08 : std::strtod(it->second.c_str(), nullptr);
        return std::make_unique<SmoothController>(t > 0.0 ? t : 0.08);
    });
}

}  // namespace robonode
