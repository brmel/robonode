#pragma once

#include <cmath>
#include <string>

#include "robonode/motion/axis_adapter.hpp"

namespace robonode {

// First-order tracking plant: actual position follows the commanded setpoint
// with time constant tau (belt-axis servo ballpark). Enough dynamics for the
// M0/M1 sim gate — following error is nonzero and rate-dependent, so governor
// and telemetry paths are exercised honestly. Replace with a second-order
// model + backlash when tuning work starts.
class SimAxis final : public AxisAdapter {
public:
    SimAxis(std::string name, double initial, double tau_s)
        : name_{std::move(name)}, setpoint_{initial}, tau_s_{tau_s} {
        state_.position = initial;
    }

    void write_setpoint(double position) noexcept override { setpoint_ = position; }

    [[nodiscard]] AxisState read() const noexcept override { return state_; }

    void step(double dt_s) noexcept override {
        const double alpha = 1.0 - std::exp(-dt_s / tau_s_);
        const double prev = state_.position;
        state_.position += alpha * (setpoint_ - state_.position);
        state_.velocity = (state_.position - prev) / dt_s;
    }

    [[nodiscard]] std::string name() const override { return name_; }

    // Fault injection for sim-gate scenarios (FR-3.3): drives the safety
    // state a real adapter would observe from the certified chain.
    void set_safety(SafetyState s) noexcept { state_.safety = s; }

private:
    std::string name_;
    double setpoint_;
    double tau_s_;
    AxisState state_{};
};

}  // namespace robonode
