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
    SimAxis(std::string name, double initial_mm, double tau_s)
        : name_{std::move(name)}, setpoint_mm_{initial_mm}, tau_s_{tau_s} {
        state_.position_mm = initial_mm;
    }

    void write_setpoint(double position_mm) noexcept override { setpoint_mm_ = position_mm; }

    [[nodiscard]] AxisState read() const noexcept override { return state_; }

    void step(double dt_s) noexcept override {
        const double alpha = 1.0 - std::exp(-dt_s / tau_s_);
        const double prev = state_.position_mm;
        state_.position_mm += alpha * (setpoint_mm_ - state_.position_mm);
        state_.velocity_mm_s = (state_.position_mm - prev) / dt_s;
    }

    [[nodiscard]] std::string name() const override { return name_; }

    // Fault injection for sim-gate scenarios (FR-3.3): drives the safety
    // state a real adapter would observe from the certified chain.
    void set_safety(SafetyState s) noexcept { state_.safety = s; }

private:
    std::string name_;
    double setpoint_mm_;
    double tau_s_;
    AxisState state_{};
};

}  // namespace robonode
