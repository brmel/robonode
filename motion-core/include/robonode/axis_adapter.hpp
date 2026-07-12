#pragma once

#include <string>

namespace robonode {

// Mirrors robonode.v0.SafetyState (robonode-idl/common.proto).
enum class SafetyState { kNormal, kReduced, kProtectiveStop, kEStop, kFault };

struct AxisState {
    double position_mm{};
    double velocity_mm_s{};
    SafetyState safety{SafetyState::kNormal};
};

// Vendor-adapter interface for a single-DOF axis (SPEC §3.1 bottom layer).
// Real implementations: EtherCAT CiA402 CSP @1 kHz, UR joint slice via
// RTDE/servoj @500 Hz, Fanuc Stream Motion @8 ms. The sim adapter implements
// the same interface so the motion core is identical against the twin
// (SPEC invariant I5).
//
// Contract: called from the RT cycle — no allocation, no blocking, no
// exceptions escaping. write() takes an ABSOLUTE position setpoint
// (FR-2.10: lost cycles self-heal).
class AxisAdapter {
public:
    virtual ~AxisAdapter() = default;

    virtual void write_setpoint(double position_mm) noexcept = 0;
    [[nodiscard]] virtual AxisState read() const noexcept = 0;

    // Advance the device by one cycle. Fieldbus adapters exchange frames
    // here; the sim adapter integrates its plant model.
    virtual void step(double dt_s) noexcept = 0;

    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace robonode
