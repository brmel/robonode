#pragma once

#include <string>

#include "robonode/core/lifecycle.hpp"
#include "robonode/core/state.hpp"
#include "robonode/motion/steppable.hpp"

namespace robonode {

// Vendor-adapter interface for a single-DOF axis (SPEC §3.1 bottom layer).
// Real implementations: EtherCAT CiA402 CSP @1 kHz, UR joint slice via
// RTDE/servoj @500 Hz, Fanuc Stream Motion @8 ms. The sim adapter implements
// the same interface so the motion core is identical against the twin
// (SPEC invariant I5).
//
// Two disciplines, one interface:
//   - lifecycle verbs (LifecycleParticipant) are non-RT, Status-returning —
//     celld drives them (FR-1.2);
//   - the cycle methods below are RT: no allocation, no blocking, no
//     exceptions escaping; failures latch into AxisState.safety.
// write_setpoint() takes an ABSOLUTE position setpoint (FR-2.10: lost
// cycles self-heal).
class AxisAdapter : public LifecycleParticipant {
public:
    virtual void write_setpoint(double position) noexcept = 0;
    [[nodiscard]] virtual AxisState read() const noexcept = 0;

    // Advance the device by one cycle. Fieldbus adapters exchange frames
    // here; the sim adapter integrates its plant model. Adapters backed by a
    // shared world stepped by the executive (see shared_world) leave this a
    // no-op.
    virtual void step(double dt_s) noexcept = 0;

    // A world this adapter reads/drives but does NOT step itself — the
    // executive ticks it once per cycle, deduped across adapters that share
    // it. Default: none (self-stepping adapter). Returning non-null decouples
    // physics stepping from adapter identity, so swapping any node is safe.
    [[nodiscard]] virtual CycleSteppable* shared_world() const noexcept { return nullptr; }

    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace robonode
