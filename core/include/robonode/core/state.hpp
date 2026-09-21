#pragma once

namespace robonode {

// Kinematic state of one DOF. Unit-agnostic doubles: units travel in the
// capability descriptor (FR-1.3), not in the type system — a linear axis
// speaks mm, a joint speaks rad, through identical code.
struct State {
    double position{};
    double velocity{};
    double acceleration{};
};

// Safety state observed from the certified chain (mirrors
// robonode.v0.SafetyState — robonode-idl/common.proto). Software observes
// and constrains; it never implements the safety function (SPEC I4).
enum class SafetyState { kNormal, kReduced, kProtectiveStop, kEStop, kFault };

// A tool's constraint is closed or it is open. Lives here because both the
// engine that welds bodies and the cell layer that asks for it must name the
// same thing — and `constrain(name, true)` never said which was which.
enum class Grip { kOpen, kClosed };

// A belt runs or it does not. Same reason as Grip: `run_conveyor(id, false)`
// tells a reader nothing about which way false points.
enum class Belt { kStopped, kRunning };

struct AxisState {
    double position{};
    double velocity{};
    SafetyState safety{SafetyState::kNormal};
};

}  // namespace robonode
