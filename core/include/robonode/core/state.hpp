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

struct AxisState {
    double position_mm{};
    double velocity_mm_s{};
    SafetyState safety{SafetyState::kNormal};
};

}  // namespace robonode
