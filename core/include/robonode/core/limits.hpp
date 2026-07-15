#pragma once

namespace robonode {

// Mirrors robonode.v0.AxisLimits — populated from the node descriptor at
// configure time, never compiled in (FR-1.3). Field names carry the shop
// default units; rotary capabilities reinterpret per their descriptor.
struct AxisLimits {
    double position_min{};
    double position_max{};
    double velocity_max{};
    double acceleration_max{};
    double jerk_max{};
};

// Per-move kinematic profile (mirrors robonode.v0.MotionProfile).
// jerk == 0 selects a trapezoidal profile explicitly — jerk is honored,
// not decorative.
struct MotionProfile {
    double velocity{};
    double acceleration{};
    double jerk{};
};

}  // namespace robonode
