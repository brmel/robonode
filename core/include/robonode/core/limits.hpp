#pragma once

namespace robonode {

// Mirrors robonode.v0.AxisLimits — populated from the node descriptor at
// configure time, never compiled in (FR-1.3). Field names carry the shop
// default units; rotary capabilities reinterpret per their descriptor.
struct AxisLimits {
    double position_min_mm{};
    double position_max_mm{};
    double velocity_max_mm_s{};
    double acceleration_max_mm_s2{};
    double jerk_max_mm_s3{};
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
