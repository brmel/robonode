#pragma once

#include <cmath>

namespace robonode {

// Minimal 3-vector for Cartesian TCP positions. Orientation is out of scope
// for the v0 Cartesian layer (position-only moveL; the 6-DOF arm's extra DOF
// is free / nullspace) — a Pose with orientation lands with full 6-DOF IK.
struct Vec3 {
    double x{}, y{}, z{};

    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    [[nodiscard]] double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

}  // namespace robonode
