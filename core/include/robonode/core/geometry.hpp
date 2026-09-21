#pragma once

#include <cmath>

namespace robonode {

// Minimal 3-vector for Cartesian TCP positions and the linear/angular parts
// of a twist.
struct Vec3 {
    double x{}, y{}, z{};

    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    [[nodiscard]] double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

// Unit quaternion (w, x, y, z) for TCP orientation. v0 position-only impls use
// identity(); a full-kinematics impl (Pinocchio) fills real orientation.
struct Quat {
    double w{1.0}, x{}, y{}, z{};

    static Quat identity() { return {1.0, 0.0, 0.0, 0.0}; }
    [[nodiscard]] double norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }
    [[nodiscard]] Quat normalized() const {
        const double n = norm();
        return n > 0.0 ? Quat{w / n, x / n, y / n, z / n} : identity();
    }
};

// Full SE(3) TCP pose: position + orientation. The Cartesian layer speaks this
// once full 6-DOF IK lands; position-only planners use the .position half.
struct Pose {
    Vec3 position;
    Quat orientation{Quat::identity()};
};

}  // namespace robonode
