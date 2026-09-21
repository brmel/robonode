#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "robonode/core/geometry.hpp"

namespace robonode {

// A camera frame: 8-bit BGR, the layout every vision library expects, plus the
// time it was CAPTURED. A frame is always older than the moment you look at it.
struct Frame {
    int width{0};
    int height{0};
    double captured_s{0.0};
    std::vector<std::uint8_t> bgr;  // width * height * 3

    [[nodiscard]] bool empty() const { return width <= 0 || height <= 0 || bgr.empty(); }
};

// A pinhole camera looking down at a work plane. Enough model to turn pixels
// into world coordinates, which is the step a detector cannot skip and the one
// that makes vision a real node rather than a scene oracle.
struct CameraModel {
    // Chosen so a 320×240 sensor 0.93 m above the belt covers its full ±0.55 m
    // of travel: a camera that cannot see the whole belt is a cell design bug,
    // not an algorithm problem.
    double fx{260.0}, fy{260.0};
    double cx{160.0}, cy{120.0};
    Vec3 position{0.9, 0.25, 1.30};  // looking straight down (-Z)

    [[nodiscard]] std::optional<std::pair<double, double>> project(Vec3 world) const {
        const double dz = position.z - world.z;
        if (dz <= 1e-6) return std::nullopt;  // behind or at the lens
        return std::pair{cx + fx * (world.x - position.x) / dz,
                         cy + fy * (world.y - position.y) / dz};
    }

    // Pixels back to the world, given the plane the target sits on. A single
    // camera cannot recover depth; the work plane supplies it.
    [[nodiscard]] Vec3 unproject(double u, double v, double plane_z) const {
        const double dz = position.z - plane_z;
        return {position.x + (u - cx) * dz / fx, position.y + (v - cy) * dz / fy, plane_z};
    }
};

// The frame source. A synthetic scene camera today, a rendered or real one
// later — a detector above this seam cannot tell the difference.
class Camera {
public:
    virtual ~Camera() = default;
    [[nodiscard]] virtual const CameraModel& model() const = 0;
    virtual Frame grab() = 0;
};

}  // namespace robonode
