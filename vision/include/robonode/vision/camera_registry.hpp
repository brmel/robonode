#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "robonode/motion/module_registry.hpp"
#include "robonode/vision/camera.hpp"

namespace robonode {

// What a camera is built with: where it looks in the scene, the clock its
// frames are stamped with, and whatever the version needs (resolution, rate,
// latency, which sites are in view).
struct CameraContext {
    std::function<Vec3(const std::string&)> site_pose;
    std::function<double()> clock;
    std::map<std::string, std::string> config;
};

// Cameras are versions like everything else: an overhead sensor, a wrist
// camera, a rendered view, a real device. A detector receives one — it never
// builds one — so the sensor and the algorithm that reads it swap separately.
using CameraRegistry = ModuleRegistry<Camera, CameraContext>;

}  // namespace robonode
