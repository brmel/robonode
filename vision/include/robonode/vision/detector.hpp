#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/motion/module_registry.hpp"

namespace robonode {

// One thing the detector found in the scene. Orientation + confidence join
// later; v0 is a labelled world position (metres).
struct Detection {
    std::string label;
    Vec3 position;
};

// What a detector is built with (ADR-11). v0 hands it a scene oracle — the
// ground-truth pose of a named site, the toy detector's stand-in for a camera.
// A real detector (#38) is built with a frame instead; nothing above this seam
// changes. `config` carries version-specific strings (the target to find, …).
struct VisionContext {
    std::function<Vec3(const std::string&)> site_pose;
    std::map<std::string, std::string> config;
};

// The vision capability seam: observe the scene, report detections. Product
// code depends on this interface, never on a concrete detector version.
class Detector {
public:
    virtual ~Detector() = default;
    virtual std::vector<Detection> detect() = 0;
};

using VisionRegistry = ModuleRegistry<Detector, VisionContext>;

}  // namespace robonode
