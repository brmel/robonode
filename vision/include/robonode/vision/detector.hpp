#pragma once

#include <functional>
#include <memory>
#include <map>
#include <string>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/motion/module_registry.hpp"
#include "robonode/vision/camera.hpp"

namespace robonode {

// One thing the detector found, and WHEN the frame it came from was captured.
// A sighting without a capture time is unusable on a moving line: by the time
// an image is processed the world has moved on, and only the timestamp says
// by how much.
struct Detection {
    std::string label;
    Vec3 position;
    double captured_s{0.0};  // scene clock when the frame was taken, not when it was processed
    double confidence{1.0};
};

// What a detector is built with (ADR-11). v0 hands it a scene oracle — the
// ground-truth pose of a named site, the toy detector's stand-in for a camera.
// A real detector (#38) is built with a frame instead; nothing above this seam
// changes. `config` carries version-specific strings (the target to find, …).
struct VisionContext {
    std::function<Vec3(const std::string&)> site_pose;
    std::function<double()> clock;  // the scene clock a frame is stamped with
    // The sensor this detector reads. Built by the camera capability, so the
    // camera and the algorithm that interprets it are chosen independently.
    std::function<std::unique_ptr<Camera>()> camera;
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
