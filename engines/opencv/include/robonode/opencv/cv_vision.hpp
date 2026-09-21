#pragma once

#include <memory>
#include <string>

#include "robonode/opencv/cv_detector.hpp"
#include "robonode/vision/detector.hpp"
#include "robonode/vision/camera.hpp"

namespace robonode {

// Registers "robonode.opencv" as a vision version: a camera over the scene plus
// a real OpenCV pipeline. Selecting it swaps the platform from a scene oracle
// to actual image processing, with nothing above the Detector seam changing —
// which is the claim the seam exists to make good on.
inline void register_cv_detector(VisionRegistry& registry) {
    registry.add("robonode.opencv", [](const VisionContext& ctx) -> std::unique_ptr<Detector> {
        const auto target = ctx.config.find("target");
        const std::string label = target == ctx.config.end() ? "part" : target->second;
        if (!ctx.camera) return nullptr;  // no sensor selected: say so, do not invent one
        return std::make_unique<CvDetector>(ctx.camera(), CvDetector::Tuning{}, label);
    });
}

}  // namespace robonode
