#pragma once


#include <memory>
#include <string>
#include <utility>

#include "robonode/vision/detector.hpp"

namespace robonode {

// Basic vision versions: report the target part's ground-truth pose from the
// scene oracle (no camera, no CV), plus a fixed offset that stands in for a
// different algorithm's answer. The interchangeable stand-in a real OpenCV/ONNX
// detector (#38) or a user's sandboxed algorithm replaces behind the same seam.
class ToyDetector final : public Detector {
public:
    ToyDetector(VisionContext ctx, Vec3 offset) : ctx_{std::move(ctx)}, offset_{offset} {
        const auto it = ctx_.config.find("target");
        target_ = it == ctx_.config.end() ? "part" : it->second;
    }

    std::vector<Detection> detect() override {
        if (!ctx_.site_pose) return {};
        const Vec3 p = ctx_.site_pose(target_);
        // A perfect sensor still reports WHEN it looked: every detector is
        // interchangeable, so they must all answer the same questions.
        const double now = ctx_.clock ? ctx_.clock() : 0.0;
        return {{target_, {p.x + offset_.x, p.y + offset_.y, p.z + offset_.z}, now, 1.0}};
    }

private:
    VisionContext ctx_;
    std::string target_;
    Vec3 offset_;
};

// Two basic versions so "try a different algorithm" is real out of the box:
// center-of-part vs a top-face grasp point. Swapping live moves the detection —
// and thus where "pick" goes.
inline void register_toy_detector(VisionRegistry& reg) {
    reg.add("robonode.toy-detector", [](const VisionContext& c) -> std::unique_ptr<Detector> {
        return std::make_unique<ToyDetector>(c, Vec3{0, 0, 0});
    });
    reg.add("robonode.toy-top-grasp", [](const VisionContext& c) -> std::unique_ptr<Detector> {
        return std::make_unique<ToyDetector>(c, Vec3{0, 0, 0.05});
    });
}

}  // namespace robonode
