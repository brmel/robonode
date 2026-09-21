#pragma once

#include <memory>
#include <string>
#include <utility>

#include "robonode/gateway/capability.hpp"
#include "robonode/log.hpp"
#include "robonode/vision/camera_registry.hpp"
#include "robonode/vision/scene_camera.hpp"

namespace robonode {

inline CapabilityDescriptor camera_descriptor() {
    return {"camera", "Camera", "📷",
            "the sensor vision reads — resolution, rate, latency and what is in view",
            {}, /*authorable=*/false, {}, 0};
}

// The sensor, as a swappable node in its own right. A detector is handed a
// camera rather than building one, so "change the camera" and "change the
// vision algorithm" are two independent choices — which is the difference
// between a pipeline and a monolith.
//
// Not authorable: a camera is hardware (or a model of it), not an algorithm.
// Bring a new one by registering a version, the way a vendor driver arrives.
class CameraCapability final : public Capability<Camera, CameraContext> {
public:
    CameraCapability() : Capability{camera_descriptor(), "robonode.overhead"} {
        register_scene_cameras(registry_);
        remember({"robonode.overhead", "builtin", {}});
        remember({"robonode.overhead-slow", "builtin", {}});
        publish();
    }

    void set_scene(std::function<Vec3(const std::string&)> site_pose,
                   std::function<double()> clock) {
        site_pose_ = std::move(site_pose);
        clock_ = std::move(clock);
    }

    // What the sensor has in view, and what of that is clutter. Cell data.
    void set_view(std::string watch, std::string clutter) {
        watch_ = std::move(watch);
        clutter_ = std::move(clutter);
    }

    Status select(const std::string& version) override {
        if (!registry_.has(version)) return Status::failure("unknown version '" + version + "'");
        version_ = version;
        publish();
        return Status::success();
    }

    void install(const std::string&, sandbox::Program, Select) override {
        RN_LOG_WARN("a camera is hardware, not an algorithm — register a version instead");
    }

    // Built fresh for whoever reads it: a detector owns its sensor for as long
    // as it lives, and swapping either rebuilds the pair.
    std::unique_ptr<Camera> make() {
        CameraContext ctx;
        ctx.site_pose = site_pose_;
        ctx.clock = clock_;
        ctx.config = {{"watch", watch_}, {"clutter", clutter_}};
        std::unique_ptr<Camera> camera;
        if (!registry_.make(version_, ctx, camera).ok()) {
            RN_LOG_WARN("camera version '{}' unavailable", version_);
        }
        return camera;
    }

private:
    std::function<Vec3(const std::string&)> site_pose_;
    std::function<double()> clock_;
    std::string watch_{"part"}, clutter_;
};

}  // namespace robonode
