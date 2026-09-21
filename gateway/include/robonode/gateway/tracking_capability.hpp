#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "robonode/core/settings.hpp"
#include "robonode/gateway/capability.hpp"
#include "robonode/log.hpp"
#include "robonode/vision/sandboxed_tracker.hpp"
#include "robonode/vision/tracker.hpp"

namespace robonode {

inline CapabilityDescriptor tracking_descriptor() {
    return {"tracking",
            "Tracking / Prediction",
            "🎯",
            "inputs <code>x y z vx vy vz dt</code> (last sighting, measured velocity, lead time) "
            "→ three lines = where the part will be",
            "x + vx * dt\ny + vy * dt\nz + vz * dt",
            /*authorable=*/true,
            {"x", "y", "z", "vx", "vy", "vz", "dt"},
            3};
}

// Where the workpiece will be. Separated from vision on purpose: detecting a
// part and predicting its future are different problems, and a user should be
// able to replace either without touching the other.
class TrackingCapability final : public Capability<Tracker, TrackerContext> {
public:
    explicit TrackingCapability(Settings::Tracking cfg = {})
        : Capability{tracking_descriptor(), "robonode.constant-velocity"}, cfg_{cfg} {
        register_basic_trackers(registry_);
        remember({"robonode.snapshot", "builtin", {}});
        remember({"robonode.constant-velocity", "builtin", {}});
        remember({"robonode.smoothed", "builtin", {}});
        rebuild();
    }

    Status select(const std::string& version) override {
        if (!registry_.has(version)) return Status::failure("unknown version '" + version + "'");
        version_ = version;
        rebuild();
        return Status::success();
    }

    void install(const std::string& id, sandbox::Program program, Select select) override {
        register_program_tracker(registry_, id, std::move(program));
        remember({id, "user", {}});
        if (select == Select::kNow) version_ = id;
        rebuild();
    }

    void remember_source(const std::string& id, const std::string& source) {
        remember({id, "user", source_hash(source)});
    }

    // A new sighting. Cheap by contract: this runs on every telemetry frame.
    void observe(double t_s, Vec3 position) {
        if (tracker_) tracker_->observe({t_s, position});
    }

    [[nodiscard]] std::optional<Vec3> predict(double t_s) const {
        return tracker_ ? tracker_->predict(t_s) : std::nullopt;
    }

    [[nodiscard]] Vec3 velocity() const { return tracker_ ? tracker_->velocity() : Vec3{}; }

    void rebuild() {
        TrackerContext ctx;
        ctx.window = cfg_.window;
        ctx.gate_m = cfg_.gate_m;
        ctx.min_span_s = cfg_.min_span_s;
        ctx.smoothing = cfg_.smoothing;
        if (!registry_.make(version_, ctx, tracker_).ok()) {
            RN_LOG_WARN("tracking version '{}' unavailable", version_);
        }
        publish();
    }

    void publish_state() {
        const Vec3 v = velocity();
        publish({{"velocity", {v.x, v.y, v.z}}, {"speed", v.norm()}});
    }

private:
    std::unique_ptr<Tracker> tracker_;
    Settings::Tracking cfg_;
};

}  // namespace robonode
