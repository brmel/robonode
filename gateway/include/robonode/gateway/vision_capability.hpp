#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/geometry.hpp"
#include "robonode/gateway/capability.hpp"
#include "robonode/log.hpp"
#include "robonode/sandbox/program.hpp"
#include "robonode/vision/sandboxed_detector.hpp"
#include "robonode/vision/toy_detector.hpp"

namespace robonode {

inline CapabilityDescriptor vision_descriptor() {
    return {"vision",
            "Vision / Tracking",
            "👁",
            "inputs <code>x y z</code> (observed part pose) → three lines = output pose",
            "x\ny\nz + 0.05",
            /*authorable=*/true,
            {"x", "y", "z"},
            3};
}

class VisionCapability final : public Capability<Detector, VisionContext> {
public:
    VisionCapability() : Capability{vision_descriptor(), "robonode.toy-detector"} {
        register_toy_detector(registry_);
        remember({"robonode.toy-detector", "builtin", {}});
    }

    void set_site_pose(std::function<Vec3(const std::string&)> site_pose) {
        site_pose_ = std::move(site_pose);
    }

    // Perception engines the app links (OpenCV, ONNX) register their versions
    // here; the capability neither names nor links them.
    void add_versions(const std::function<void(VisionRegistry&)>& add) {
        add(registry_);
        for (const auto& name : registry_.names()) {
            if (name.rfind("robonode.", 0) == 0) remember({name, "builtin", {}});
        }
    }

    Status select(const std::string& version) override {
        if (!registry_.has(version)) return Status::failure("unknown version '" + version + "'");
        version_ = version;
        rebuild();
        observe();
        return Status::success();
    }

    void install(const std::string& id, sandbox::Program program, Select select) override {
        register_program_detector(registry_, id, std::move(program));
        remember({id, "user", {}});
        if (select == Select::kNow) version_ = id;
        rebuild();
        observe();
    }

    void remember_source(const std::string& id, const std::string& source) {
        remember({id, "user", source_hash(source)});
    }

    // The model frame this capability tracks — a viewer needs it to know which
    // body the platform reports dynamically.
    [[nodiscard]] const std::string& target_site() const { return target_site_; }

    void rebuild() {
        VisionContext ctx;
        ctx.site_pose = site_pose_;
        ctx.clock = clock_;
        ctx.camera = camera_;
        ctx.config = {{"target", target_site_}, {"clutter", clutter_}};
        if (!registry_.make(version_, ctx, detector_).ok()) {
            RN_LOG_WARN("vision version '{}' unavailable", version_);
        }
    }

    void set_clock(std::function<double()> clock) { clock_ = std::move(clock); }

    // Where the detector's frames come from. Supplied, never constructed: the
    // sensor is its own swappable node.
    void set_camera_source(std::function<std::unique_ptr<Camera>()> camera) {
        camera_ = std::move(camera);
    }

    // Nothing detected returns nothing — a caller must never receive a
    // default-constructed pose that reads as the world origin.
    std::optional<Vec3> observe() {
        const auto sighting = look();
        return sighting ? std::optional<Vec3>{sighting->position} : std::nullopt;
    }

    // The full sighting, including WHEN the frame was captured. A caller on a
    // moving line needs that: acting on a stale pose as if it were current is
    // the whole latency problem.
    std::optional<Detection> look() {
        const auto ds = detector_ ? detector_->detect() : std::vector<Detection>{};
        publish_detections(ds);
        if (ds.empty()) return std::nullopt;
        return ds.front();
    }

    // Where the last detection was, for a surface that wants to draw it. NOT a
    // fresh detect: running the detector to answer a reader would change what
    // the cell is doing in order to describe what it did.
    [[nodiscard]] std::optional<Vec3> last_seen() const {
        std::lock_guard<std::mutex> lk{seen_mtx_};
        return last_seen_;
    }

    // Sites the cell declares as visible clutter — same colour, wrong size.
    void set_clutter(std::string sites) { clutter_ = std::move(sites); }

private:
    void publish_detections(const std::vector<Detection>& ds) {
        auto arr = nlohmann::json::array();
        for (const auto& d : ds) {
            arr.push_back({{"label", d.label},
                           {"pos", {d.position.x, d.position.y, d.position.z}},
                           {"captured_s", d.captured_s},
                           {"confidence", d.confidence}});
        }
        const Vec3 part = ds.empty() ? Vec3{} : ds.front().position;
        {
            std::lock_guard<std::mutex> lk{seen_mtx_};
            last_seen_ = ds.empty() ? std::nullopt : std::optional<Vec3>{part};
        }
        publish({{"detections", arr},
                 {"part", {part.x, part.y, part.z}},
                 {"detected", !ds.empty()},
                 {"captured_s", ds.empty() ? 0.0 : ds.front().captured_s},
                 {"confidence", ds.empty() ? 0.0 : ds.front().confidence}});
    }

    mutable std::mutex seen_mtx_;
    std::optional<Vec3> last_seen_;
    std::unique_ptr<Detector> detector_;
    std::function<Vec3(const std::string&)> site_pose_;
    std::function<double()> clock_;
    std::function<std::unique_ptr<Camera>()> camera_;
    std::string target_site_{"part"};
    std::string clutter_;
};

}  // namespace robonode
