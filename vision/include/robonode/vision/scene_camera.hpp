#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "robonode/vision/camera_registry.hpp"

namespace robonode {

// A camera over the live scene, with the properties that make machine vision
// hard on a real line:
//
//   * **latency** — a frame is exposed, transferred and processed, so what a
//     detector sees is always the past. On a belt that is the difference
//     between a grasp and a miss.
//   * **noise** — the sensor is not clean, so a threshold has to survive it.
//   * **clutter** — the workpiece is not the only thing in view; something the
//     same colour will fool "find the biggest blob".
//
// It projects the scene rather than rendering it, so it stands in for an
// offscreen render. The seam is the durable part: a real renderer plugs in here
// and every detector above keeps working.
// Sensor characteristics, at namespace scope so it can carry defaults and still
// be a default argument.
struct CameraOptions {
    int width{320};
    int height{240};
    double noise{6.0};
    double latency_s{0.12};  // exposure + transfer + processing
    int fps{20};
};

class SceneCamera final : public Camera {
public:
    using SitePose = std::function<Vec3(const std::string&)>;
    using Clock = std::function<double()>;

    struct Target {
        std::string site;
        std::uint8_t b{60}, g{60}, r{225};  // BGR — a red workpiece
        double size_m{0.06};
    };

    SceneCamera(CameraModel model, SitePose site_pose, Clock clock, std::vector<Target> targets,
                CameraOptions options = {})
        : model_{model}, site_pose_{std::move(site_pose)}, clock_{std::move(clock)},
          targets_{std::move(targets)}, opt_{options} {}

    [[nodiscard]] const CameraModel& model() const override { return model_; }

    Frame grab() override {
        const double now = clock_ ? clock_() : 0.0;
        capture(now);
        return deliver(now);
    }

private:
    // Expose a frame if the shutter is due.
    void capture(double now) {
        const double period = opt_.fps > 0 ? 1.0 / opt_.fps : 0.0;
        if (!pending_.empty() && now - pending_.back().captured_s < period) return;

        Frame f{opt_.width, opt_.height, now,
                std::vector<std::uint8_t>(static_cast<std::size_t>(opt_.width * opt_.height * 3))};
        paint_background(f);
        for (const auto& t : targets_) paint_target(f, t);
        pending_.push_back(std::move(f));
        while (pending_.size() > 64) pending_.pop_front();
    }

    // Hand back the newest frame old enough to have been processed.
    Frame deliver(double now) {
        Frame chosen;
        for (const auto& f : pending_) {
            if (now - f.captured_s >= opt_.latency_s) chosen = f;
        }
        return chosen;
    }

    void paint_background(Frame& f) {
        std::normal_distribution<double> noise{0.0, opt_.noise};
        for (std::size_t i = 0; i < f.bgr.size(); i += 3) {
            const double base = 48.0 + noise(rng_);
            const auto v = static_cast<std::uint8_t>(std::clamp(base, 0.0, 255.0));
            f.bgr[i] = v;
            f.bgr[i + 1] = v;
            f.bgr[i + 2] = static_cast<std::uint8_t>(std::clamp(base * 0.9, 0.0, 255.0));
        }
    }

    void paint_target(Frame& f, const Target& t) {
        const Vec3 world = site_pose_ ? site_pose_(t.site) : Vec3{};
        const auto uv = model_.project(world);
        if (!uv) return;
        const double dz = std::max(1e-3, model_.position.z - world.z);
        const int half = static_cast<int>(0.5 * t.size_m * model_.fx / dz);
        const int cu = static_cast<int>(uv->first), cv = static_cast<int>(uv->second);
        for (int y = cv - half; y <= cv + half; ++y) {
            if (y < 0 || y >= f.height) continue;
            for (int x = cu - half; x <= cu + half; ++x) {
                if (x < 0 || x >= f.width) continue;
                const auto i = static_cast<std::size_t>((y * f.width + x) * 3);
                f.bgr[i] = t.b;
                f.bgr[i + 1] = t.g;
                f.bgr[i + 2] = t.r;
            }
        }
    }

    CameraModel model_;
    SitePose site_pose_;
    Clock clock_;
    std::vector<Target> targets_;
    CameraOptions opt_;
    std::deque<Frame> pending_;
    std::mt19937 rng_{1234};
};

// Registers the scene cameras this build offers. Sites in view, resolution,
// rate and latency all come from the context, so a differently-specified sensor
// is a different registry entry — not a different detector.
inline void register_scene_cameras(CameraRegistry& reg) {
    const auto build = [](const CameraContext& c, CameraOptions opt) -> std::unique_ptr<Camera> {
        CameraModel model;
        std::vector<SceneCamera::Target> targets;
        const auto watch = c.config.find("watch");
        if (watch != c.config.end()) {
            std::stringstream names{watch->second};
            for (std::string site; std::getline(names, site, ',');) {
                if (!site.empty()) targets.push_back({site});
            }
        }
        // Clutter shares the workpiece's colour and is deliberately the wrong size.
        const auto clutter = c.config.find("clutter");
        if (clutter != c.config.end()) {
            std::stringstream names{clutter->second};
            for (std::string site; std::getline(names, site, ',');) {
                if (!site.empty()) targets.push_back({site, 60, 60, 225, 0.10});
            }
        }
        return std::make_unique<SceneCamera>(model, c.site_pose, c.clock, std::move(targets), opt);
    };

    reg.add("robonode.overhead", [build](const CameraContext& c) { return build(c, {}); });
    // The same sensor, run harder: half the frame rate and three times the
    // latency. Nothing about the detector changes — but the tracker has much
    // less to work with, which is the point of being able to swap it.
    reg.add("robonode.overhead-slow", [build](const CameraContext& c) {
        CameraOptions opt;
        opt.fps = 8;
        opt.latency_s = 0.35;
        opt.noise = 10.0;
        return build(c, opt);
    });
}

}  // namespace robonode
