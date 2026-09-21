#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "robonode/celld/scene.hpp"
#include "robonode/core/status.hpp"
#include "robonode/core/geometry.hpp"

namespace robonode {

// The cell's live world, and the last snapshot taken of it.
//
// Two threads stepping the same physics data is a segfault, not a race you get
// away with (#103: the gripper and the conveyor touched the world while an idle
// tick stepped it). So there is exactly one way in — `with(fn)`, which runs the
// caller's work under the cell lock — and readers never get the world at all.
// They get the snapshot, taken while that lock is still held, which is why a
// reader can never observe the gap between a step and the sample of it.
//
// The lock and the world path belong to the cell, so both arrive as callbacks:
// this type owns the scene and the snapshot, and nothing else.
class SceneView {
public:
    using UnderCellLock = std::function<Status(const std::function<Status()>&)>;
    using WorldPath = std::function<std::string()>;
    using OpenScene = std::function<std::unique_ptr<Scene>(const std::string& world)>;

    SceneView(UnderCellLock locked, WorldPath world, OpenScene open)
        : locked_{std::move(locked)}, world_{std::move(world)}, open_{std::move(open)} {}

    // Work on the live world, under the cell lock, with the snapshot refreshed
    // before the lock is released: whatever `fn` moved, readers see moved.
    Status with(const std::function<Status(Scene&)>& fn) {
        return locked_([&] {
            auto* live = scene();
            if (live == nullptr) return Status::failure("no scene in this cell");
            const auto st = fn(*live);
            sample();
            return st;
        });
    }

    // Let time pass. Advancing and re-sampling are one locked step on purpose.
    Status advance(double dt_s) {
        return with([dt_s](Scene& live) {
            live.advance(dt_s);
            return Status::success();
        });
    }

    // Take a fresh snapshot without changing anything — for the moments when
    // the world moved underneath us (a rebuild, a boot) rather than because of
    // us.
    void resample() {
        (void)locked_([this] {
            sample();
            return Status::success();
        });
    }

    // Sample WITHOUT taking the cell lock. Two callers need this and both would
    // be wrong to use resample(): the cycle thread already holds the lock and
    // would deadlock on it, and a cell still booting has no cell to lock — so
    // resample() there fails closed and leaves every detector reading an
    // unsampled world. The distinction is in the name rather than left to be
    // remembered.
    void sample_now() { sample(); }

    // The scene is a different model now (a scene swap): drop it so the next
    // caller opens the new one. The cell lock is the caller's to hold.
    void invalidate() { scene_.reset(); }

    [[nodiscard]] Vec3 site(const std::string& name) const {
        std::lock_guard<std::mutex> lk{mtx_};
        for (const auto& [id, pos] : sites_) {
            if (id == name) return pos;
        }
        return {};
    }

    [[nodiscard]] std::vector<std::pair<std::string, std::string>> contacts() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return contacts_;
    }

private:
    // Called with the cell lock held, by everything above.
    void sample() {
        auto* live = scene();
        if (live == nullptr) return;
        auto sites = live->site_poses();
        auto touching = live->contacts();
        std::lock_guard<std::mutex> lk{mtx_};
        sites_ = std::move(sites);
        contacts_ = std::move(touching);
    }

    Scene* scene() {
        if (!scene_) scene_ = open_(world_());
        return scene_.get();
    }

    UnderCellLock locked_;
    WorldPath world_;
    OpenScene open_;
    std::unique_ptr<Scene> scene_;

    mutable std::mutex mtx_;  // guards the snapshot, which request threads read
    std::vector<std::pair<std::string, Vec3>> sites_;
    std::vector<std::pair<std::string, std::string>> contacts_;
};

}  // namespace robonode
