#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "robonode/celld/scene.hpp"
#include "robonode/sim_mujoco/mujoco_world.hpp"

namespace robonode {

// The Scene seam over the LIVE physics world — the same one the axis adapters
// drive, not a scratch copy. That distinction is the whole point: a sensor
// reading this sees the workpiece actually moving on the belt.
class MujocoScene final : public Scene {
public:
    explicit MujocoScene(std::shared_ptr<MujocoWorld> world) : world_{std::move(world)} {}

    [[nodiscard]] std::optional<Vec3> site(const std::string& name) const override {
        const int id = world_->site_id(name);
        if (id < 0) return std::nullopt;
        world_->forward();
        return world_->site_xpos(id);
    }

    Status drive(const std::string& actuator, double value) override {
        const int id = world_->actuator_id(actuator);
        if (id < 0) return Status::failure("no actuator '" + actuator + "'");
        world_->set_ctrl(id, value);
        return Status::success();
    }

    Status place(const std::string& joint, double value) override {
        const int id = world_->joint_id(joint);
        if (id < 0) return Status::failure("no joint '" + joint + "'");
        world_->set_qpos(world_->qpos_adr(id), value);
        world_->forward();
        return Status::success();
    }

    Status place_body(const std::string& body, const Vec3& pos) override {
        return world_->place_body(body, pos);
    }

    Status constrain(const std::string& name, Grip grip) override {
        return world_->constrain(name, grip);
    }

    [[nodiscard]] std::vector<std::pair<std::string, Vec3>> site_poses() const override {
        // Frames are only valid once kinematics have been evaluated — a world
        // that has been reset but not yet stepped has none. Safe here: sampling
        // only ever runs on the thread that owns the physics.
        world_->forward();
        return world_->site_world_poses();
    }

    [[nodiscard]] std::vector<std::pair<std::string, std::string>> contacts() const override {
        return world_->contacts();
    }

    void advance(double dt_s) override { world_->tick(dt_s); }

private:
    std::shared_ptr<MujocoWorld> world_;
};

}  // namespace robonode
