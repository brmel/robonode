#pragma once

#include <map>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "robonode/core/status.hpp"

namespace robonode {

// Owns one MuJoCo model + data — the physics substrate a cell's axis/joint
// adapters read and drive. One world may back many adapters (an arm's
// joints share it); it is stepped exactly once per control cycle by the
// designated clock-owner adapter (SyncExecutive phases write→step→read so
// every ctrl is set before physics advances).
//
// mujoco.h is private to this module (boundary lint): nothing above
// sim-mujoco sees a MuJoCo type.
class MujocoWorld {
public:
    // Loads an MJCF world from disk. Failure Status carries MuJoCo's parser
    // error verbatim.
    static Status load(const std::string& xml_path, std::shared_ptr<MujocoWorld>& out) {
        char err[1024] = {0};
        mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
        if (m == nullptr) {
            return Status::failure("mujoco load '" + xml_path + "': " + err);
        }
        out = std::shared_ptr<MujocoWorld>(new MujocoWorld(m));
        return Status::success();
    }

    ~MujocoWorld() {
        if (data_ != nullptr) mj_deleteData(data_);
        if (model_ != nullptr) mj_deleteModel(model_);
    }
    MujocoWorld(const MujocoWorld&) = delete;
    MujocoWorld& operator=(const MujocoWorld&) = delete;

    // Name lookups return -1 when absent (adapters fault-check in configure).
    [[nodiscard]] int joint_id(const std::string& name) const {
        return mj_name2id(model_, mjOBJ_JOINT, name.c_str());
    }
    [[nodiscard]] int actuator_id(const std::string& name) const {
        return mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());
    }
    [[nodiscard]] int qpos_adr(int joint_id) const { return model_->jnt_qposadr[joint_id]; }
    [[nodiscard]] int qvel_adr(int joint_id) const { return model_->jnt_dofadr[joint_id]; }

    void set_ctrl(int actuator, double value) noexcept { data_->ctrl[actuator] = value; }
    [[nodiscard]] double qpos(int adr) const noexcept { return data_->qpos[adr]; }
    [[nodiscard]] double qvel(int adr) const noexcept { return data_->qvel[adr]; }

    // Advance physics by dt_s, accumulating a residual so sim time tracks the
    // control clock regardless of the mj timestep / cycle-rate ratio.
    void step(double dt_s) noexcept {
        accum_ += dt_s;
        const double ts = model_->opt.timestep;
        while (accum_ >= ts - 1e-12) {
            mj_step(model_, data_);
            accum_ -= ts;
        }
    }

    [[nodiscard]] double timestep() const noexcept { return model_->opt.timestep; }

private:
    explicit MujocoWorld(mjModel* m) : model_{m}, data_{mj_makeData(m)} {}
    mjModel* model_;
    mjData* data_;
    double accum_{0.0};
};

// Shares one MujocoWorld across every adapter that names the same MJCF path,
// so an arm's joints (7 nodes, one `world` path) drive a single physics
// body. The first adapter to request a path is its clock owner (the one that
// steps physics); the rest only marshal ctrl/state. Loaded lazily, once.
class MujocoWorldPool {
public:
    Status get(const std::string& path, std::shared_ptr<MujocoWorld>& out, bool& is_clock_owner) {
        const auto it = worlds_.find(path);
        if (it != worlds_.end()) {
            out = it->second;
            is_clock_owner = false;
            return Status::success();
        }
        std::shared_ptr<MujocoWorld> w;
        if (const auto st = MujocoWorld::load(path, w); !st.ok()) return st;
        worlds_[path] = w;
        out = w;
        is_clock_owner = true;
        return Status::success();
    }

private:
    std::map<std::string, std::shared_ptr<MujocoWorld>> worlds_;
};

}  // namespace robonode
