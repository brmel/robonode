#pragma once

#include <map>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "robonode/core/geometry.hpp"
#include "robonode/core/status.hpp"
#include "robonode/motion/steppable.hpp"

namespace robonode {

// Owns one MuJoCo model + data — the physics substrate a cell's axis/joint
// adapters read and drive. One world may back many adapters (an arm's joints
// share it); it is a CycleSteppable the executive ticks exactly once per
// control cycle (SyncExecutive phases write→step→read so every ctrl is set
// before physics advances). Stepping is the executive's job, not any single
// adapter's — so swapping any node leaves the world advancing (#50).
//
// mujoco.h is private to this module (boundary lint): nothing above
// sim-mujoco sees a MuJoCo type.
class MujocoWorld : public CycleSteppable {
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
    // control clock regardless of the mj timestep / cycle-rate ratio. This is
    // the CycleSteppable hook the executive calls once per cycle.
    void tick(double dt_s) noexcept override {
        accum_ += dt_s;
        const double ts = model_->opt.timestep;
        while (accum_ >= ts - 1e-12) {
            mj_step(model_, data_);
            accum_ -= ts;
        }
    }

    [[nodiscard]] double timestep() const noexcept { return model_->opt.timestep; }

    // --- kinematics accessors (used by MujocoKinematics on a scratch world) ---
    [[nodiscard]] int nv() const noexcept { return model_->nv; }
    [[nodiscard]] int site_id(const std::string& name) const {
        return mj_name2id(model_, mjOBJ_SITE, name.c_str());
    }
    void set_qpos(int adr, double v) noexcept { data_->qpos[adr] = v; }
    void forward() noexcept { mj_forward(model_, data_); }  // FK + Jacobians, no integration
    [[nodiscard]] Vec3 site_xpos(int site) const noexcept {
        return {data_->site_xpos[3 * site], data_->site_xpos[3 * site + 1],
                data_->site_xpos[3 * site + 2]};
    }
    // Position Jacobian of a site: jacp is 3×nv, row-major (caller-sized).
    void jac_site(int site, double* jacp) noexcept {
        mj_jacSite(model_, data_, jacp, nullptr, site);
    }

private:
    // Wake in the model's "home" keyframe when it defines one — it sets qpos AND
    // ctrl to a natural ready pose, so the servos hold that posture at rest (a
    // real robot at rest, not a vertical mast) and moves start from it.
    explicit MujocoWorld(mjModel* m) : model_{m}, data_{mj_makeData(m)} {
        if (model_->nkey > 0) mj_resetDataKeyframe(model_, data_, 0);
    }
    mjModel* model_;
    mjData* data_;
    double accum_{0.0};
};

// Shares one MujocoWorld across every adapter that names the same MJCF path,
// so an arm's joints (7 nodes, one `world` path) drive a single physics body.
// Every adapter reads/drives the shared world; the executive ticks it once
// per cycle (no adapter "owns" the clock — #50). Loaded lazily, once.
class MujocoWorldPool {
public:
    Status get(const std::string& path, std::shared_ptr<MujocoWorld>& out) {
        const auto it = worlds_.find(path);
        if (it != worlds_.end()) {
            out = it->second;
            return Status::success();
        }
        std::shared_ptr<MujocoWorld> w;
        if (const auto st = MujocoWorld::load(path, w); !st.ok()) return st;
        worlds_[path] = w;
        out = w;
        return Status::success();
    }

private:
    std::map<std::string, std::shared_ptr<MujocoWorld>> worlds_;
};

}  // namespace robonode
