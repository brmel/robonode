#pragma once

#include <memory>
#include <string>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/core/status.hpp"
#include "robonode/motion/kinematics.hpp"
#include "robonode/sim_mujoco/mujoco_world.hpp"

namespace robonode {

// MuJoCo-backed Kinematics: FK and the position Jacobian for a set of joints
// driving a TCP site, computed on a private scratch world (loaded from the
// same MJCF) so queries never disturb the live physics sim. The model is the
// reference — #4's tests cross-check IK convergence against this FK.
//
// Controls the listed joints (e.g. the 6 arm joints); other DOF (the rail)
// stay at their scratch-world defaults, so FK is relative to that base.
class MujocoKinematics final : public Kinematics {
public:
    static Status create(const std::string& xml_path, const std::vector<std::string>& joints,
                         const std::string& tcp_site, std::unique_ptr<MujocoKinematics>& out) {
        std::shared_ptr<MujocoWorld> world;
        if (const auto st = MujocoWorld::load(xml_path, world); !st.ok()) return st;
        std::unique_ptr<MujocoKinematics> k{new MujocoKinematics(std::move(world))};
        for (const auto& jn : joints) {
            const int jid = k->world_->joint_id(jn);
            if (jid < 0) return Status::failure("kinematics: joint '" + jn + "' not found");
            k->qadr_.push_back(k->world_->qpos_adr(jid));
            k->vadr_.push_back(k->world_->qvel_adr(jid));  // dof (velocity) address
        }
        k->tcp_ = k->world_->site_id(tcp_site);
        if (k->tcp_ < 0) return Status::failure("kinematics: site '" + tcp_site + "' not found");
        k->nv_ = k->world_->nv();
        out = std::move(k);
        return Status::success();
    }

    [[nodiscard]] std::size_t dof() const override { return qadr_.size(); }

    // World pose of a named site (e.g. a target the vision node reports).
    [[nodiscard]] Vec3 site_position(const std::string& name) const {
        world_->forward();
        return world_->site_xpos(world_->site_id(name));
    }

    [[nodiscard]] Vec3 tcp_position(const std::vector<double>& q) const override {
        write_and_forward(q);
        return world_->site_xpos(tcp_);
    }

    [[nodiscard]] std::vector<double> position_jacobian(
        const std::vector<double>& q) const override {
        write_and_forward(q);
        std::vector<double> jacp(static_cast<std::size_t>(3 * nv_));
        world_->jac_site(tcp_, jacp.data());
        const std::size_t n = qadr_.size();
        std::vector<double> J(3 * n);
        for (int r = 0; r < 3; ++r) {
            for (std::size_t k = 0; k < n; ++k) {
                J[r * n + k] = jacp[static_cast<std::size_t>(r * nv_) + vadr_[k]];
            }
        }
        return J;
    }

private:
    explicit MujocoKinematics(std::shared_ptr<MujocoWorld> w) : world_{std::move(w)} {}

    void write_and_forward(const std::vector<double>& q) const {
        for (std::size_t k = 0; k < qadr_.size() && k < q.size(); ++k) {
            world_->set_qpos(qadr_[k], q[k]);
        }
        world_->forward();
    }

    std::shared_ptr<MujocoWorld> world_;  // scratch world for kinematic queries
    std::vector<int> qadr_;
    std::vector<int> vadr_;
    int tcp_{-1};
    int nv_{0};
};

}  // namespace robonode
