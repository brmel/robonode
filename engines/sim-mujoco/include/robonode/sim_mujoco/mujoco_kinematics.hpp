#pragma once

#include <memory>
#include <mutex>
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
// The scratch world is an mjData, and mjData is single-owner: every query
// writes joint positions into it and runs a forward pass. Planning does that on
// the worker thread while telemetry asks where the tool points on another, so
// this class serialises access to its own world. The lock is the object's
// invariant, not the caller's problem — a seam that needs a note in its docs to
// be used safely will eventually be used unsafely.
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
        // A world at rest is already touching itself in places a plan cannot
        // help (a part resting on the belt). Only contacts ABOVE that baseline
        // are the arm running into something.
        k->world_->forward();
        k->baseline_contacts_ = k->world_->contact_count();
        out = std::move(k);
        return Status::success();
    }

    [[nodiscard]] std::size_t dof() const override { return qadr_.size(); }

    // World pose of a named site (e.g. a target the vision node reports).
    [[nodiscard]] Vec3 site_position(const std::string& name) const {
        std::lock_guard<std::mutex> lk{mtx_};
        world_->forward();
        return world_->site_xpos(world_->site_id(name));
    }

    [[nodiscard]] Vec3 tcp_position(const std::vector<double>& q) const override {
        std::lock_guard<std::mutex> lk{mtx_};
        write_and_forward(q);
        return world_->site_xpos(tcp_);
    }

    [[nodiscard]] std::vector<double> position_jacobian(
        const std::vector<double>& q) const override {
        std::lock_guard<std::mutex> lk{mtx_};
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

    // The model IS the reference for orientation too: the site's own rotation,
    // not an assumption about how the tool is mounted.
    [[nodiscard]] Pose tcp_pose(const std::vector<double>& q) const override {
        std::lock_guard<std::mutex> lk{mtx_};
        write_and_forward(q);
        return {world_->site_xpos(tcp_), world_->site_xquat(tcp_)};
    }

    // Geometric Jacobian, 6×dof: linear rows then angular. Same columns as the
    // position half, so a solver can use three rows or all six.
    [[nodiscard]] std::vector<double> jacobian(const std::vector<double>& q) const override {
        std::lock_guard<std::mutex> lk{mtx_};
        write_and_forward(q);
        const auto nv = static_cast<std::size_t>(nv_);
        std::vector<double> jacp(3 * nv), jacr(3 * nv);
        world_->jac_site(tcp_, jacp.data(), jacr.data());
        const std::size_t n = qadr_.size();
        std::vector<double> J(6 * n);
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t k = 0; k < n; ++k) {
                J[r * n + k] = jacp[r * nv + static_cast<std::size_t>(vadr_[k])];
                J[(r + 3) * n + k] = jacr[r * nv + static_cast<std::size_t>(vadr_[k])];
            }
        }
        return J;
    }

    // The scratch world is a full copy of the model, so asking "would this
    // configuration be touching something?" is a forward pass and a count —
    // no separate collision library, and no second geometry to keep in step.
    [[nodiscard]] bool can_check_collision() const override { return true; }

    [[nodiscard]] bool in_collision(const std::vector<double>& q) const override {
        std::lock_guard<std::mutex> lk{mtx_};
        write_and_forward(q);
        return world_->contact_count() > baseline_contacts_;
    }

private:
    explicit MujocoKinematics(std::shared_ptr<MujocoWorld> w) : world_{std::move(w)} {}

    void write_and_forward(const std::vector<double>& q) const {
        for (std::size_t k = 0; k < qadr_.size() && k < q.size(); ++k) {
            world_->set_qpos(qadr_[k], q[k]);
        }
        world_->forward();
    }

    mutable std::mutex mtx_;              // the scratch world has ONE owner at a time
    std::shared_ptr<MujocoWorld> world_;  // scratch world for kinematic queries
    std::vector<int> qadr_;
    std::vector<int> vadr_;
    int tcp_{-1};
    int nv_{0};
    int baseline_contacts_{0};
};

}  // namespace robonode
