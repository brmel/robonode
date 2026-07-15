#pragma once

#include <memory>
#include <string>

#include "robonode/motion/axis_adapter.hpp"
#include "robonode/sim_mujoco/mujoco_world.hpp"

namespace robonode {

// A single MotionAxis backed by one MuJoCo joint + actuator. Implements the
// same AxisAdapter seam as SimAxis and UrWristAdapter, so the executive,
// governor, and celld are unchanged — only the physics underneath is real
// (inertia, gravity, actuator dynamics ⇒ a genuine, rate-dependent following
// error).
//
// Units: the descriptor speaks its own units (mm for a linear rail, rad for
// a joint); MuJoCo speaks SI (m, rad). `units_per_m` is the descriptor-unit-
// per-metre scale (1000 for mm, 1 for rad/SI). The adapter is the unit
// boundary; everything above it stays in descriptor units.
//
// The adapter never steps physics itself: it exposes the shared world via
// shared_world(), and the executive ticks that world once per cycle (deduped
// across all adapters sharing it). SyncExecutive's write→step→read phasing
// guarantees all setpoints are in place before that single tick — and because
// stepping is not tied to any adapter's identity, swapping ANY node (even the
// rail) leaves the world advancing (#50).
class MujocoAxisAdapter final : public AxisAdapter {
public:
    MujocoAxisAdapter(std::string name, std::shared_ptr<MujocoWorld> world, std::string joint,
                      std::string actuator, double units_per_m)
        : name_{std::move(name)}, world_{std::move(world)}, joint_{std::move(joint)},
          actuator_{std::move(actuator)}, scale_{units_per_m} {}

    Status configure() override {
        const int jid = world_->joint_id(joint_);
        act_ = world_->actuator_id(actuator_);
        if (jid < 0 || act_ < 0) {
            set_lifecycle(Lifecycle::kFault);
            return Status::failure(name_ + ": joint '" + joint_ + "' or actuator '" + actuator_ +
                                   "' not found in world");
        }
        qadr_ = world_->qpos_adr(jid);
        vadr_ = world_->qvel_adr(jid);
        set_lifecycle(Lifecycle::kInactive);
        return Status::success();
    }

    void write_setpoint(double position_units) noexcept override {
        if (act_ >= 0) world_->set_ctrl(act_, position_units / scale_);
    }

    // No-op: the executive ticks the shared world (see shared_world).
    void step(double /*dt_s*/) noexcept override {}

    [[nodiscard]] CycleSteppable* shared_world() const noexcept override { return world_.get(); }

    // Live read straight from mjData — no cached state, so telemetry is fresh
    // and order-independent within a cycle.
    [[nodiscard]] AxisState read() const noexcept override {
        AxisState s;
        if (qadr_ >= 0) {
            s.position_mm = world_->qpos(qadr_) * scale_;
            s.velocity_mm_s = world_->qvel(vadr_) * scale_;
        }
        s.safety = SafetyState::kNormal;  // sim: certified chain modelled later
        return s;
    }

    [[nodiscard]] std::string name() const override { return name_; }

private:
    std::string name_;
    std::shared_ptr<MujocoWorld> world_;
    std::string joint_;
    std::string actuator_;
    double scale_;
    int act_{-1};
    int qadr_{-1};
    int vadr_{-1};
};

}  // namespace robonode
