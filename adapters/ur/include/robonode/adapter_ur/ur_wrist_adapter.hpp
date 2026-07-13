#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <ur_client_library/ur/ur_driver.h>

#include "robonode/motion/axis_adapter.hpp"

namespace robonode {

// First real vendor adapter behind the AxisAdapter seam: one UR joint
// (wrist 3) driven in SERVOJ mode at 500 Hz through the same interface the
// sim axis implements — motion core, governor, and executive are unchanged
// (SPEC I5 in practice).
//
// Units: radians (AxisAdapter is unit-agnostic doubles; the capability
// descriptor carries units — FR-1.3). M1 scope: single joint; the full
// ArmKinematics adapter generalizes this to 6-DOF + Cartesian.
class UrWristAdapter final : public AxisAdapter {
public:
    // driver must outlive the adapter and already be connected (external
    // control script sent). The adapter never owns the connection: celld
    // will. configure() verifies the robot is actually readable.
    UrWristAdapter(urcl::UrDriver& driver, std::string name)
        : driver_{driver}, name_{std::move(name)},
          pkg_{driver.getRTDEOutputRecipe()} {}

    // Lifecycle (FR-1.2): configure = first state read; robot must be
    // streaming RTDE (powered, safety confirmed) to leave kUnconfigured.
    Status configure() override {
        if (!driver_.getDataPackage(pkg_)) {
            set_lifecycle(Lifecycle::kFault);
            return Status::failure(name_ + ": no RTDE data (robot powered? safety confirmed?)");
        }
        refresh_from_pkg();
        setpoint_rad_ = state_.position_mm;  // hold current pose until commanded
        target_q_ = q_;
        set_lifecycle(Lifecycle::kInactive);
        return Status::success();
    }

    void write_setpoint(double position_rad) noexcept override { setpoint_rad_ = position_rad; }

    [[nodiscard]] AxisState read() const noexcept override { return state_; }

    // One cycle: send the wrist setpoint (absolute — FR-2.10), read back
    // state. Any transport failure latches kFault; the executive's safety
    // gate then holds (and a real celld would fault the node).
    void step(double /*dt_s*/) noexcept override {
        try {
            target_q_ = q_;
            target_q_[5] = setpoint_rad_;
            if (!driver_.writeJointCommand(target_q_, urcl::comm::ControlMode::MODE_SERVOJ,
                                           urcl::RobotReceiveTimeout::millisec(20))) {
                state_.safety = SafetyState::kFault;
                return;
            }
            if (driver_.getDataPackage(pkg_)) refresh_from_pkg();
        } catch (...) {
            state_.safety = SafetyState::kFault;
        }
    }

    [[nodiscard]] std::string name() const override { return name_; }

    [[nodiscard]] const urcl::vector6d_t& joints() const noexcept { return q_; }

private:
    void refresh_from_pkg() {
        pkg_.getData("actual_q", q_);
        urcl::vector6d_t qd{};
        pkg_.getData("actual_qd", qd);
        std::int32_t safety_mode = 0;
        pkg_.getData("safety_mode", safety_mode);
        state_.position_mm = q_[5];
        state_.velocity_mm_s = qd[5];
        state_.safety = map_safety(safety_mode);
    }

    static SafetyState map_safety(std::int32_t ur_safety_mode) noexcept {
        // UR safety modes: 1 NORMAL, 2 REDUCED, 3 PROTECTIVE_STOP,
        // 4 RECOVERY, 5 SAFEGUARD_STOP, 6 SYSTEM_EMERGENCY_STOP,
        // 7 ROBOT_EMERGENCY_STOP, 8 VIOLATION, 9 FAULT.
        switch (ur_safety_mode) {
            case 1: return SafetyState::kNormal;
            case 2: return SafetyState::kReduced;
            case 3:
            case 4:
            case 5: return SafetyState::kProtectiveStop;
            case 6:
            case 7: return SafetyState::kEStop;
            default: return SafetyState::kFault;
        }
    }

    urcl::UrDriver& driver_;
    std::string name_;
    urcl::rtde_interface::DataPackage pkg_;
    urcl::vector6d_t q_{};
    urcl::vector6d_t target_q_{};
    double setpoint_rad_{};
    AxisState state_{};
};

}  // namespace robonode
