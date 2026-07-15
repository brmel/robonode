#pragma once

#include <algorithm>

#include "robonode/core/limits.hpp"
#include "robonode/core/state.hpp"
#include "robonode/motion/setpoint_source.hpp"

// Ruckig is a motion-module implementation detail (ARCHITECTURE.md pin v0.17.3) —
// nothing outside motion/ may include it (boundary lint).
#include <ruckig/ruckig.hpp>

namespace robonode {

// The online trajectory generator: jerk-limited, time-optimal setpoints
// from the CURRENT kinematic state to the target — retargetable any cycle.
// This is the FR-2.6 streaming primitive: commands (SDK, Tier B, jog) just
// call retarget(); the output stays smooth and inside the profile whatever
// the caller does. "Online generation is the shock absorber between
// soft-RT commands and hard-RT streaming."
//
// Wraps ruckig::Ruckig<1> (community: single target state — exactly this
// use; waypoint sequences remain SyncBlendPlan's job, see ARCHITECTURE.md).
class Otg final : public SetpointSource {
public:
    Otg(double rate_hz, const AxisLimits& limits, double initial_position)
        : gen_{1.0 / rate_hz} {
        in_.current_position = {initial_position};
        in_.current_velocity = {0.0};
        in_.current_acceleration = {0.0};
        in_.max_velocity = {limits.velocity_max};
        in_.max_acceleration = {limits.acceleration_max};
        // Ruckig requires jerk > 0; descriptor jerk 0 means "no jerk data" —
        // fall back to a stiff but finite limit derived from acceleration.
        in_.max_jerk = {limits.jerk_max > 0.0 ? limits.jerk_max
                                                    : limits.acceleration_max * 100.0};
        retarget(initial_position);
    }

    // Retarget any cycle (rest target; velocity targets come with the
    // streaming API). Cheap: takes effect on the next next().
    void retarget(double position) noexcept {
        in_.target_position = {position};
        in_.target_velocity = {0.0};
        in_.target_acceleration = {0.0};
        done_ = false;
    }

    State next(double /*t_s*/, double /*dt_s*/) noexcept override {
        if (!done_) {
            const auto res = gen_.update(in_, out_);
            if (res == ruckig::Result::Working || res == ruckig::Result::Finished) {
                out_.pass_to_input(in_);  // Finished delivers the exact target state
                if (res == ruckig::Result::Finished) done_ = true;
            } else {
                done_ = true;  // error: hold current state (governor-safe)
            }
        }
        return {in_.current_position[0], in_.current_velocity[0], in_.current_acceleration[0]};
    }

    [[nodiscard]] double duration_s() const noexcept override {
        return done_ ? 0.0 : out_.trajectory.get_duration();
    }

    [[nodiscard]] bool done() const noexcept { return done_; }

private:
    ruckig::Ruckig<1> gen_;
    ruckig::InputParameter<1> in_;
    ruckig::OutputParameter<1> out_;
    bool done_{false};
};

}  // namespace robonode
