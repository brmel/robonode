#pragma once

#include <algorithm>

#include "robonode/core/state.hpp"
#include "robonode/motion/motion_plan.hpp"

namespace robonode {

// The OTG slot (SPEC §3.2), v0: whatever feeds the executive one setpoint
// per cycle. Occupants today: PlanSource (precomputed profiles) and Otg
// (Ruckig — retargetable mid-flight, FR-2.6). A Tier C plugin implements
// exactly this shape later; the executive neither knows nor cares.
//
// RT contract: next() is called once per cycle — no allocation, no
// blocking, no exceptions escaping.
class SetpointSource {
public:
    virtual ~SetpointSource() = default;

    // The setpoint for cycle time t (dt = cycle period). Sources with a
    // finite plan clamp internally past their end.
    virtual State next(double t_s, double dt_s) noexcept = 0;

    // Nominal duration; executives add their settle window. Retargetable
    // sources report the horizon of their CURRENT target.
    [[nodiscard]] virtual double duration_s() const noexcept = 0;
};

// MotionPlan as a SetpointSource — executives speak only the slot.
class PlanSource final : public SetpointSource {
public:
    explicit PlanSource(const MotionPlan& plan) : plan_{plan} {}

    State next(double t_s, double /*dt_s*/) noexcept override {
        return plan_.sample(std::min(t_s, plan_.duration_s()));
    }
    [[nodiscard]] double duration_s() const noexcept override { return plan_.duration_s(); }

private:
    const MotionPlan& plan_;
};

}  // namespace robonode
