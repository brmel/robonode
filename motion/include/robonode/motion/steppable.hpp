#pragma once

namespace robonode {

// Something advanced exactly once per control cycle, independent of any single
// adapter — e.g. a shared physics world driven by several joint adapters. The
// executive ticks each DISTINCT steppable once per cycle in the step phase, so
// world-stepping never depends on which adapter happens to be the "owner". Any
// node can be swapped to a different driver and the shared world keeps
// advancing (fixes the clock-owner freeze, #50).
//
// RT contract: tick() is called once per cycle — no allocation, no blocking,
// no exceptions escaping.
class CycleSteppable {
public:
    virtual ~CycleSteppable() = default;
    virtual void tick(double dt_s) noexcept = 0;
};

}  // namespace robonode
