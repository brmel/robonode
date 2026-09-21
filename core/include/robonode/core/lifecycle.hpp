#pragma once

#include "robonode/core/status.hpp"

namespace robonode {

// Mirrors robonode.v0.NodeLifecycle (FR-1.2). celld drives these
// transitions; until it exists, apps drive them explicitly.
enum class Lifecycle {
    kUnconfigured,
    kConfiguring,
    kInactive,
    kActive,
    kDegraded,
    kFault,
};

// Uniform lifecycle verbs every module/adapter implements — the MIL
// Alloc/Control/Free discipline, matched to the IDL state machine:
//   configure(): unconfigured → inactive (acquire/verify resources)
//   activate():  inactive → active      (begin exchanging with hardware)
//   deactivate(): active → inactive     (stop commanding, keep resources)
// Defaults are for modules with nothing to do at a given step.
class LifecycleParticipant {
public:
    virtual ~LifecycleParticipant() = default;

    virtual Status configure() {
        lifecycle_ = Lifecycle::kInactive;
        return Status::success();
    }
    virtual Status activate() {
        lifecycle_ = Lifecycle::kActive;
        return Status::success();
    }
    virtual Status deactivate() {
        lifecycle_ = Lifecycle::kInactive;
        return Status::success();
    }

    [[nodiscard]] Lifecycle lifecycle() const { return lifecycle_; }

protected:
    void set_lifecycle(Lifecycle l) { lifecycle_ = l; }

private:
    Lifecycle lifecycle_{Lifecycle::kUnconfigured};
};

}  // namespace robonode
