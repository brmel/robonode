#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/core/limits.hpp"
#include "robonode/gateway/capability.hpp"
#include "robonode/log.hpp"
#include "robonode/motion/controller.hpp"

namespace robonode {

// Not authorable by design: no user code on the 1 kHz path (ADR-5). The flag
// says so in the descriptor, so surfaces stop offering an editor that fails.
inline CapabilityDescriptor control_descriptor() {
    return {"control", "Control", "🎛", "built-in versions only — no user code on the 1 kHz path",
            {}, /*authorable=*/false, {}, 0};
}

class ControlCapability final : public Capability<Controller, ControlContext> {
public:
    ControlCapability() : Capability{control_descriptor(), "robonode.direct"} {
        register_basic_controllers(registry_);
        remember({"robonode.direct", "builtin", {}});
        remember({"robonode.smooth", "builtin", {}});
    }

    void rebuild(std::vector<AxisLimits> per_node) {
        limits_ = std::move(per_node);
        build();
    }

    Status select(const std::string& version) override {
        if (!registry_.has(version)) return Status::failure("unknown version '" + version + "'");
        version_ = version;
        build();
        return Status::success();
    }

    void install(const std::string&, sandbox::Program, Select) override {
        RN_LOG_WARN("control versions cannot be authored (ADR-5)");
    }

    std::vector<Controller*> ptrs() {
        std::vector<Controller*> out;
        out.reserve(controllers_.size());
        for (auto& c : controllers_) out.push_back(c.get());
        return out;
    }

private:
    void build() {
        controllers_.clear();
        for (const auto& lim : limits_) {
            ControlContext ctx;
            ctx.limits = lim;
            std::unique_ptr<Controller> ctrl;
            if (!registry_.make(version_, ctx, ctrl).ok()) {
                RN_LOG_WARN("control version '{}' unavailable", version_);
                controllers_.clear();
                break;
            }
            controllers_.push_back(std::move(ctrl));
        }
        publish();
    }

    std::vector<std::unique_ptr<Controller>> controllers_;
    std::vector<AxisLimits> limits_;
};

}  // namespace robonode
