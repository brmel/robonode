#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/core/status.hpp"
#include "robonode/sandbox/compiler.hpp"
#include "robonode/sandbox/program.hpp"
#include "robonode/vision/detector.hpp"

namespace robonode {

class SandboxedDetector final : public Detector {
public:
    SandboxedDetector(VisionContext ctx, sandbox::Program program, sandbox::SandboxLimits limits,
                      Vec3 bounds_lo, Vec3 bounds_hi)
        : ctx_{std::move(ctx)}, program_{std::move(program)}, limits_{limits},
          lo_{bounds_lo}, hi_{bounds_hi} {
        const auto it = ctx_.config.find("target");
        target_ = it == ctx_.config.end() ? "part" : it->second;
    }

    std::vector<Detection> detect() override {
        if (!ctx_.site_pose) return {};
        const Vec3 p = ctx_.site_pose(target_);
        std::vector<double> out;
        if (!sandbox::run(program_, {p.x, p.y, p.z}, limits_, out).ok()) return {};
        if (out.size() != 3) return {};
        for (const double v : out) if (!std::isfinite(v)) return {};
        const Vec3 q{out[0], out[1], out[2]};
        if (!in_bounds(q)) return {};
        return {{target_, q}};
    }

private:
    [[nodiscard]] bool in_bounds(const Vec3& q) const {
        return q.x >= lo_.x && q.x <= hi_.x && q.y >= lo_.y && q.y <= hi_.y && q.z >= lo_.z &&
               q.z <= hi_.z;
    }

    VisionContext ctx_;
    sandbox::Program program_;
    sandbox::SandboxLimits limits_;
    Vec3 lo_, hi_;
    std::string target_;
};

inline Status compile_vision_program(const std::string& source, sandbox::Program& out) {
    return sandbox::Compiler::compile(source, {"x", "y", "z"}, 3, out);
}

inline void register_program_detector(VisionRegistry& reg, const std::string& version,
                                      sandbox::Program program, Vec3 bounds_lo = {-3, -3, -1},
                                      Vec3 bounds_hi = {3, 3, 3},
                                      sandbox::SandboxLimits limits = {}) {
    reg.add(version, [program = std::move(program), limits, bounds_lo, bounds_hi](
                         const VisionContext& c) -> std::unique_ptr<Detector> {
        return std::make_unique<SandboxedDetector>(c, program, limits, bounds_lo, bounds_hi);
    });
}

inline Status register_sandboxed_detector(VisionRegistry& reg, const std::string& version,
                                          const std::string& source,
                                          Vec3 bounds_lo = {-3, -3, -1},
                                          Vec3 bounds_hi = {3, 3, 3},
                                          sandbox::SandboxLimits limits = {}) {
    sandbox::Program program;
    if (const auto st = compile_vision_program(source, program); !st.ok()) return st;
    register_program_detector(reg, version, std::move(program), bounds_lo, bounds_hi, limits);
    return Status::success();
}

}  // namespace robonode
