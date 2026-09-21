#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/core/status.hpp"
#include "robonode/sandbox/compiler.hpp"
#include "robonode/wasm/wasm_emit.hpp"
#include "robonode/wasm/wasm_runner.hpp"
#include "robonode/vision/detector.hpp"

namespace robonode {

// A user vision algorithm compiled to WebAssembly and run in Wasmtime behind the
// Detector seam. Same narrow ABI as the interpreter path (pose x,y,z -> three
// outputs), same host validation (finite + inside the workspace); a fuel trap or
// out-of-bounds answer yields no detection. This is the real WASM sandbox: the
// user's source becomes an isolated, fuel-limited module, no external toolchain.
class WasmDetector final : public Detector {
public:
    WasmDetector(VisionContext ctx, std::shared_ptr<wasm::WasmModule> module, Vec3 lo, Vec3 hi,
                 std::uint64_t fuel)
        : ctx_{std::move(ctx)}, module_{std::move(module)}, lo_{lo}, hi_{hi}, fuel_{fuel} {
        const auto it = ctx_.config.find("target");
        target_ = it == ctx_.config.end() ? "part" : it->second;
    }

    std::vector<Detection> detect() override {
        if (!ctx_.site_pose || !module_) return {};
        const Vec3 p = ctx_.site_pose(target_);
        std::vector<double> out;
        if (!module_->eval({p.x, p.y, p.z}, 3, out, fuel_).ok()) return {};
        if (out.size() != 3) return {};
        for (const double v : out) if (!std::isfinite(v)) return {};
        const Vec3 q{out[0], out[1], out[2]};
        if (q.x < lo_.x || q.x > hi_.x || q.y < lo_.y || q.y > hi_.y || q.z < lo_.z || q.z > hi_.z) {
            return {};
        }
        return {{target_, q}};
    }

private:
    VisionContext ctx_;
    std::shared_ptr<wasm::WasmModule> module_;
    Vec3 lo_, hi_;
    std::uint64_t fuel_;
    std::string target_;
};

inline Status register_wasm_detector(VisionRegistry& reg, const std::string& version,
                                     const std::string& source, Vec3 bounds_lo = {-3, -3, -1},
                                     Vec3 bounds_hi = {3, 3, 3}, std::uint64_t fuel = 1'000'000) {
    sandbox::Program prog;
    if (const auto st = sandbox::Compiler::compile(source, {"x", "y", "z"}, 3, prog); !st.ok()) {
        return st;
    }
    std::shared_ptr<wasm::WasmModule> module;
    {
        std::unique_ptr<wasm::WasmModule> m;
        if (const auto st = wasm::WasmModule::create(wasm::emit(prog), m); !st.ok()) return st;
        module = std::move(m);
    }
    reg.add(version, [module, bounds_lo, bounds_hi, fuel](
                         const VisionContext& c) -> std::unique_ptr<Detector> {
        return std::make_unique<WasmDetector>(c, module, bounds_lo, bounds_hi, fuel);
    });
    return Status::success();
}

}  // namespace robonode
