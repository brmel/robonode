#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "check.hpp"
#include "robonode/sandbox/compiler.hpp"
#include "robonode/wasm/wasm_detector.hpp"
#include "robonode/wasm/wasm_emit.hpp"
#include "robonode/wasm/wasm_runner.hpp"

namespace {

using robonode::sandbox::Compiler;
using robonode::sandbox::Program;

std::unique_ptr<robonode::wasm::WasmModule> module_from(const std::string& src,
                                                        std::size_t channels) {
    Program prog;
    CHECK(Compiler::compile(src, {"x", "y", "z"}, channels, prog).ok());
    std::unique_ptr<robonode::wasm::WasmModule> m;
    CHECK(robonode::wasm::WasmModule::create(robonode::wasm::emit(prog), m).ok());
    return m;
}

// The full loop: user source -> bytecode -> WASM -> Wasmtime -> correct numbers.
void test_wasm_evaluates_user_program() {
    auto m = module_from("x; y; z + 0.05", 3);
    std::vector<double> out;
    CHECK(m->eval({0.9, 0.1, 0.30}, 3, out).ok());
    CHECK(out.size() == 3);
    CHECK(std::abs(out[0] - 0.9) < 1e-9);
    CHECK(std::abs(out[2] - 0.35) < 1e-9);
}

void test_wasm_arithmetic_and_functions() {
    auto m = module_from("min(x, y); max(x, y); clamp(z, 0, 1)", 3);
    std::vector<double> out;
    CHECK(m->eval({2.0, 5.0, 9.0}, 3, out).ok());
    CHECK(std::abs(out[0] - 2.0) < 1e-9);
    CHECK(std::abs(out[1] - 5.0) < 1e-9);
    CHECK(std::abs(out[2] - 1.0) < 1e-9);
}

// Fuel is the real limit: a program that needs more ops than the budget traps.
void test_wasm_fuel_trips() {
    auto m = module_from("1 + 1 + 1 + 1 + 1 + 1", 1);
    std::vector<double> out;
    CHECK(m->eval({0, 0, 0}, 1, out, /*generous*/ 1'000).ok());       // runs with fuel
    CHECK(m->eval({0, 0, 0}, 1, out, /*starved*/ 1).ok() == false);   // traps without
}

// The Detector seam: a WASM user algorithm reports a transformed pose; an
// out-of-workspace answer is rejected (no detection).
void test_wasm_detector_behind_seam() {
    robonode::VisionRegistry reg;
    CHECK(robonode::register_wasm_detector(reg, "user.wasm-grasp", "x; y; z + 0.05").ok());
    robonode::VisionContext ctx;
    ctx.site_pose = [](const std::string&) { return robonode::Vec3{0.9, 0.1, 0.30}; };
    std::unique_ptr<robonode::Detector> det;
    CHECK(reg.make("user.wasm-grasp", ctx, det).ok());
    const auto ds = det->detect();
    CHECK(ds.size() == 1);
    CHECK(std::abs(ds[0].position.z - 0.35) < 1e-9);

    robonode::VisionRegistry reg2;
    CHECK(robonode::register_wasm_detector(reg2, "user.evil", "x; y; z + 1000").ok());
    std::unique_ptr<robonode::Detector> evil;
    CHECK(reg2.make("user.evil", ctx, evil).ok());
    CHECK(evil->detect().empty());
}

}  // namespace

int main() {
    test_wasm_evaluates_user_program();
    test_wasm_arithmetic_and_functions();
    test_wasm_fuel_trips();
    test_wasm_detector_behind_seam();
    std::puts("robonode wasm: all tests passed");
    return 0;
}
