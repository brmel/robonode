#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "check.hpp"
#include "robonode/sandbox/compiler.hpp"
#include "robonode/sandbox/program.hpp"
#include "robonode/vision/sandboxed_detector.hpp"

namespace {

using robonode::sandbox::Compiler;
using robonode::sandbox::Program;
using robonode::sandbox::SandboxLimits;

Program compiled(const std::string& src, std::size_t channels = 3) {
    Program p;
    CHECK(Compiler::compile(src, {"x", "y", "z"}, channels, p).ok());
    return p;
}

double eval1(const std::string& src, std::vector<double> in) {
    Program p = compiled(src, 1);
    std::vector<double> out;
    CHECK(robonode::sandbox::run(p, in, {}, out).ok());
    CHECK(out.size() == 1);
    return out[0];
}

void test_compiler_evaluates_infix() {
    CHECK(std::abs(eval1("1 + 2 * 3", {0, 0, 0}) - 7.0) < 1e-9);
    CHECK(std::abs(eval1("(1 + 2) * 3", {0, 0, 0}) - 9.0) < 1e-9);
    CHECK(std::abs(eval1("-x + 0.5", {2, 0, 0}) - (-1.5)) < 1e-9);
    CHECK(std::abs(eval1("x * y - z", {2, 3, 1}) - 5.0) < 1e-9);
    CHECK(std::abs(eval1("min(x, y)", {2, 5, 0}) - 2.0) < 1e-9);
    CHECK(std::abs(eval1("max(x, y)", {2, 5, 0}) - 5.0) < 1e-9);
    CHECK(std::abs(eval1("clamp(x, 0, 1)", {9, 0, 0}) - 1.0) < 1e-9);
    CHECK(std::abs(eval1("abs(0 - x)", {4, 0, 0}) - 4.0) < 1e-9);
}

void test_compiler_rejects_malformed() {
    Program p;
    CHECK(!Compiler::compile("x +", {"x", "y", "z"}, 1, p).ok());
    CHECK(!Compiler::compile("(x + y", {"x", "y", "z"}, 1, p).ok());
    CHECK(!Compiler::compile("open(x)", {"x", "y", "z"}, 1, p).ok());
    CHECK(!Compiler::compile("secret", {"x", "y", "z"}, 1, p).ok());
    CHECK(!Compiler::compile("x; y", {"x", "y", "z"}, 3, p).ok());
}

void test_sandbox_fuel_budget_trips() {
    Program p = compiled("1 + 1 + 1 + 1 + 1", 1);
    std::vector<double> out;
    CHECK(robonode::sandbox::run(p, {0, 0, 0}, SandboxLimits{.max_ops = 3, .max_stack = 64}, out)
              .ok() == false);
}

void test_sandboxed_detector_transforms_pose() {
    robonode::VisionRegistry reg;
    CHECK(robonode::register_sandboxed_detector(reg, "user.top-grasp", "x; y; z + 0.05").ok());
    robonode::VisionContext ctx;
    ctx.site_pose = [](const std::string&) { return robonode::Vec3{0.9, 0.1, 0.30}; };
    std::unique_ptr<robonode::Detector> det;
    CHECK(reg.make("user.top-grasp", ctx, det).ok());
    const auto ds = det->detect();
    CHECK(ds.size() == 1);
    CHECK(std::abs(ds[0].position.z - 0.35) < 1e-9);
    CHECK(std::abs(ds[0].position.x - 0.9) < 1e-9);
}

void test_sandboxed_detector_rejects_out_of_bounds() {
    robonode::VisionRegistry reg;
    CHECK(robonode::register_sandboxed_detector(reg, "user.evil", "x; y; z + 1000").ok());
    robonode::VisionContext ctx;
    ctx.site_pose = [](const std::string&) { return robonode::Vec3{0.9, 0.1, 0.30}; };
    std::unique_ptr<robonode::Detector> det;
    CHECK(reg.make("user.evil", ctx, det).ok());
    CHECK(det->detect().empty());
}

}  // namespace

int main() {
    test_compiler_evaluates_infix();
    test_compiler_rejects_malformed();
    test_sandbox_fuel_budget_trips();
    test_sandboxed_detector_transforms_pose();
    test_sandboxed_detector_rejects_out_of_bounds();
    std::puts("robonode sandbox: all tests passed");
    return 0;
}
