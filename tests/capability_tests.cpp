#include <cmath>
#include <cstdio>
#include <string>

#include <nlohmann/json.hpp>

#include "check.hpp"
#include "robonode/gateway/control_capability.hpp"
#include "robonode/gateway/vision_capability.hpp"
#include "robonode/sandbox/compiler.hpp"

using json = nlohmann::json;

namespace {

void test_vision_capability_detects_swaps_and_defines() {
    robonode::VisionCapability vision;
    vision.set_site_pose([](const std::string&) { return robonode::Vec3{0.9, 0.1, 0.30}; });
    vision.rebuild();
    const auto part = vision.observe();
    CHECK(part.has_value());
    CHECK(std::abs(part->z - 0.30) < 1e-9);

    auto j0 = json::parse(vision.json());
    CHECK(j0.at("version") == "robonode.toy-detector");
    CHECK(j0.at("available").size() >= 2);

    CHECK(vision.select("robonode.toy-top-grasp").ok());
    CHECK(std::abs(vision.observe()->z - 0.35) < 1e-9);

    robonode::sandbox::Program prog;
    CHECK(robonode::compile_vision_program("x; y; z + 0.02", prog).ok());
    vision.install("user.lift", std::move(prog), robonode::Select::kNow);
    CHECK(std::abs(vision.observe()->z - 0.32) < 1e-9);
    CHECK(json::parse(vision.json()).at("version") == "user.lift");
}

void test_control_capability_builds_per_node_and_swaps() {
    robonode::ControlCapability control;
    control.rebuild({robonode::AxisLimits{}, robonode::AxisLimits{}, robonode::AxisLimits{}});
    CHECK(control.ptrs().size() == 3);
    CHECK(json::parse(control.json()).at("version") == "robonode.direct");

    CHECK(control.select("robonode.smooth").ok());
    CHECK(json::parse(control.json()).at("version") == "robonode.smooth");
    CHECK(control.ptrs().size() == 3);

    CHECK(!control.select("robonode.does-not-exist").ok());  // unknown version is refused, not silently empty
    CHECK(control.ptrs().size() == 3);                   // the working controllers stay
}

}  // namespace

int main() {
    test_vision_capability_detects_swaps_and_defines();
    test_control_capability_builds_per_node_and_swaps();
    std::puts("robonode capability: all tests passed");
    return 0;
}
