// The capability seam end to end: a version swapped live, one authored in the
// sandbox, a comparison judging two of them on the work that exercises them,
// and vision reporting what it actually saw. Split out of the integration suite
// when that file stopped being about one subject.
//
// Built only with ROBONODE_BUILD_MUJOCO (needs the physics world).

#include "gateway_fixture.hpp"

using namespace robonode::testing;
using nlohmann::json;
using namespace std::chrono_literals;

namespace {

// The vision capability reports the workpiece where it actually sits on the
// belt — the pose the whole picking pipeline is built on.
void test_gateway_vision_detects_part() {
    auto gw = gateway();
    const auto v = json::parse(gw.capability_json("vision"));
    CHECK(v.at("detected") == true);
    const auto& p = v.at("part");
    CHECK(std::abs(double(p[0]) - 1.05) < 0.02);  // where the belt presents it
    CHECK(std::abs(double(p[2]) - 0.375) < 0.02);  // resting on the belt surface
}

// ADR-11: vision is a swappable capability — set_version rebuilds the detector
// and the reported part pose moves (a different algorithm, same Detector seam).
void test_gateway_vision_version_swap() {
    auto gw = gateway();
    const auto v0 = json::parse(gw.capability_json("vision"));
    CHECK(v0.at("version") == "robonode.toy-detector");
    CHECK(v0.at("available").size() >= 2);
    // The part is a free body settling onto the belt, so z0 is a live reading,
    // not a constant: the assertion is about the algorithms differing by their
    // grasp offset, not about millimetres of contact settling.
    const double z0 = double(v0.at("part")[2]);
    CHECK(json::parse(gw.submit_command(
                          R"({"cmd":"set_version","capability":"vision","version":"robonode.toy-top-grasp"})"))
              .at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.capability_json("vision"); },
        [&](const json& j) {
            return j.at("version") == "robonode.toy-top-grasp" &&
                   std::abs(double(j.at("part")[2]) - (z0 + 0.05)) < 5e-3;
        },
        5s));
}

// ADR-11: trajectory is a swappable capability — moveL (straight line) and
// moveJ (point-to-point) both reach; set_version selects, nothing else changes.
void test_gateway_planner_version_swap() {
    auto gw = gateway();
    CHECK(json::parse(gw.capability_json("planner")).at("version") == "robonode.moveL");
    CHECK(json::parse(gw.submit_command(
                          R"({"cmd":"set_version","capability":"planner","version":"robonode.moveJ"})"))
              .at("ok") == true);
    CHECK(poll_until([&] { return gw.capability_json("planner"); },
                     [](const json& j) { return j.at("version") == "robonode.moveJ"; }, 5s));
    const json c = {{"cmd", "move_l"}, {"x", 0.9}, {"y", 0.25}, {"z", 0.35}};
    CHECK(json::parse(gw.submit_command(c.dump())).at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            if (!j.contains("tcp")) return false;
            const auto& t = j.at("tcp");
            const double dx = double(t[0]) - 0.9, dy = double(t[1]) - 0.25, dz = double(t[2]) - 0.35;
            return dx * dx + dy * dy + dz * dz < 0.03 * 0.03;  // moveJ reaches
        },
        20s));
}

void test_gateway_control_version_swap() {
    auto gw = gateway();
    const auto c0 = json::parse(gw.capability_json("control"));
    CHECK(c0.at("version") == "robonode.direct");
    CHECK(c0.at("available").size() >= 2);
    CHECK(json::parse(gw.submit_command(
                          R"({"cmd":"set_version","capability":"control","version":"robonode.smooth"})"))
              .at("ok") == true);
    CHECK(poll_until([&] { return gw.capability_json("control"); },
                     [](const json& j) { return j.at("version") == "robonode.smooth"; }, 5s));
    const json c = {{"cmd", "move_l"}, {"x", 0.9}, {"y", 0.25}, {"z", 0.35}};
    CHECK(json::parse(gw.submit_command(c.dump())).at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            if (!j.contains("tcp") || j.value("running", true)) return false;
            const auto& t = j.at("tcp");
            const double dx = double(t[0]) - 0.9, dy = double(t[1]) - 0.25, dz = double(t[2]) - 0.35;
            return dx * dx + dy * dy + dz * dz < 0.03 * 0.03;  // smooth control still reaches
        },
        20s));
}

void test_gateway_define_user_vision_module() {
    auto gw = gateway();
    const double z0 = double(json::parse(gw.capability_json("vision")).at("part")[2]);

    const auto bad = json::parse(gw.submit_command(
        R"({"cmd":"define_module","capability":"vision","name":"oops","source":"z +"})"));
    CHECK(bad.at("ok") == false);
    CHECK(bad.contains("error"));

    const auto ok = json::parse(gw.submit_command(
        R"({"cmd":"define_module","capability":"vision","name":"lift","source":"x; y; z + 0.08"})"));
    CHECK(ok.at("ok") == true);
    CHECK(ok.at("version") == "user.lift");
    CHECK(poll_until(
        [&] { return gw.capability_json("vision"); },
        [&](const json& j) {
            return j.at("version") == "user.lift" &&
                   std::abs(double(j.at("part")[2]) - (z0 + 0.08)) < 5e-3;
        },
        5s));
    const auto v = json::parse(gw.capability_json("vision"));
    bool listed = false;
    for (const auto& n : v.at("available")) listed = listed || n == "user.lift";
    CHECK(listed);
}

void test_gateway_define_user_planner_module() {
    auto gw = gateway();
    const auto ok = json::parse(gw.submit_command(
        R"({"cmd":"define_module","capability":"planner","name":"approach","source":"x; y; z + 0.2; x; y; z"})"));
    CHECK(ok.at("ok") == true);
    CHECK(ok.at("version") == "user.approach");
    CHECK(poll_until([&] { return gw.capability_json("planner"); },
                     [](const json& j) { return j.at("version") == "user.approach"; }, 5s));

    const json c = {{"cmd", "move_l"}, {"x", 0.9}, {"y", 0.25}, {"z", 0.35}};
    CHECK(json::parse(gw.submit_command(c.dump())).at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            if (!j.contains("tcp") || j.value("running", true)) return false;
            const auto& t = j.at("tcp");
            const double dx = double(t[0]) - 0.9, dy = double(t[1]) - 0.25, dz = double(t[2]) - 0.35;
            return dx * dx + dy * dy + dz * dz < 0.03 * 0.03;
        },
        20s));

    const auto bad = json::parse(gw.submit_command(
        R"({"cmd":"define_module","capability":"planner","name":"short","source":"x; y; z"})"));
    CHECK(bad.at("ok") == false);
    CHECK(bad.contains("error"));
}

void test_platform_persists_and_reloads_user_modules() {
    const auto dir = std::filesystem::temp_directory_path() /
                     ("rn-modules-" + std::to_string(::time(nullptr)));
    std::filesystem::remove_all(dir);
    {
        auto p = platform(ROBONODE_APPS, dir.string());
        CHECK(p.define_module("vision", "persisted", "x; y; z + 0.07").ok());
    }
    {
        auto p = platform(ROBONODE_APPS, dir.string());
        CHECK(poll_until(
            [&] { std::string v; (void)p.capability_json("vision", v); return v; },
            [](const json& j) {
                for (const auto& n : j.at("available")) {
                    if (n == "user.persisted") return true;
                }
                return false;
            },
            5s));
    }
    std::filesystem::remove_all(dir);
}

// One contract, many surfaces: /capabilities carries the descriptor every
// surface renders from — id, title, ABI, and whether users may author it.
void test_gateway_capabilities_describe_themselves() {
    auto gw = gateway();
    const auto caps = json::parse(gw.capabilities_json());
    // camera · vision · tracking · planner · control — every layer of the stack.
    CHECK(caps.is_array() && caps.size() == 5);
    bool saw_vision = false, saw_control = false, saw_tracking = false, saw_camera = false;
    for (const auto& c : caps) {
        const auto& d = c.at("descriptor");
        CHECK(d.contains("title") && d.contains("abi") && d.contains("authorable"));
        if (d.at("id") == "vision") {
            saw_vision = true;
            CHECK(d.at("authorable") == true);
            CHECK(d.at("abi").at("outputs") == 3);
            CHECK(!c.at("provenance").empty());  // versions carry their origin
        }
        if (d.at("id") == "control") {
            saw_control = true;
            CHECK(d.at("authorable") == false);  // no user code on the 1 kHz path
        }
        if (d.at("id") == "tracking") {
            saw_tracking = true;
            CHECK(d.at("authorable") == true);
            CHECK(d.at("abi").at("inputs").size() == 7);  // pose, velocity, lead time
        }
        if (d.at("id") == "camera") {
            saw_camera = true;
            CHECK(d.at("authorable") == false);  // a sensor is hardware, not an algorithm
            CHECK(c.at("available").size() >= 2);
        }
    }
    CHECK(saw_vision && saw_control && saw_tracking && saw_camera);
}


// Comparing two versions is ONE implementation in the facade: it runs the work
// the capability itself names, once per version, and restores what was live.
// Every surface renders that answer rather than measuring its own.
void test_compare_runs_each_version_on_the_capabilitys_trial() {
    auto p = platform(ROBONODE_APPS);
    std::string control_before;
    CHECK(p.capability_json("control", control_before).ok());
    const auto before = json::parse(control_before);

    std::string report;
    CHECK(p.compare("control", {"robonode.direct", "robonode.smooth"}, report).ok());
    const auto j = json::parse(report);
    CHECK(j.at("capability") == "control");
    CHECK(j.at("trial") == "pick-demo.app.json");  // what the settings name for control
    CHECK(j.at("results").size() == 2);
    for (const auto& r : j.at("results")) {
        CHECK(r.contains("ok"));
        CHECK(r.value("seconds", 0.0) > 0.0);
    }
    // The comparison leaves the cell as it found it.
    std::string control_after;
    CHECK(p.capability_json("control", control_after).ok());
    CHECK(json::parse(control_after).at("version") == before.at("version"));

    std::string ignored;
    CHECK(!p.compare("nope", {"a", "b"}, ignored).ok());
}

// An unknown capability is a caller error, not an empty answer.
void test_unknown_capability_is_refused() {
    auto p = platform();
    std::string view;
    CHECK(!p.capability_json("nope", view).ok());
    CHECK(p.capability_json("vision", view).ok());
    CHECK(!view.empty());
}

// The platform's hard problem, pinned: a part travelling on a belt cannot be
// picked by reaching for where it was. The naive tracker misses every attempt;
// swapping the tracking capability — and nothing else — catches it.
//
// This is the shape of every challenge the platform is meant to pose: the
// scenario is real physics, the failure is honest, and the fix is one seam.
void test_moving_target_needs_prediction_to_be_picked() {
    auto p = platform(ROBONODE_APPS);

    const auto run = [&](const char* tracker, int attempts) {
        CHECK(p.set_version("tracking", tracker).ok());
        CHECK(p.await_settled().ok());
        const auto ack = json::parse(p.submit_command(
            nlohmann::json{{"cmd", "run_app"},
                           {"program", {{{"verb", "conveyor"}, {"args", {{"station", "conveyor-1"}, {"run", "true"}}}},
                                        {{"verb", "intercept"}, {"args", {{"attempts", std::to_string(attempts)}}}}}}}
                .dump()));
        CHECK(ack.at("ok") == true);
        return p.await_settled();
    };

    // EQUAL budgets: what separates them is the algorithm, not how many tries
    // each was given. It did not use to be — snapshot got two attempts and
    // prediction four, and measuring with equal budgets showed the two were not
    // separable at all. Two things about the SCENE were why, and both are fixed:
    // the line now recirculates (the part no longer stops at the end where
    // anything can catch it), and the belt is slow enough that several reaches
    // fit inside one traverse (it was 1.4, so no tracker could converge).
    constexpr int kAttempts = 5;

    // Reaching for the last sighting: the belt has moved on by the time the arm
    // arrives, every time, and the part never stops to wait.
    CHECK(!run("robonode.snapshot", kAttempts).ok());
    CHECK(!json::parse(p.telemetry_json()).value("holding", true));

    // The same cell, the same app, one different algorithm.
    CHECK(run("robonode.constant-velocity", kAttempts).ok());
    const auto j = json::parse(p.telemetry_json());
    CHECK(j.value("holding", false));
}

// The other half of the control story, on the plant that ships.
//
// `test_control_smooth_tracks_gentler_than_direct_on_an_ideal_axis` shows
// smoothing winning where the plant answers instantly. Under inertia it loses:
// a smoothing controller lags further through the acceleration phase, so its
// PEAK following error is worse. That is not a defect in either version — it is
// the trade-off a person swapping them is entitled to see, and it is the reason
// `compare` reports how well a version RAN and not just whether it finished.
//
// Pinned because the docs claimed "gentler follow" unqualified for months while
// the platform's own comparison reported the opposite.
void test_smoothing_costs_peak_error_on_a_real_plant() {
    auto p = platform(ROBONODE_APPS);
    std::string report;
    CHECK(p.compare("control", {"robonode.direct", "robonode.smooth"}, report).ok());

    const auto results = json::parse(report).at("results");
    CHECK(results.size() == 2);
    const auto peak = [&](const char* version) {
        for (const auto& row : results) {
            if (row.at("version") == version) {
                return row.at("quality").at("worst_axis").at("following_error").get<double>();
            }
        }
        return -1.0;
    };
    const double direct = peak("robonode.direct");
    const double smooth = peak("robonode.smooth");
    CHECK(direct > 0.0 && smooth > 0.0);
    CHECK(smooth > direct);  // smoothing lags: the ideal-axis result does not transfer
}

// Vision reads the LIVE scene: a detector built on a scratch copy could never
// see a part travel, which would make the whole scenario a fiction.
void test_vision_sees_the_part_move_on_the_belt() {
    auto p = platform(ROBONODE_APPS);
    CHECK(json::parse(p.submit_command(
                          R"({"cmd":"run_app","program":[{"verb":"conveyor","args":{"station":"conveyor-1","run":"true"}}]})"))
              .at("ok") == true);
    CHECK(p.await_settled().ok());

    const auto part_x = [&] {
        const auto j = json::parse(p.telemetry_json());
        return double(j.at("vision").at("part")[0]);
    };
    const double first = part_x();
    CHECK(poll_until([&] { return p.telemetry_json(); },
                     [&](const json&) { return std::abs(part_x() - first) > 0.02; }, 10s));

    // …and the tracking capability turns that into a velocity.
    const auto tracking = json::parse(view_of(p, "tracking"));
    CHECK(tracking.at("speed") > 0.01);
}


// Sensing is not instantaneous and the workpiece is not alone in view. Both are
// part of the problem the platform poses, so both are pinned here.
[[maybe_unused]] void test_vision_reports_capture_time_and_ignores_clutter() {
    auto p = platform();
    CHECK(p.set_version("vision", "robonode.opencv").ok());
    CHECK(p.await_settled().ok());

    // Frames flow continuously; the first look after a swap has none yet.
    CHECK(poll_until([&] { std::string v; (void)p.capability_json("vision", v); return v; },
                     [](const json& j) { return j.value("detected", false); }, 10s));

    const auto v = json::parse(view_of(p, "vision"));
    CHECK(v.at("captured_s") > 0.0);            // stamped when the frame was taken
    CHECK(v.at("confidence") > 0.5);
    // The decoy is larger and the same colour, and sits at x≈0.62. Size
    // plausibility is what keeps the detector on the actual part.
    CHECK(std::abs(double(v.at("part")[0]) - 1.1) < 0.05);
}

}  // namespace

int main() {
    test_gateway_vision_detects_part();
    test_gateway_vision_version_swap();
    test_gateway_planner_version_swap();
    test_gateway_control_version_swap();
    test_gateway_define_user_vision_module();
    test_gateway_define_user_planner_module();
    test_platform_persists_and_reloads_user_modules();
    test_gateway_capabilities_describe_themselves();
    test_compare_runs_each_version_on_the_capabilitys_trial();
    test_smoothing_costs_peak_error_on_a_real_plant();
    test_unknown_capability_is_refused();
    test_moving_target_needs_prediction_to_be_picked();
    test_vision_sees_the_part_move_on_the_belt();
#ifdef ROBONODE_HAS_OPENCV
    test_vision_reports_capture_time_and_ignores_clutter();
#else
    std::puts("  skipped vision capture-time: needs robonode.opencv, "
              "and this build has no OpenCV");
#endif
    std::puts("gateway_capability_tests: OK");
    return 0;
}
