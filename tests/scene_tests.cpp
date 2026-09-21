// Scenes and sessions: changing the world is data, and everyone's changes are
// their own. This is what lets a user start from a scenario that works and
// make it theirs without touching XML or anyone else's files.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "check.hpp"
#include "robonode/celld/scene_descriptor.hpp"
#include "robonode/gateway/workspace.hpp"
#include "robonode/motion/planner.hpp"
#include "robonode/sim_mujoco/mjcf_composer.hpp"
#include "robonode/sim_mujoco/mujoco_kinematics.hpp"

namespace {

const std::string kWorlds = ROBONODE_WORLDS;

// Composes into the worlds directory so relative mesh paths still resolve.
robonode::MjcfComposer composer() { return robonode::MjcfComposer{kWorlds, kWorlds}; }

void test_scene_descriptor_reads_objects_as_data() {
    robonode::SceneDescriptor scene;
    const auto json = nlohmann::json::parse(R"({
        "name": "Test line",
        "base": "rail_ur10e.xml",
        "objects": [
          {"name": "crate", "type": "box", "size": [0.1, 0.1, 0.12], "pos": [0.55, 0, 0.12],
           "collides": true, "mass": 8.0},
          {"name": "runner", "size": [0.03, 0.03, 0.03], "pos": [1.3, 0.25, 0.37],
           "collides": false, "slide_axis": "-1 0 0", "slide_range": [-0.5, 0.5]}
        ]})");
    CHECK(robonode::parse_scene_descriptor(json, scene).ok());
    CHECK(scene.base == "rail_ur10e.xml");
    CHECK(scene.objects.size() == 2);
    CHECK(scene.objects[0].name == "crate");
    CHECK(std::abs(scene.objects[0].size[2] - 0.12) < 1e-9);
    CHECK(scene.objects[0].collides);
    CHECK(!scene.objects[1].collides);
    CHECK(scene.objects[1].slide_axis == "-1 0 0");  // something that travels
}

// A scene with no changes is the base world itself: composing nothing costs
// nothing, and the shipped scenario stays byte-identical.
void test_empty_scene_uses_the_base_world_untouched() {
    robonode::SceneDescriptor scene;
    scene.name = "Plain";
    scene.base = "rail_ur10e.xml";

    std::string world;
    CHECK(composer().compose(scene, world).ok());
    CHECK(world == (std::filesystem::path{kWorlds} / "rail_ur10e.xml").string());
}

// Adding an object to the JSON puts a body in the physics model.
void test_composed_scene_contains_the_placed_objects() {
    robonode::SceneDescriptor scene;
    scene.name = "composer-test";
    scene.base = "rail_ur10e.xml";
    robonode::SceneObject crate;
    crate.name = "crate";
    crate.pos[0] = 0.55;
    scene.objects.push_back(crate);

    std::string world;
    CHECK(composer().compose(scene, world).ok());
    std::ifstream f{world};
    CHECK(f.good());
    const std::string xml{std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{}};
    CHECK(xml.find("name=\"crate\"") != std::string::npos);
    CHECK(xml.find("</worldbody>") != std::string::npos);  // still a valid document
    std::filesystem::remove(world);
}

// #93: composition was additive only, so a scene could add clutter but never
// move the conveyor or delete the decoy. An override edits what the BASE world
// declares — the point at which "make it yours" stops meaning "add to it".
void test_overrides_move_and_remove_what_the_base_declares() {
    robonode::SceneDescriptor scene;
    scene.name = "override-test";
    scene.base = "rail_ur10e.xml";
    scene.overrides.push_back({"conveyor", std::array<double, 3>{0.9, 0.4, 0.33}, {}, {}, {}});
    scene.overrides.push_back({"decoy", {}, false, {}, {}});
    scene.overrides.push_back({"part", {}, {}, {}, 1.2});

    std::string world;
    CHECK(composer().compose(scene, world).ok());
    std::ifstream f{world};
    const std::string xml{std::istreambuf_iterator<char>{f}, std::istreambuf_iterator<char>{}};
    CHECK(xml.find("pos=\"0.9 0.4 0.33\"") != std::string::npos);  // the belt moved
    CHECK(xml.find("name=\"decoy\"") == std::string::npos);        // the decoy is gone
    CHECK(xml.find("mass=\"1.2\"") != std::string::npos);          // the part got heavier
    std::filesystem::remove(world);
}

// An override naming a body the base does not have is an error: a scene that
// silently ignores half of what it says is worse than one that refuses.
void test_an_override_of_something_that_does_not_exist_is_refused() {
    robonode::SceneDescriptor scene;
    scene.name = "bad-override";
    scene.base = "rail_ur10e.xml";
    scene.overrides.push_back({"no-such-body", std::array<double, 3>{0, 0, 0}, {}, {}, {}});
    std::string world;
    const auto st = composer().compose(scene, world);
    CHECK(!st.ok());
    CHECK(st.message().find("no-such-body") != std::string::npos);
}

// Overrides are data like everything else: the descriptor reads them from JSON.
void test_overrides_parse_from_json() {
    robonode::SceneDescriptor scene;
    const auto json = nlohmann::json::parse(R"({
        "base": "rail_ur10e.xml",
        "overrides": [
          {"name": "conveyor", "pos": [0.9, 0.4, 0.33]},
          {"name": "decoy", "visible": false},
          {"name": "part", "material": "steel", "mass": 1.2}
        ]})");
    CHECK(robonode::parse_scene_descriptor(json, scene).ok());
    CHECK(scene.overrides.size() == 3);
    CHECK(scene.overrides[0].pos.has_value());
    CHECK(scene.overrides[1].visible.has_value() && !*scene.overrides[1].visible);
    CHECK(scene.overrides[2].material.value_or("") == "steel");
}

// #39, deterministically: in a world with a crate in the way, the straight line
// between two points passes through it and the avoiding planner never emits a
// path that does. Physics is not involved — this is the plan, not the run.
void test_the_avoiding_planner_never_plans_through_the_crate() {
    robonode::SceneDescriptor scene;
    scene.name = "planner-clutter";
    scene.base = "rail_ur10e.xml";
    robonode::SceneObject crate;
    crate.name = "crate";
    crate.pos[0] = 0.55;
    crate.pos[1] = -0.05;
    crate.pos[2] = 0.12;
    crate.size[0] = crate.size[1] = 0.1;
    crate.size[2] = 0.12;
    crate.mass = 8.0;
    scene.objects.push_back(crate);

    std::string world;
    CHECK(composer().compose(scene, world).ok());

    std::unique_ptr<robonode::MujocoKinematics> kin;
    CHECK(robonode::MujocoKinematics::create(
              world, {"j1", "j2", "j3", "j4", "j5", "j6"}, "tcp", kin)
              .ok());

    const std::vector<double> start{0.0, -1.5708, -1.5708, 1.5708, -1.5708, -1.5708};
    const robonode::Vec3 across{0.35, -0.05, 0.20};

    const auto blocked = [&](const std::vector<std::vector<double>>& wp) {
        std::vector<double> q(wp.size());
        for (std::size_t s = 0; s < wp.front().size(); ++s) {
            for (std::size_t k = 0; k < wp.size(); ++k) q[k] = wp[k][s];
            if (kin->in_collision(q)) return true;
        }
        return false;
    };

    // The naive path: reachable, and straight through the crate.
    // Start beside the crate, so the line to the far side runs through it.
    std::vector<std::vector<double>> approach;
    CHECK(robonode::plan_move_l(*kin, start, robonode::Vec3{0.75, -0.05, 0.20}, 20, approach, {}).ok());
    std::vector<double> beside(approach.size());
    for (std::size_t k = 0; k < approach.size(); ++k) beside[k] = approach[k].back();

    std::vector<std::vector<double>> direct;
    CHECK(robonode::plan_move_l(*kin, beside, across, 20, direct, {}).ok());
    CHECK(blocked(direct));

    // The avoiding planner either routes round it or refuses — never through.
    robonode::AvoidingPlanner planner{*kin, 20, 0.18};
    robonode::Goal goal;
    goal.kind = robonode::Goal::kCartesianPosition;
    goal.cartesian = across;
    std::vector<std::vector<double>> planned;
    if (planner.plan(beside, goal, planned).ok()) {
        CHECK(!blocked(planned));
    } else {
        CHECK(planned.empty() || !blocked(planned));
    }
    std::filesystem::remove(world);
}

void test_missing_base_is_refused() {
    robonode::SceneDescriptor scene;
    scene.base = "no-such-world.xml";
    scene.objects.push_back({});
    std::string world;
    CHECK(!composer().compose(scene, world).ok());

    robonode::SceneDescriptor parsed;
    CHECK(!robonode::parse_scene_descriptor(nlohmann::json::parse(R"({"name":"x"})"), parsed).ok());
}

// A session is a workspace: what you author is yours, and it does not appear in
// anyone else's library.
void test_sessions_keep_each_users_work_apart() {
    const auto root = std::filesystem::temp_directory_path() / "robonode-sessions-test";
    std::filesystem::remove_all(root);
    robonode::Sessions sessions{root.string()};

    auto& alice = sessions.open("alice");
    auto& bob = sessions.open("bob");
    CHECK(alice.scenes().save("mine.scene.json", R"({"name":"Alice","base":"rail_ur10e.xml"})").ok());

    CHECK(alice.scenes().list().size() == 1);
    CHECK(bob.scenes().list().empty());  // bob sees nothing of alice's

    // Opening the same session again is the same workspace, not a new one.
    CHECK(sessions.open("alice").scenes().list().size() == 1);
    CHECK(sessions.exists("alice"));

    const auto listed = nlohmann::json::parse(sessions.list_json());
    CHECK(listed.size() >= 2);

    CHECK(sessions.remove("alice").ok());
    CHECK(!sessions.exists("alice"));
    CHECK(!sessions.remove("shared").ok());  // the shared space is not deletable
    std::filesystem::remove_all(root);
}

}  // namespace

int main() {
    test_scene_descriptor_reads_objects_as_data();
    test_empty_scene_uses_the_base_world_untouched();
    test_composed_scene_contains_the_placed_objects();
    test_overrides_parse_from_json();
    test_overrides_move_and_remove_what_the_base_declares();
    test_an_override_of_something_that_does_not_exist_is_refused();
    test_the_avoiding_planner_never_plans_through_the_crate();
    test_missing_base_is_refused();
    test_sessions_keep_each_users_work_apart();
    std::puts("robonode scenes: all tests passed");
    return 0;
}
