// robonode::celld module tests — descriptor loading (limits-as-data) and
// lifecycle orchestration over the node tree.

#include <cmath>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <filesystem>
#include <fstream>

#include "check.hpp"
#include "robonode/celld/app_descriptor.hpp"
#include "robonode/celld/json_doc_store.hpp"
#include "robonode/celld/cell.hpp"
#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/celld/robot_node.hpp"
#include "robonode/celld/settings_io.hpp"
#include "robonode/celld/station.hpp"
#include "robonode/motion/sim_axis.hpp"
#include "robonode/motion/sim_driver.hpp"

namespace {

robonode::DriverRegistry sim_registry() {
    robonode::DriverRegistry reg;
    robonode::register_sim_axis(reg);
    return reg;
}


void test_descriptor_load_matches_canonical_example() {
    robonode::Descriptor d;
    const auto st =
        robonode::load_descriptor(ROBONODE_IDL_EXAMPLES "/rail-x.descriptor.json", d);
    CHECK(st.ok());
    CHECK(d.id == "rail-x");
    CHECK(d.driver == "robonode.sim-axis");
    CHECK(d.simulated);
    CHECK(d.command_rate_hz == 1000.0);
    CHECK(d.limits.position_min == 0.0);
    CHECK(d.limits.position_max == 1450.0);
    CHECK(d.limits.velocity_max == 1200.0);
    CHECK(d.limits.acceleration_max == 8000.0);
    CHECK(d.limits.jerk_max == 120000.0);
}

void test_descriptor_load_rejects_garbage() {
    robonode::Descriptor d;
    CHECK(!robonode::load_descriptor("does-not-exist.json", d).ok());
}

void test_cell_lifecycle_and_coherent_run() {
    robonode::Descriptor rail, turret;
    CHECK(robonode::load_descriptor(ROBONODE_IDL_EXAMPLES "/rail-x.descriptor.json", rail).ok());
    CHECK(robonode::load_descriptor(ROBONODE_IDL_EXAMPLES "/turret-a.descriptor.json", turret).ok());

    const auto reg = sim_registry();
    robonode::Cell cell{reg};
    CHECK(cell.add_node(rail).ok());
    CHECK(cell.add_node(turret).ok());

    // Running before activation must fail closed.
    std::vector<std::vector<robonode::TelemetryRow>> rows;
    robonode::CycleStats stats{};
    CHECK(!cell.run_waypoints({{0.0, 500.0}, {0.0, 45.0}}, 1000.0, rows, stats, 0.5, 0.3).ok());

    CHECK(cell.configure_all().ok());
    CHECK(cell.activate_all().ok());
    for (const auto& n : cell.nodes()) {
        CHECK(n.adapter->lifecycle() == robonode::Lifecycle::kActive);
    }

    // Waypoints start at each node's home (sim homes at 0 clamped into range).
    CHECK(cell.run_waypoints({{0.0, 500.0, 300.0}, {0.0, 45.0, 20.0}}, 1000.0, rows, stats, 0.5, 0.3).ok());
    CHECK(rows.size() == 2);
    CHECK(std::abs(rows[0].back().actual_position - 300.0) < 0.5);
    CHECK(std::abs(rows[1].back().actual_position - 20.0) < 0.5);
    // Limits came from the descriptor files, not from this test: governors
    // stayed silent because the plan respected the JSON envelope.
    for (const auto& n : cell.nodes()) {
        CHECK(n.governor->position_clamps() == 0);
        CHECK(n.governor->velocity_clamps() == 0);
    }

    CHECK(cell.deactivate_all().ok());
}

void test_cell_rejects_unknown_driver() {
    robonode::Descriptor d;
    d.id = "mystery";
    d.driver = "robonode.ethercat-cia402";  // real driver, not registered here
    d.limits = {0, 100, 10, 100, 0};
    d.command_rate_hz = 1000;
    const auto reg = sim_registry();  // sim only — ethercat unknown
    robonode::Cell cell{reg};
    CHECK(!cell.add_node(d).ok());
}

// The seam is generic, not sim-hardcoded: any factory registered under any
// name is dispatched by celld with the descriptor's id + limits + config.
void test_registry_dispatches_custom_driver() {
    robonode::DriverRegistry reg;
    std::string seen_id;
    std::string seen_ip;
    reg.register_driver("test.fake", [&](const robonode::DriverContext& ctx) {
        seen_id = ctx.id;
        auto ip = ctx.config.find("robot_ip");
        seen_ip = ip != ctx.config.end() ? ip->second : "";
        return std::make_unique<robonode::SimAxis>(ctx.id, ctx.limits.position_min, 0.005);
    });

    robonode::Descriptor d;
    d.id = "widget-1";
    d.driver = "test.fake";
    d.limits = {0, 100, 10, 100, 0};
    d.command_rate_hz = 1000;
    d.config = {{"robot_ip", "10.0.0.7"}};

    robonode::Cell cell{reg};
    CHECK(cell.add_node(d).ok());
    CHECK(seen_id == "widget-1");            // celld passed id through
    CHECK(seen_ip == "10.0.0.7");            // ...and config through
    CHECK(cell.nodes().size() == 1);
    CHECK(cell.nodes()[0].adapter->name() == "widget-1");
    CHECK(cell.configure_all().ok());        // built adapter is a real node
}

// #29: a cell is data — robot joints AND stations (conveyor/deck/pallet) load
// from one descriptor file, no ids or config in code.
void test_cell_descriptor_loads_nodes_and_stations() {
    robonode::CellDescriptor cd;
    CHECK(robonode::load_cell_descriptor(ROBONODE_CELL, cd).ok());
    // The cell names the world it runs in — a physics model, or a scene
    // descriptor that starts from one and adds the user's objects.
    CHECK(cd.world == "conveyor-line.scene.json");
    CHECK(cd.nodes.size() == 7);
    CHECK(cd.nodes[0].id == "rail-x");
    CHECK(cd.stations.size() == 2);
    CHECK(cd.stations[0].id == "conveyor-1");
    CHECK(cd.stations[0].type == "conveyor");
    CHECK(std::strtod(cd.stations[0].config.at("speed_m_s").c_str(), nullptr) > 0.0);
    CHECK(cd.stations[0].config.at("actuator") == "belt_drive");
    CHECK(cd.stations[1].type == "pallet");
    CHECK(cd.stations[1].config.at("cols") == "2");
}

// A descriptor names its axes in four places. A hand-edited robot that drops an
// axis from one of them used to load and fail somewhere unrelated later; every
// reference is resolved when it is parsed, whichever path it arrived by.
void test_a_descriptor_that_contradicts_itself_is_refused() {
    const auto parse = [](const char* body) {
        robonode::CellDescriptor out;
        return robonode::parse_cell_descriptor(nlohmann::json::parse(body, nullptr, false), out,
                                               "test");
    };
    const std::string kBase = R"({"world":"w.xml","families":[{"id":"physics","driver":"d"}],
        "nodes":[{"id":"j1","joint":"j1","actuator":"a1","limits":{"position_min":0,"position_max":1,"velocity_max":1,"acceleration_max":1,"jerk_max":1}},
                 {"id":"j2","joint":"j2","actuator":"a2","limits":{"position_min":0,"position_max":1,"velocity_max":1,"acceleration_max":1,"jerk_max":1}}],)";

    CHECK(parse((kBase + R"("robots":[{"id":"arm","joints":["j1","j2"]}]})").c_str()).ok());

    // A robot that articulates an axis the cell does not declare.
    auto st = parse((kBase + R"("robots":[{"id":"arm","joints":["j1","ghost"]}]})").c_str());
    CHECK(!st.ok());
    CHECK(st.message().find("ghost") != std::string::npos);

    // An axis that both carries the arm and belongs to it would be driven twice.
    st = parse((kBase + R"("robots":[{"id":"arm","carrier":["j1"],"joints":["j1","j2"]}]})").c_str());
    CHECK(!st.ok());
    CHECK(st.message().find("both carrier and joint") != std::string::npos);

    // Waypoint lists that disagree cannot be blended into one coordinated move.
    st = parse((kBase + R"("robots":[{"id":"arm","joints":["j1","j2"]}],
        "motions":[{"id":"m","waypoints":{"j1":[0,1,2],"j2":[0,1]}}]})").c_str());
    CHECK(!st.ok());
    CHECK(st.message().find("waypoints") != std::string::npos);

    // Two nodes with one id: whichever wins, half the descriptor means the other.
    st = parse(R"({"world":"w.xml","families":[{"id":"physics","driver":"d"}],
        "nodes":[{"id":"j1","joint":"j1","actuator":"a1","limits":{"position_min":0,"position_max":1,"velocity_max":1,"acceleration_max":1,"jerk_max":1}},
                 {"id":"j1","joint":"j2","actuator":"a2","limits":{"position_min":0,"position_max":1,"velocity_max":1,"acceleration_max":1,"jerk_max":1}}],
        "robots":[{"id":"arm","joints":["j1"]}]})");
    CHECK(!st.ok());
    CHECK(st.message().find("share the id") != std::string::npos);
}

// Two clients saving one document at once used to interleave into a file
// neither of them wrote, and a reader in between saw an empty one. A write
// lands whole or not at all.
void test_a_document_is_never_read_half_written() {
    const auto dir = std::filesystem::temp_directory_path() / "robonode-store-race";
    std::filesystem::remove_all(dir);
    const robonode::JsonDocStore store{dir.string(), ".json"};

    std::atomic<bool> stop{false};
    std::atomic<int> torn{0}, reads{0};
    std::thread reader{[&] {
        while (!stop.load()) {
            std::string body;
            if (!store.load("doc.json", body).ok()) continue;
            reads.fetch_add(1);
            if (nlohmann::json::parse(body, nullptr, false).is_discarded()) torn.fetch_add(1);
        }
    }};

    std::vector<std::thread> writers;
    for (int w = 0; w < 4; ++w) {
        writers.emplace_back([&store, w] {
            for (int n = 0; n < 50; ++n) {
                nlohmann::json doc{{"writer", w}, {"n", n}, {"pad", std::string(2000, 'x')}};
                (void)store.save("doc.json", doc.dump());
            }
        });
    }
    for (auto& w : writers) w.join();
    stop.store(true);
    reader.join();

    CHECK(reads.load() > 0);
    CHECK(torn.load() == 0);
    // And nothing is left behind: a staging file that survived would be listed.
    CHECK(store.list().size() == 1);
    std::filesystem::remove_all(dir);
}

// A cell asked for an impossible motion says so. This is the path that used to
// throw from the plan builder and, for one commit, silently reported success
// because the wrapper that catches exceptions discarded what its work returned.
void test_an_impossible_motion_is_refused_not_swallowed() {
    robonode::DriverRegistry registry;
    robonode::register_sim_axis(registry);
    robonode::Cell cell{registry};

    robonode::Descriptor rail;
    CHECK(robonode::load_descriptor(ROBONODE_IDL_EXAMPLES "/rail-x.descriptor.json", rail).ok());
    CHECK(cell.add_node(rail).ok());
    CHECK(cell.configure_all().ok());
    CHECK(cell.activate_all().ok());

    std::vector<std::vector<robonode::TelemetryRow>> rows;
    robonode::CycleStats stats{};
    // One waypoint is not a motion: the plan cannot be built, and the cell must
    // say that rather than report a move it never made.
    const auto st = cell.run_waypoints({{0.0}}, 1000.0, rows, stats, 0.5, 0.3);
    CHECK(!st.ok());
    CHECK(st.message().find("two waypoints") != std::string::npos);
    CHECK(stats.cycles == 0);
}

// A write that dies between staging and rename leaves a .part file. Nothing
// reads it, and nothing removed it either — a directory that accumulates debris
// forever is a leak with a tidy name.
void test_debris_from_a_dead_write_is_cleared() {
    const auto dir = std::filesystem::temp_directory_path() / "robonode-debris-test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    std::ofstream{dir / "one.json.part7"} << "half a document";
    std::ofstream{dir / "one.json"} << R"({"name":"one"})";

    const robonode::JsonDocStore store{dir.string(), ".json"};
    CHECK(store.list().size() == 1);  // it was never listed
    CHECK(!std::filesystem::exists(dir / "one.json.part7"));  // and now it is gone
    CHECK(store.read("one.json").has_value());
    std::filesystem::remove_all(dir);
}

// A read either hands back the document or says why it cannot. The failing path
// carries no value at all, which is the point: there is nothing to read by
// mistake, and the call can be used in an expression.
void test_a_read_returns_the_document_or_the_reason() {
    const auto dir = std::filesystem::temp_directory_path() / "robonode-result-test";
    std::filesystem::remove_all(dir);
    const robonode::JsonDocStore store{dir.string(), ".json"};
    CHECK(store.save("one.json", R"({"name":"one"})").ok());

    const auto found = store.read("one.json");
    CHECK(found.has_value());
    CHECK(found->find("one") != std::string::npos);

    const auto missing = store.read("nope.json");
    CHECK(!missing.has_value());
    CHECK(missing.error().message().find("nope.json") != std::string::npos);
    std::filesystem::remove_all(dir);
}

// A document is edited by hand, so a parse failure is a message to a person: it
// has to say which element was wrong, not just which key was missing.
void test_a_parse_error_says_where_it_was() {
    robonode::CellDescriptor out;
    const auto st = robonode::parse_cell_descriptor(nlohmann::json::parse(R"({
        "world":"w.xml","families":[{"id":"physics","driver":"d"}],
        "nodes":[{"id":"j1","joint":"j1","actuator":"a1",
                  "limits":{"position_min":0,"position_max":1,"velocity_max":1,"acceleration_max":1}},
                 {"id":"j2","joint":"j2","actuator":"a2","limits":{"position_max":1}}],
        "robots":[{"id":"arm","joints":["j1","j2"]}]})",
                                                                          nullptr, false),
                                                   out, "test");
    CHECK(!st.ok());
    CHECK(st.message().find("nodes[1] 'j2'") != std::string::npos);
    CHECK(st.message().find("position_min") != std::string::npos);
}

// Every robot the platform ships is one it can actually run.
void test_every_shipped_robot_is_valid() {
    for (const auto& entry : std::filesystem::directory_iterator{ROBONODE_ROBOTS}) {
        if (entry.path().extension() != ".json") continue;
        robonode::CellDescriptor cd;
        const auto st = robonode::load_cell_descriptor(entry.path().string(), cd);
        if (!st.ok()) std::printf("  %s: %s\n", entry.path().filename().c_str(), st.message().c_str());
        CHECK(st.ok());
    }
}

void test_pallet_station_computes_slots() {
    robonode::StationSpec s{"pallet-1", "pallet",
                            {{"cols", "2"}, {"ox", "0.45"}, {"oy", "-0.3"}, {"oz", "0.45"},
                             {"dx", "0.15"}, {"dz", "0.15"}}};
    const auto s0 = robonode::pallet_slot(s, 0);
    const auto s1 = robonode::pallet_slot(s, 1);
    const auto s3 = robonode::pallet_slot(s, 3);
    CHECK(std::abs(s0.x - 0.45) < 1e-9 && std::abs(s0.z - 0.45) < 1e-9);   // col0 row0
    CHECK(std::abs(s1.x - 0.60) < 1e-9 && std::abs(s1.z - 0.45) < 1e-9);   // col1 row0
    CHECK(std::abs(s3.x - 0.60) < 1e-9 && std::abs(s3.z - 0.60) < 1e-9);   // col1 row1
}

// #56: an Application is data — name + target cell + a program of task steps.
void test_app_descriptor_loads_program() {
    robonode::AppDescriptor ad;
    CHECK(robonode::load_app_descriptor(ROBONODE_APP, ad).ok());
    CHECK(ad.name == "Pick demo");
    CHECK(ad.cell == "ur10e.cell.json");
    CHECK(ad.program.size() == 2);
    CHECK(ad.program[0].verb == "family");
    CHECK(ad.program[0].args.at("family") == "physics");
    CHECK(ad.program[1].verb == "motion");
    CHECK(ad.program[1].args.at("id") == "home-cycle");  // motions are named cell data
}

// #59: the app store lists + loads saved applications (filesystem backend).
void test_app_store_lists_and_loads() {
    robonode::JsonDocStore store{ROBONODE_APPS};
    const auto apps = store.list();
    CHECK(!apps.empty());
    bool has_pick = false;
    std::string pick_file;
    for (const auto& a : apps) {
        if (a.name == "Pick demo") { has_pick = true; pick_file = a.file; }
    }
    CHECK(has_pick);
    std::string body;
    CHECK(store.load(pick_file, body).ok());
    CHECK(body.find("Pick demo") != std::string::npos);
    CHECK(!store.load("nope.json", body).ok());  // missing fails closed
}


// A robot is bound by NAME: nothing depends on where an axis sits in the flat
// node list, and the unit conversion between the cell (mm) and the model (m)
// lives in exactly one place.
void test_robot_node_binds_by_name_and_scales_units() {
    robonode::DriverRegistry reg;
    robonode::register_sim_axis(reg);

    robonode::CellDescriptor desc;
    desc.nodes = {{"rail-x", "rail", "rail_servo", "mm", 1000.0, {0, 1450, 1200, 8000, 0}},
                  {"j1", "j1", "j1_servo", "rad", 1.0, {-6.28, 6.28, 3, 15, 0}},
                  {"j2", "j2", "j2_servo", "rad", 1.0, {-6.28, 6.28, 3, 15, 0}}};
    robonode::RobotSpec spec{"arm", {"rail-x"}, {"j1", "j2"}, "tcp"};

    robonode::Cell cell{reg};
    for (const auto& n : desc.nodes) {
        robonode::Descriptor d;
        d.id = n.id;
        d.driver = "robonode.sim-axis";
        d.limits = n.limits;
        CHECK(cell.add_node(d).ok());
    }
    CHECK(cell.configure_all().ok() && cell.activate_all().ok());

    robonode::RobotNode robot;
    CHECK(robonode::RobotNode::bind(spec, cell, desc, /*carrier_weight=*/0.1, robot).ok());
    CHECK(robot.dof() == 3);                       // carrier first, then the arm
    CHECK(robot.joint_weights()[0] < robot.joint_weights()[1]);  // carrier is a last resort

    // Model units out (rail 0 mm -> 0 m), cell units back in (0.25 m -> 250 mm).
    CHECK(robot.joint_positions(cell).size() == 3);
    std::vector<std::vector<double>> cell_wp;
    CHECK(robot.compose(cell, {{0.0, 0.25}, {0.0, 0.5}, {0.0, -0.5}}, cell_wp).ok());
    CHECK(cell_wp.size() == 3);
    CHECK(std::abs(cell_wp[0].back() - 250.0) < 1e-9);  // metres -> mm
    CHECK(std::abs(cell_wp[1].back() - 0.5) < 1e-9);    // radians unchanged

    robonode::RobotNode missing;
    const robonode::RobotSpec bad{"arm", {}, {"nope"}, "tcp"};
    CHECK(!robonode::RobotNode::bind(bad, cell, desc, 0.1, missing).ok());
}

// Tuning is data: a partial settings file overrides only what it names.
void test_settings_partial_override_keeps_defaults() {
    const auto path = std::filesystem::temp_directory_path() / "robonode-settings-test.json";
    std::ofstream{path} << R"({"ik":{"carrier_weight":0.5},"motion":{"settle_s":2.5}})";

    robonode::Settings s;
    CHECK(robonode::load_settings(path.string(), s).ok());
    CHECK(std::abs(s.ik.carrier_weight - 0.5) < 1e-12);
    CHECK(std::abs(s.motion.settle_s - 2.5) < 1e-12);
    CHECK(std::abs(s.motion.rate_hz - robonode::Settings{}.motion.rate_hz) < 1e-12);  // untouched
    CHECK(!robonode::load_settings("no-such-settings.json", s).ok());
    std::filesystem::remove(path);
}


// The store is generic over document kind: the suffix is what a store is for,
// so apps and modules share one implementation without seeing each other.
void test_json_doc_store_filters_by_suffix() {
    const auto dir = std::filesystem::temp_directory_path() / "robonode-docstore-test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    std::ofstream{dir / "one.app.json"} << R"({"name":"One"})";
    std::ofstream{dir / "two.module.json"} << R"({"name":"Two"})";
    std::ofstream{dir / "notes.txt"} << "ignored";

    const robonode::JsonDocStore apps{dir.string(), ".app.json"};
    const robonode::JsonDocStore modules{dir.string(), ".module.json"};
    CHECK(apps.list().size() == 1);
    CHECK(apps.list().front().name == "One");
    CHECK(modules.list().size() == 1);
    CHECK(modules.list().front().name == "Two");

    CHECK(apps.save("three.app.json", R"({"name":"Three"})").ok());
    CHECK(apps.list().size() == 2);
    CHECK(apps.remove("three.app.json").ok());
    CHECK(!apps.remove("three.app.json").ok());  // gone stays gone
    std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
    test_descriptor_load_matches_canonical_example();
    test_app_store_lists_and_loads();
    test_descriptor_load_rejects_garbage();
    test_cell_lifecycle_and_coherent_run();
    test_cell_rejects_unknown_driver();
    test_registry_dispatches_custom_driver();
    test_cell_descriptor_loads_nodes_and_stations();
    test_a_descriptor_that_contradicts_itself_is_refused();
    test_an_impossible_motion_is_refused_not_swallowed();
    test_debris_from_a_dead_write_is_cleared();
    test_a_read_returns_the_document_or_the_reason();
    test_a_document_is_never_read_half_written();
    test_a_parse_error_says_where_it_was();
    test_every_shipped_robot_is_valid();
    test_pallet_station_computes_slots();
    test_app_descriptor_loads_program();
    test_robot_node_binds_by_name_and_scales_units();
    test_settings_partial_override_keeps_defaults();
    test_json_doc_store_filters_by_suffix();
    std::puts("robonode celld: all tests passed");
    return 0;
}
