// Integration test — the CellGateway seam end to end: JSON command in →
// worker thread → celld → motion → MuJoCo physics → telemetry JSON out. This
// is the SAME contract the web UI and the (coming) CLI both bind to (#43), so
// exercising it here is exercising both surfaces at once. Cross-module by
// construction: gateway + celld + motion + sim_mujoco on one shared world.
//
// Built only with ROBONODE_BUILD_MUJOCO (needs the physics world).

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "check.hpp"
#include "robonode/gateway/cell_gateway.hpp"
#include "robonode/platform.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must be defined for the gateway integration test"
#endif

namespace {

const std::string kWorld = std::string{ROBONODE_WORLDS} + "/rail_ur10e.xml";
const std::string kCell = ROBONODE_CELL;

using nlohmann::json;
using namespace std::chrono_literals;

// Poll a thread-safe snapshot until `pred(parsed_json)` holds or timeout.
template <typename Getter, typename Pred>
bool poll_until(Getter get, Pred pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto j = json::parse(get(), nullptr, false);
        if (!j.is_discarded() && pred(j)) return true;
        std::this_thread::sleep_for(50ms);
    }
    return false;
}

// The cell boots with 7 nodes under the physics family, each advertising the
// three swappable driver versions — the node tree the UI/CLI render.
void test_gateway_boots_seven_nodes_with_driver_versions() {
    robonode::CellGateway gw{kWorld, kCell};
    const auto j = json::parse(gw.nodes_json());
    CHECK(j.at("family") == "physics");
    CHECK(j.at("nodes").size() == 7);
    CHECK(j.at("nodes")[0].at("id") == "rail-x");
    CHECK(j.at("nodes")[0].at("driver") == "robonode.mujoco-axis");
    // Every node offers the same forced-interface version list.
    const auto avail = j.at("available");
    bool has_mj = false, has_sim = false, has_soft = false;
    for (const auto& d : avail) {
        has_mj |= d == "robonode.mujoco-axis";
        has_sim |= d == "robonode.sim-axis";
        has_soft |= d == "robonode.sim-axis-soft";
    }
    CHECK(has_mj && has_sim && has_soft);
}

// Command in → coordinated move → telemetry out: the rail traverses to its
// final waypoint (400 mm) under real physics, proving the whole loop streamed.
void test_gateway_run_moves_the_cell_and_streams_telemetry() {
    robonode::CellGateway gw{kWorld, kCell};
    CHECK(json::parse(gw.submit_command(R"({"cmd":"run"})")).at("ok") == true);

    // rail-x (pos[0]) must reach its final waypoint (400 mm) within the run.
    const bool arrived = poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return !pos.empty() && std::abs(double(pos[0]) - 400.0) < 20.0;
        },
        20s);
    CHECK(arrived);
}

// A single node's implementation is swapped live through the registry — the
// per-node "try each version" contract. The published tree reflects it.
void test_gateway_swaps_one_node_driver_live() {
    robonode::CellGateway gw{kWorld, kCell};
    CHECK(json::parse(gw.submit_command(R"({"cmd":"set_driver","node":"j3","driver":"robonode.sim-axis"})"))
              .at("ok") == true);

    const bool swapped = poll_until(
        [&] { return gw.nodes_json(); },
        [](const json& j) {
            for (const auto& n : j.at("nodes")) {
                if (n.at("id") == "j3") return n.at("driver") == "robonode.sim-axis";
            }
            return false;
        },
        5s);
    CHECK(swapped);
}

// Regression (#50): swapping the node that used to be the physics "clock
// owner" (rail-x, built first) to a non-physics driver must NOT freeze the
// shared world — the remaining physics joints still move. World-stepping is
// the executive's job now, not any adapter's identity.
void test_gateway_swap_clock_owner_keeps_physics_alive() {
    robonode::CellGateway gw{kWorld, kCell};
    CHECK(json::parse(gw.submit_command(
                          R"({"cmd":"set_driver","node":"rail-x","driver":"robonode.sim-axis"})"))
              .at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.nodes_json(); },
        [](const json& j) {
            for (const auto& n : j.at("nodes")) {
                if (n.at("id") == "rail-x") return n.at("driver") == "robonode.sim-axis";
            }
            return false;
        },
        5s));

    CHECK(json::parse(gw.submit_command(R"({"cmd":"run"})")).at("ok") == true);
    // j1 (pos[1], still robonode.mujoco-axis) must move off zero under physics.
    const bool j1_moved = poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return pos.size() > 1 && std::abs(double(pos[1])) > 0.05;
        },
        20s);
    CHECK(j1_moved);
}

// Bring-your-own (#23): the user example driver is registered, shows up in
// every node's version list, and drives a node when selected — proving a
// third-party AxisAdapter needs nothing but the seam.
void test_gateway_byo_example_driver_selectable_and_drives() {
    robonode::CellGateway gw{kWorld, kCell};
    const auto avail = json::parse(gw.nodes_json()).at("available");
    bool has_byo = false;
    for (const auto& d : avail) has_byo |= d == "robonode.byo-example";
    CHECK(has_byo);

    CHECK(json::parse(gw.submit_command(
                          R"({"cmd":"set_driver","node":"j2","driver":"robonode.byo-example"})"))
              .at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.nodes_json(); },
        [](const json& j) {
            for (const auto& n : j.at("nodes")) {
                if (n.at("id") == "j2") return n.at("driver") == "robonode.byo-example";
            }
            return false;
        },
        5s));

    CHECK(json::parse(gw.submit_command(R"({"cmd":"run"})")).at("ok") == true);
    const bool j2_moved = poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return pos.size() > 2 && std::abs(double(pos[2])) > 0.05;
        },
        20s);
    CHECK(j2_moved);
}

// #42/#44: events land on the log surface — after a run, logs_json carries the
// run record. The same followable stream the UI panel and `robonode logs` tail.
void test_gateway_logs_surface_records_events() {
    robonode::CellGateway gw{kWorld, kCell};
    CHECK(json::parse(gw.submit_command(R"({"cmd":"run"})")).at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return !pos.empty() && std::abs(double(pos[0]) - 400.0) < 20.0;
        },
        20s));
    const auto logs = json::parse(gw.logs_json());
    CHECK(logs.is_array() && !logs.empty());
    bool has_run = false;
    for (const auto& l : logs) has_run |= std::string(l).find("run") != std::string::npos;
    CHECK(has_run);
}

// #22: a Cartesian goal — real IK plans a straight moveL and the arm reaches
// the TCP target in physics (within servo tolerance).
void test_gateway_cartesian_move_reaches_target() {
    robonode::CellGateway gw{kWorld, kCell};
    const auto home = json::parse(gw.telemetry_json());
    CHECK(home.contains("tcp"));
    const auto& h = home.at("tcp");
    const double tx = double(h[0]) + 0.06, ty = double(h[1]) - 0.05, tz = double(h[2]) - 0.08;
    const json c = {{"cmd", "move_l"}, {"x", tx}, {"y", ty}, {"z", tz}};
    CHECK(json::parse(gw.submit_command(c.dump())).at("ok") == true);

    const bool arrived = poll_until(
        [&] { return gw.telemetry_json(); },
        [&](const json& j) {
            if (!j.contains("tcp")) return false;
            const auto& t = j.at("tcp");
            const double dx = double(t[0]) - tx, dy = double(t[1]) - ty, dz = double(t[2]) - tz;
            return dx * dx + dy * dy + dz * dz < 0.03 * 0.03;  // real UR10e servos reach ~cm
        },
        20s);
    CHECK(arrived);
}

// #6: the toy vision detector reports the target part's pose from the sim.
void test_gateway_vision_detects_part() {
    robonode::CellGateway gw{kWorld, kCell};
    const auto v = json::parse(gw.vision_json());
    CHECK(v.contains("part"));
    const auto& p = v.at("part");
    CHECK(std::abs(double(p[0]) - 0.9) < 0.02);
    CHECK(std::abs(double(p[2]) - 0.35) < 0.02);
}

// ADR-11: vision is a swappable capability — set_version rebuilds the detector
// and the reported part pose moves (a different algorithm, same Detector seam).
void test_gateway_vision_version_swap() {
    robonode::CellGateway gw{kWorld, kCell};
    const auto v0 = json::parse(gw.vision_json());
    CHECK(v0.at("version") == "robonode.toy-detector");
    CHECK(v0.at("available").size() >= 2);
    const double z0 = double(v0.at("part")[2]);
    CHECK(json::parse(gw.submit_command(
                          R"({"cmd":"set_version","capability":"vision","version":"robonode.toy-top-grasp"})"))
              .at("ok") == true);
    CHECK(poll_until(
        [&] { return gw.vision_json(); },
        [&](const json& j) {
            return j.at("version") == "robonode.toy-top-grasp" &&
                   std::abs(double(j.at("part")[2]) - (z0 + 0.05)) < 1e-6;
        },
        5s));
}

// #61/#64: the program engine deploys a real saved app end to end. The
// bin-picking program is family → pick (vision → part) → move_l (place); the
// TCP must finish at the place target — the vision→pick→place primitive.
void test_gateway_bin_picking_app_places_part() {
    robonode::Platform p{kWorld, kCell, ROBONODE_APPS};
    CHECK(p.run_app("bin-picking.app.json").ok());
    const bool placed = poll_until(
        [&] { return p.telemetry_json(); },
        [](const json& j) {
            if (!j.contains("tcp") || j.value("running", true)) return false;  // program done
            const auto& t = j.at("tcp");
            const double dx = double(t[0]) - 0.5, dy = double(t[1]) + 0.3, dz = double(t[2]) - 0.5;
            return dx * dx + dy * dy + dz * dz < 0.05 * 0.05;  // at the place target
        },
        40s);
    CHECK(placed);
}

// The forced interface refuses what it does not know — bad commands and
// unregistered driver families fail closed, never corrupt the cell.
void test_gateway_rejects_bad_commands() {
    robonode::CellGateway gw{kWorld, kCell};
    CHECK(json::parse(gw.submit_command(R"({"cmd":"nonsense"})")).at("ok") == false);
    CHECK(json::parse(gw.submit_command(R"({"cmd":"driver","family":"warp"})")).at("ok") == false);
    CHECK(json::parse(gw.submit_command("not json at all")).at("ok") == false);
    // The cell is still intact and serving 7 nodes.
    CHECK(json::parse(gw.nodes_json()).at("nodes").size() == 7);
}

// #33: the Platform facade — one typed entry (run / set_node_driver /
// nodes_json), no JSON hand-crafting; the same object the CLI (#43) binds to.
void test_platform_facade_typed_verbs() {
    robonode::Platform p{kWorld, kCell};
    CHECK(json::parse(p.nodes_json()).at("nodes").size() == 7);
    CHECK(p.run().ok());
    const bool arrived = poll_until(
        [&] { return p.telemetry_json(); },
        [](const json& j) {
            const auto& pos = j.at("pos");
            return !pos.empty() && std::abs(double(pos[0]) - 400.0) < 20.0;
        },
        20s);
    CHECK(arrived);
    CHECK(p.set_node_driver("j3", "robonode.sim-axis").ok());
}

}  // namespace

int main() {
    test_platform_facade_typed_verbs();
    test_gateway_boots_seven_nodes_with_driver_versions();
    test_gateway_run_moves_the_cell_and_streams_telemetry();
    test_gateway_swaps_one_node_driver_live();
    test_gateway_swap_clock_owner_keeps_physics_alive();
    test_gateway_byo_example_driver_selectable_and_drives();
    test_gateway_logs_surface_records_events();
    test_gateway_cartesian_move_reaches_target();
    test_gateway_vision_detects_part();
    test_gateway_vision_version_swap();
    test_gateway_bin_picking_app_places_part();
    test_gateway_rejects_bad_commands();
    std::puts("robonode gateway integration: all tests passed");
    return 0;
}
