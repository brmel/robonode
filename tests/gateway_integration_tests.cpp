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

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must be defined for the gateway integration test"
#endif

namespace {

const std::string kWorld = std::string{ROBONODE_WORLDS} + "/rail_ur10e.xml";

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
    robonode::CellGateway gw{kWorld};
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
    robonode::CellGateway gw{kWorld};
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
    robonode::CellGateway gw{kWorld};
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
    robonode::CellGateway gw{kWorld};
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

// The forced interface refuses what it does not know — bad commands and
// unregistered driver families fail closed, never corrupt the cell.
void test_gateway_rejects_bad_commands() {
    robonode::CellGateway gw{kWorld};
    CHECK(json::parse(gw.submit_command(R"({"cmd":"nonsense"})")).at("ok") == false);
    CHECK(json::parse(gw.submit_command(R"({"cmd":"driver","family":"warp"})")).at("ok") == false);
    CHECK(json::parse(gw.submit_command("not json at all")).at("ok") == false);
    // The cell is still intact and serving 7 nodes.
    CHECK(json::parse(gw.nodes_json()).at("nodes").size() == 7);
}

}  // namespace

int main() {
    test_gateway_boots_seven_nodes_with_driver_versions();
    test_gateway_run_moves_the_cell_and_streams_telemetry();
    test_gateway_swaps_one_node_driver_live();
    test_gateway_swap_clock_owner_keeps_physics_alive();
    test_gateway_rejects_bad_commands();
    std::puts("robonode gateway integration: all tests passed");
    return 0;
}
