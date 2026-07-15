// robonode::celld module tests — descriptor loading (limits-as-data) and
// lifecycle orchestration over the node tree.

#include <cmath>
#include <cstdio>

#include "check.hpp"
#include "robonode/celld/cell.hpp"
#include "robonode/celld/cell_descriptor.hpp"
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
    CHECK(!cell.run_waypoints({{0.0, 500.0}, {0.0, 45.0}}, 1000.0, rows, stats).ok());

    CHECK(cell.configure_all().ok());
    CHECK(cell.activate_all().ok());
    for (const auto& n : cell.nodes()) {
        CHECK(n.adapter->lifecycle() == robonode::Lifecycle::kActive);
    }

    // Waypoints start at each node's home (sim homes at 0 clamped into range).
    CHECK(cell.run_waypoints({{0.0, 500.0, 300.0}, {0.0, 45.0, 20.0}}, 1000.0, rows, stats).ok());
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
    CHECK(cd.nodes.size() == 7);
    CHECK(cd.nodes[0].id == "rail-x");
    CHECK(cd.stations.size() == 1);
    CHECK(cd.stations[0].id == "conveyor-1");
    CHECK(cd.stations[0].type == "conveyor");
    CHECK(cd.stations[0].config.at("speed_mm_s") == "150");
}

}  // namespace

int main() {
    test_descriptor_load_matches_canonical_example();
    test_descriptor_load_rejects_garbage();
    test_cell_lifecycle_and_coherent_run();
    test_cell_rejects_unknown_driver();
    test_registry_dispatches_custom_driver();
    test_cell_descriptor_loads_nodes_and_stations();
    std::puts("robonode celld: all tests passed");
    return 0;
}
