// robonode::celld module tests — descriptor loading (limits-as-data) and
// lifecycle orchestration over the node tree.

#include <cmath>
#include <cstdio>

#include "check.hpp"
#include "robonode/celld/cell.hpp"

namespace {

void test_descriptor_load_matches_canonical_example() {
    robonode::Descriptor d;
    const auto st =
        robonode::load_descriptor(ROBONODE_IDL_EXAMPLES "/rail-x.descriptor.json", d);
    CHECK(st.ok());
    CHECK(d.id == "rail-x");
    CHECK(d.driver == "robonode.sim-axis");
    CHECK(d.simulated);
    CHECK(d.command_rate_hz == 1000.0);
    CHECK(d.limits.position_min_mm == 0.0);
    CHECK(d.limits.position_max_mm == 1450.0);
    CHECK(d.limits.velocity_max_mm_s == 1200.0);
    CHECK(d.limits.acceleration_max_mm_s2 == 8000.0);
    CHECK(d.limits.jerk_max_mm_s3 == 120000.0);
}

void test_descriptor_load_rejects_garbage() {
    robonode::Descriptor d;
    CHECK(!robonode::load_descriptor("does-not-exist.json", d).ok());
}

void test_cell_lifecycle_and_coherent_run() {
    robonode::Descriptor rail, turret;
    CHECK(robonode::load_descriptor(ROBONODE_IDL_EXAMPLES "/rail-x.descriptor.json", rail).ok());
    CHECK(robonode::load_descriptor(ROBONODE_IDL_EXAMPLES "/turret-a.descriptor.json", turret).ok());

    robonode::Cell cell;
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

    CHECK(cell.run_waypoints({{0.0, 500.0, 300.0}, {-10.0, 45.0, 20.0}}, 1000.0, rows, stats).ok());
    CHECK(rows.size() == 2);
    CHECK(std::abs(rows[0].back().actual_position_mm - 300.0) < 0.5);
    CHECK(std::abs(rows[1].back().actual_position_mm - 20.0) < 0.5);
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
    d.driver = "robonode.ethercat-cia402";  // real driver, not in v0 registry
    d.limits = {0, 100, 10, 100, 0};
    d.command_rate_hz = 1000;
    robonode::Cell cell;
    CHECK(!cell.add_node(d).ok());
}

}  // namespace

int main() {
    test_descriptor_load_matches_canonical_example();
    test_descriptor_load_rejects_garbage();
    test_cell_lifecycle_and_coherent_run();
    test_cell_rejects_unknown_driver();
    std::puts("robonode celld: all tests passed");
    return 0;
}
