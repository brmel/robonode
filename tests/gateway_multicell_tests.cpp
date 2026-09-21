// More than one robot on one platform: each with its own chain, its own worker
// and its own log, addressed by id. A command names the cell it is for, and the
// one it does not name stays where it was.
//
// Built only with ROBONODE_BUILD_MUJOCO (needs the physics world).

#include "gateway_fixture.hpp"

using namespace robonode::testing;
using nlohmann::json;
using namespace std::chrono_literals;

namespace {

// C7/#multi-cell: the CellManager runs more than one cell independently — a
// second robot is `add(id,...)`, each with its own worker + telemetry.
void test_cell_manager_runs_two_independent_cells() {
    robonode::CellManager m{kWorlds, kCell, {}, robonode::vendor_drivers(),
                            robonode::vision_versions(), ROBONODE_SCENES};
    CHECK(m.add("robot2", kWorlds, kCell).ok());
    CHECK(m.has("main") && m.has("robot2"));
    const auto ids = json::parse(m.cells_json());
    CHECK(ids.size() == 2);
    // Both boot their own 7-node cell; a swap on one does not touch the other.
    CHECK(json::parse(m.cell("main")->nodes_json()).at("nodes").size() == 7);
    CHECK(json::parse(m.cell("robot2")->nodes_json()).at("nodes").size() == 7);
    CHECK(json::parse(m.cell("robot2")->submit_command(
                          R"({"cmd":"set_version","capability":"planner","version":"robonode.moveJ"})"))
              .at("ok") == true);
    CHECK(poll_until([&] { return m.cell("robot2")->capability_json("planner"); },
                     [](const json& j) { return j.at("version") == "robonode.moveJ"; }, 5s));
    CHECK(json::parse(m.cell("main")->capability_json("planner")).at("version") == "robonode.moveL");  // untouched
}

// Multi-cell is reachable, not just modelled: a second robot is added at run
// time, runs independently, and can be removed. The main cell is not removable.
void test_platform_adds_and_removes_cells() {
    auto p = platform();
    CHECK(json::parse(p.cells_json()).size() == 1);

    CHECK(p.add_cell("robot2").ok());
    CHECK(!p.add_cell("robot2").ok());  // ids are unique
    const auto ids = json::parse(p.cells_json());
    CHECK(ids.size() == 2);

    CHECK(!p.remove_cell("main").ok());  // the main cell stays
    CHECK(p.remove_cell("robot2").ok());
    CHECK(!p.remove_cell("robot2").ok());
    CHECK(json::parse(p.cells_json()).size() == 1);
}


// A robot is a cell descriptor in a catalogue: starting a second one with a
// different chain is data, not code. Six joints where the first has seven.
void test_a_second_robot_runs_with_its_own_chain() {
    auto p = platform(ROBONODE_APPS);
    CHECK(p.add_cell("fixed", "ur10e-fixed.cell.json").ok());
    const auto cells = json::parse(p.cells_json());
    CHECK(cells.size() == 2);
    std::map<std::string, int> chain;
    for (const auto& c : cells) chain[c.at("id").get<std::string>()] = c.at("nodes").get<int>();
    CHECK(chain.at("main") == 7);
    CHECK(chain.at("fixed") == 6);

    // A descriptor the catalogue does not have is refused, and nothing starts.
    CHECK(!p.add_cell("ghost", "no-such-robot.cell.json").ok());
    CHECK(json::parse(p.cells_json()).size() == 2);
    CHECK(p.remove_cell("fixed").ok());
}

// A second robot you cannot drive is decoration. A command names the cell it is
// for; the one it does not name stays where it was.
void test_a_command_drives_the_robot_it_names() {
    auto p = platform(ROBONODE_APPS);
    CHECK(p.add_cell("second", "ur10e-fixed.cell.json").ok());

    const auto pos_of = [&p](const std::string& cell) {
        return json::parse(p.telemetry_json(cell)).at("pos").get<std::vector<double>>();
    };
    // Read the baseline from a cell that has FINISHED booting. A cell still
    // settling into its start pose moves on its own, and this assertion would
    // then be reporting that as a failure of isolation — the wrong answer to
    // the right question, intermittently.
    CHECK(p.await_settled().ok());
    const auto main_before = pos_of("");
    CHECK(pos_of("second").size() == 6);  // its own chain, not a copy of the first

    const auto ack = json::parse(p.submit_command(R"({"cmd":"run","cell":"second"})"));
    CHECK(ack.at("ok") == true);
    CHECK(p.await_settled(kProgramTimeout).ok());

    // The robot that was asked moved; the one that was not did not.
    const auto second_after = pos_of("second");
    bool moved = false;
    for (const auto& q : second_after) moved = moved || std::abs(q) > 1e-6;
    CHECK(moved);
    const auto main_after = pos_of("");
    for (std::size_t i = 0; i < main_before.size(); ++i) {
        CHECK(std::abs(main_after[i] - main_before[i]) < 1e-6);
    }

    // A cell that does not exist is refused rather than silently answered by
    // main — for commands AND for reads, because a plausible wrong answer about
    // a different robot is the worst kind.
    const auto ghost = json::parse(p.submit_command(R"({"cmd":"run","cell":"ghost"})"));
    CHECK(ghost.at("ok") == false);
    CHECK(!p.has_cell("ghost"));
    CHECK(p.has_cell(""));  // empty means the first one, which always exists
    // A reader that outlives the robot it was watching (an SSE stream, a slow
    // request) gets nothing, not a crash and not another robot's answer.
    CHECK(p.remove_cell("second").ok());
    CHECK(p.telemetry_json("second").empty());
    CHECK(!p.has_cell("second"));
}

// A cell-scoped view answers about THAT cell. The log was the last one that did
// not: every cell shared one buffer, so `/logs?cell=x` showed x's records mixed
// with every other robot's — a view naming a scope the platform could not back.
void test_logs_belong_to_the_cell_that_made_them() {
    auto p = platform(ROBONODE_APPS);
    CHECK(p.add_cell("second", "ur10e-fixed.cell.json").ok());
    // Count the runs each cell reports, not the whole buffer: main keeps an idle
    // ticker that logs contact changes, so "unchanged" would be a flake, not a
    // claim about scope.
    const auto runs_reported = [](const std::string& body) {
        std::size_t n = 0;
        for (const auto& line : json::parse(body)) {
            n += line.get<std::string>().find("run ok") != std::string::npos ? 1 : 0;
        }
        return n;
    };
    // Counted as a DELTA: the ring is process-wide, so an earlier test that also
    // named a cell "second" leaves records behind. What this asserts is that the
    // run lands on the cell that was asked, not that the buffer starts empty.
    const auto main_runs_before = runs_reported(p.logs_json("main"));
    const auto second_runs_before = runs_reported(p.logs_json("second"));

    CHECK(json::parse(p.submit_command(R"({"cmd":"run","cell":"second"})")).at("ok") == true);
    CHECK(p.await_settled(kProgramTimeout, "second").ok());

    CHECK(runs_reported(p.logs_json("second")) == second_runs_before + 1);
    // The robot that was not asked reports no run of its own.
    CHECK(runs_reported(p.logs_json("main")) == main_runs_before);
}

}  // namespace

int main() {
    test_cell_manager_runs_two_independent_cells();
    test_platform_adds_and_removes_cells();
    test_a_second_robot_runs_with_its_own_chain();
    test_a_command_drives_the_robot_it_names();
    test_logs_belong_to_the_cell_that_made_them();
    std::puts("gateway_multicell_tests: OK");
    return 0;
}
