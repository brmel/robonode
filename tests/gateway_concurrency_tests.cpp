// What happens when two threads want the same thing at once: readers against
// the physics, the tool and the stations against the idle ticker, planning
// against telemetry, sessions opened from everywhere, and a cell dropped while
// someone is reading it. These are the tests ThreadSanitizer is pointed at, so
// they are worth having in one place.
//
// Built only with ROBONODE_BUILD_MUJOCO (needs the physics world).

#include "gateway_fixture.hpp"

using namespace robonode::testing;
using nlohmann::json;
using namespace std::chrono_literals;

namespace {

// MuJoCo's mjData has one owner. Every surface reads it — telemetry, the
// capability views, the model — and doing that from an HTTP thread while the
// worker steps physics corrupts the solver's stack (it surfaces as a crash
// inside mj_collideTree, one run in four). Readers must see a snapshot, and
// this test is the thing that fails when someone reaches for the live world.
void test_readers_do_not_race_the_physics() {
    auto p = platform(ROBONODE_APPS);
    std::atomic<bool> stop{false};
    std::atomic<int> reads{0};
    std::vector<std::thread> readers;
    for (int i = 0; i < 4; ++i) {
        readers.emplace_back([&] {
            while (!stop.load()) {
                (void)p.telemetry_json();
                std::string ignored;
                (void)p.capability_json("vision", ignored);
                (void)p.capability_json("tracking", ignored);
                (void)p.stations_json();
                reads.fetch_add(1);
            }
        });
    }
    CHECK(p.run_app("moving-bin-picking.app.json").ok());
    const auto settled = p.await_settled(kProgramTimeout);
    stop.store(true);
    for (auto& r : readers) r.join();
    CHECK(settled.ok());
    CHECK(reads.load() > 100);  // the readers really did hammer it
}

// The tool and the stations touch the physics directly, and the idle ticker
// steps it between commands. They used to do that without the cell lock, which
// is two threads inside one mjData: an intermittent SIGSEGV in mj_step (#103).
// Hammering both paths while the cell is idle is what reproduces it.
void test_tool_and_stations_do_not_race_the_idle_ticker() {
    auto p = platform(ROBONODE_APPS);
    for (int i = 0; i < 40; ++i) {
        // Each of these may legitimately fail (nothing at the tool, no part to
        // recycle yet); what must never happen is a crash in the physics.
        (void)p.run_conveyor("", robonode::Belt::kRunning);
        (void)p.grasp();
        (void)p.release();
        (void)p.run_conveyor("", robonode::Belt::kStopped);
        (void)p.deliver("");
    }
    // Several of those commands are expected to fail (nothing at the tool yet):
    // what this test asserts is that the platform is still ALIVE and answering
    // after hammering the two paths that used to touch physics unlocked.
    // A faulted cell settles with a reason rather than hanging: whether the
    // last grasp found a part or not, the platform answers.
    (void)p.await_settled();
    const auto j = json::parse(p.telemetry_json(), nullptr, false);
    CHECK(!j.is_discarded());
    CHECK(j.value("running", true) == false);
    CHECK(!j.value("state", std::string{}).empty());
}

// The kinematics' scratch world is an mjData too, and telemetry asks it where
// the tool points on every frame while the worker plans against it. Planning
// under a live telemetry stream is the situation that crashed.
void test_planning_does_not_race_the_telemetry_readers() {
    auto p = platform(ROBONODE_APPS);
    std::atomic<bool> stop{false};
    std::atomic<int> reads{0};
    std::vector<std::thread> readers;
    for (int i = 0; i < 3; ++i) {
        readers.emplace_back([&] {
            while (!stop.load()) {
                (void)p.telemetry_json();
                (void)p.model_json();
                reads.fetch_add(1);
            }
        });
    }
    CHECK(p.set_version("planner", "robonode.avoid").ok());
    // Planning must actually HAPPEN, or this test proves only that readers can
    // read: a move that silently failed every time would pass it.
    int planned = 0;
    for (int i = 0; i < 3; ++i) {
        if (p.move_l(0.9, 0.25, 0.45 + 0.02 * i).ok() && p.await_settled(kProgramTimeout).ok()) {
            ++planned;
        }
    }
    CHECK(planned > 0);
    stop.store(true);
    for (auto& r : readers) r.join();
    CHECK(reads.load() > 50);
    CHECK(!json::parse(p.telemetry_json(), nullptr, false).is_discarded());
}

// Every HTTP request resolves its session, and each is answered on its own
// thread. The map of open sessions was written from all of them at once, which
// corrupts the tree rather than losing an entry — a crash inside std::map's
// rebalance, nowhere near the code that caused it.
void test_sessions_are_opened_from_many_threads_at_once() {
    auto p = platform(ROBONODE_APPS);
    std::atomic<int> reads{0};
    std::vector<std::thread> callers;
    for (int i = 0; i < 6; ++i) {
        callers.emplace_back([&p, &reads, i] {
            for (int n = 0; n < 40; ++n) {
                const auto session = "s" + std::to_string((i * 40 + n) % 12);
                (void)p.docs_json("apps", session);
                (void)p.docs_json("robots", session);
                reads.fetch_add(1);
            }
        });
    }
    for (auto& c : callers) c.join();
    CHECK(reads.load() == 240);
    CHECK(!json::parse(p.sessions_json(), nullptr, false).is_discarded());
}

// A cell can be dropped while a request is reading it. A reference into the
// manager's map would be a use-after-free the moment the map let go; a shared
// pointer keeps the cell alive for as long as the work in flight needs it.
void test_a_cell_can_be_dropped_while_it_is_being_read() {
    auto p = platform(ROBONODE_APPS);
    std::atomic<bool> stop{false};
    std::atomic<int> reads{0};
    std::vector<std::thread> readers;
    for (int i = 0; i < 3; ++i) {
        readers.emplace_back([&] {
            while (!stop.load()) {
                (void)p.cells_json();
                (void)p.telemetry_json();
                reads.fetch_add(1);
            }
        });
    }
    for (int i = 0; i < 5; ++i) {
        CHECK(p.add_cell("churn", "ur10e-fixed.cell.json").ok());
        CHECK(p.remove_cell("churn").ok());
    }
    stop.store(true);
    for (auto& r : readers) r.join();
    CHECK(reads.load() > 10);
    CHECK(json::parse(p.cells_json(), nullptr, false).size() == 1);
}

}  // namespace

int main() {
    test_readers_do_not_race_the_physics();
    test_tool_and_stations_do_not_race_the_idle_ticker();
    test_planning_does_not_race_the_telemetry_readers();
    test_sessions_are_opened_from_many_threads_at_once();
    test_a_cell_can_be_dropped_while_it_is_being_read();
    std::puts("gateway_concurrency_tests: OK");
    return 0;
}
