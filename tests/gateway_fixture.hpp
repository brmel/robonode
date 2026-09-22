#pragma once

// What every gateway suite needs to stand a platform up: the shipped settings,
// the same engines the server links, and a poll that waits on a published
// snapshot rather than on a sleep. One copy — four suites were about to carry
// four.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <set>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "check.hpp"
#include "robonode/gateway/cell_gateway.hpp"
#include "robonode/apps/vendors.hpp"
#include "robonode/celld/settings_io.hpp"
#include "robonode/platform.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must be defined for the gateway suites"
#endif

namespace robonode::testing {

// Cells name their own scene; the platform is told where scenes live.
const std::string kWorlds = ROBONODE_WORLDS;
const std::string kCell = ROBONODE_CELL;

using nlohmann::json;
using namespace std::chrono_literals;

// Physics runs in real time, so a whole application takes minutes on a loaded
// machine. Tests wait on the platform's completion signal with a generous
// bound rather than pacing themselves against a wall clock.
constexpr auto kProgramTimeout = std::chrono::minutes{5};

// ThreadSanitizer instruments every memory access, so reader threads make far
// fewer passes through the same window. The bounds below exist to prove the
// readers really ran — that a concurrency test is not passing vacuously — not
// to assert a throughput, so they scale with the build rather than being
// relaxed everywhere.
#if defined(__SANITIZE_THREAD__)
#define ROBONODE_TSAN 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define ROBONODE_TSAN 1
#endif
#endif

#ifdef ROBONODE_TSAN
constexpr int kBusyReads = 5;
constexpr int kHammeredReads = 10;
#else
constexpr int kBusyReads = 50;
constexpr int kHammeredReads = 100;
#endif

// A platform composed the way an app composes one: the cell names a scene, so
// the scene library has to be reachable or the cell has no world to live in.
inline robonode::Platform platform(const std::string& apps = {}, const std::string& modules = {}) {
    // The SHIPPED tuning, like a deployment: settings carry which application
    // exercises each capability, so a test that compares versions is testing
    // what an operator gets rather than a fixture of its own.
    return robonode::Platform{kWorlds,
                              kCell,
                              apps,
                              modules,
                              ROBONODE_SCENES,
                              {},
                              robonode::settings_or_defaults(ROBONODE_SETTINGS),
                              robonode::vendor_drivers(),
                              robonode::vision_versions(),
                              ROBONODE_ROBOTS};
}

// The same, one layer down, for tests that drive the gateway directly.
inline robonode::CellGateway gateway() {
    return robonode::CellGateway{kWorlds, kCell, {}, robonode::vendor_drivers(),
                                 robonode::vision_versions(), ROBONODE_SCENES};
}

// One capability's view, as a string. The checked form is the only one — a
// capability that does not exist is a caller error, not an empty answer.
inline std::string view_of(robonode::Platform& p, const std::string& id) {
    std::string out;
    (void)p.capability_json(id, out);
    return out;
}

// Poll a thread-safe snapshot until `pred(parsed_json)` holds or timeout.
template <typename Getter, typename Pred>
inline bool poll_until(Getter get, Pred pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto j = json::parse(get(), nullptr, false);
        if (!j.is_discarded() && pred(j)) return true;
        std::this_thread::sleep_for(50ms);
    }
    return false;
}

}  // namespace robonode::testing
