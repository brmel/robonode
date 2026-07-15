// robonode — drive the cell headless, the same facade the web UI binds to
// (ADR-8, #43). Every verb maps 1:1 to robonode::Platform, so an agent does
// everything a human does in the browser, with machine-readable JSON output.
//
//   robonode nodes                     # the node tree (JSON)
//   robonode run                       # run the coordinated move, print result
//   robonode swap <node> <driver>      # swap one node's version, live
//   robonode family <physics|sim>      # rebuild the cell under a driver family
//   robonode telemetry                 # current live I/O snapshot
//
// Self-contained: it boots its own in-process Platform (no server needed).

#include <chrono>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include "robonode/celld/app_descriptor.hpp"
#include "robonode/platform.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must point at the sim-mujoco worlds dir"
#endif
#ifndef ROBONODE_CELL
#error "ROBONODE_CELL must point at the cell descriptor JSON"
#endif

namespace {

int fail(const robonode::Status& st) {
    std::fprintf(stderr, "error: %s\n", st.message().c_str());
    return 1;
}

// Commands are applied by the worker thread, so read-back is eventually
// consistent: poll until `done` holds or timeout. Returns whether it landed.
bool wait_until(const std::function<std::string()>& snapshot,
                const std::function<bool(const nlohmann::json&)>& done,
                std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto j = nlohmann::json::parse(snapshot(), nullptr, false);
        if (!j.is_discarded() && done(j)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds{50});
    }
    return false;
}

bool node_has_driver(const nlohmann::json& tree, const std::string& id, const std::string& drv) {
    for (const auto& n : tree.at("nodes")) {
        if (n.at("id") == id) return n.at("driver") == drv;
    }
    return false;
}

// Run one application step over the facade (#56), waiting for it to land.
bool run_step(robonode::Platform& p, const robonode::AppStep& s) {
    using namespace std::chrono_literals;
    if (s.verb == "run") {
        if (!p.run().ok()) return false;
        return wait_until([&] { return p.telemetry_json(); },
                          [](const nlohmann::json& j) {
                              const auto& pos = j.at("pos");
                              return !pos.empty() && std::abs(double(pos[0]) - 400.0) < 20.0;
                          },
                          20s);
    }
    if (s.verb == "family") {
        const auto it = s.args.find("family");
        if (it == s.args.end() || !p.set_family(it->second).ok()) return false;
        return wait_until([&] { return p.nodes_json(); },
                          [&](const nlohmann::json& j) { return j.at("family") == it->second; }, 5s);
    }
    if (s.verb == "swap") {
        const auto n = s.args.find("node"), d = s.args.find("driver");
        if (n == s.args.end() || d == s.args.end()) return false;
        if (!p.set_node_driver(n->second, d->second).ok()) return false;
        return wait_until([&] { return p.nodes_json(); },
                          [&](const nlohmann::json& j) { return node_has_driver(j, n->second, d->second); },
                          5s);
    }
    std::fprintf(stderr, "unknown program verb: %s\n", s.verb.c_str());
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    CLI::App app{"robonode — headless cell control (same facade as the UI)"};
    app.require_subcommand(1);
    bool json = true;  // machine output is the default; agents live here
    app.add_flag("--json,!--no-json", json, "machine-readable JSON output");

    auto* c_nodes = app.add_subcommand("nodes", "print the node tree");
    auto* c_run = app.add_subcommand("run", "run the coordinated move");
    auto* c_telem = app.add_subcommand("telemetry", "print the live I/O snapshot");
    auto* c_logs = app.add_subcommand("logs", "print recent log records");

    std::string node, driver;
    auto* c_swap = app.add_subcommand("swap", "swap one node's driver version");
    c_swap->add_option("node", node, "node id")->required();
    c_swap->add_option("driver", driver, "driver/version name")->required();

    std::string family;
    auto* c_family = app.add_subcommand("family", "rebuild the cell under a driver family");
    c_family->add_option("family", family, "physics | sim")->required();

    std::string appfile;
    auto* c_app = app.add_subcommand("app", "run an application (a program of steps, #56)");
    c_app->add_option("file", appfile, "app descriptor JSON")->required();

    CLI11_PARSE(app, argc, argv);

    using namespace std::chrono_literals;
    robonode::Platform p{std::string{ROBONODE_WORLDS} + "/rail_ur10e.xml", ROBONODE_CELL};

    if (*c_nodes) {
        std::printf("%s\n", p.nodes_json().c_str());
    } else if (*c_telem) {
        std::printf("%s\n", p.telemetry_json().c_str());
    } else if (*c_logs) {
        const auto arr = nlohmann::json::parse(p.logs_json(), nullptr, false);
        if (!arr.is_discarded()) {
            for (const auto& l : arr) std::printf("%s\n", l.get<std::string>().c_str());
        }
    } else if (*c_run) {
        if (const auto st = p.run(); !st.ok()) return fail(st);
        wait_until([&] { return p.telemetry_json(); },
                   [](const nlohmann::json& j) {
                       const auto& pos = j.at("pos");
                       return !pos.empty() && std::abs(double(pos[0]) - 400.0) < 20.0;
                   },
                   20s);
        std::printf("%s\n", p.telemetry_json().c_str());
    } else if (*c_swap) {
        if (const auto st = p.set_node_driver(node, driver); !st.ok()) return fail(st);
        if (!wait_until([&] { return p.nodes_json(); },
                        [&](const nlohmann::json& j) { return node_has_driver(j, node, driver); },
                        5s)) {
            std::fprintf(stderr, "error: swap of '%s' to '%s' did not apply (unknown driver?)\n",
                         node.c_str(), driver.c_str());
            return 1;
        }
        std::printf("%s\n", p.nodes_json().c_str());
    } else if (*c_family) {
        if (const auto st = p.set_family(family); !st.ok()) return fail(st);
        wait_until([&] { return p.nodes_json(); },
                   [&](const nlohmann::json& j) { return j.at("family") == family; }, 5s);
        std::printf("%s\n", p.nodes_json().c_str());
    } else if (*c_app) {
        robonode::AppDescriptor ad;
        if (const auto st = robonode::load_app_descriptor(appfile, ad); !st.ok()) return fail(st);
        std::fprintf(stderr, "app: %s — %zu step(s) on cell %s\n", ad.name.c_str(),
                     ad.program.size(), ad.cell.c_str());
        for (const auto& step : ad.program) {
            if (!run_step(p, step)) {
                std::fprintf(stderr, "error: step '%s' failed\n", step.verb.c_str());
                return 1;
            }
        }
        std::printf("%s\n", p.telemetry_json().c_str());
    }
    (void)json;  // outputs are JSON today; --no-json reserved for a human mode
    return 0;
}
