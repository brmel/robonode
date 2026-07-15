#pragma once

#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "robonode/celld/app_store.hpp"
#include "robonode/core/status.hpp"
#include "robonode/gateway/cell_gateway.hpp"

namespace robonode {

// The one clean entry (#33): the MIL-style facade the web UI, the CLI (#43),
// and the SDK all bind to. Typed verbs over the cell — callers never hand-craft
// command JSON, and celld/motion/gateway internals never leak. Thin: it owns a
// CellGateway and speaks the same command contract, so every surface shares one
// definition of what the platform can do (ADR-8).
class Platform {
public:
    Platform(std::string world_path, const std::string& cell_path, std::string apps_dir = "")
        : gw_{std::move(world_path), cell_path}, store_{std::move(apps_dir)} {}

    // --- action verbs (typed) ---
    Status run() { return apply({{"cmd", "run"}}); }
    Status set_family(const std::string& family) {
        return apply({{"cmd", "driver"}, {"family", family}});
    }
    Status set_node_driver(const std::string& node, const std::string& driver) {
        return apply({{"cmd", "set_driver"}, {"node", node}, {"driver", driver}});
    }
    // Cartesian move to a TCP target (x,y,z in metres) — real IK behind the seam.
    Status move_l(double x, double y, double z) {
        return apply({{"cmd", "move_l"}, {"x", x}, {"y", y}, {"z", z}});
    }
    // Deploy a saved app (#61): load its program and run it on the worker (#64).
    Status run_app(const std::string& file) {
        std::string body;
        if (const auto st = store_.load(file, body); !st.ok()) return st;
        const auto app = nlohmann::json::parse(body, nullptr, false);
        if (app.is_discarded() || !app.contains("program")) return Status::failure("bad app");
        return apply({{"cmd", "run_app"}, {"program", app.at("program")}});
    }

    // --- observation surface (the same JSON every surface renders) ---
    std::string nodes_json() { return gw_.nodes_json(); }
    std::string telemetry_json() { return gw_.telemetry_json(); }
    std::string logs_json() { return gw_.logs_json(); }
    std::string vision_json() { return gw_.vision_json(); }

    // Application store (#59): the saved apps a user can deploy or edit.
    std::string apps_json() { return store_.list_json(); }
    Status load_app(const std::string& file, std::string& out) { return store_.load(file, out); }
    Status save_app(const std::string& file, const std::string& json) {
        return store_.save(file, json);
    }

    // Transport escape hatch: the HTTP layer forwards raw command bodies here so
    // the wire contract lives in one place. Deploying an app arrives as
    // {"cmd":"run_app","file":X} — resolve it through the same run_app the CLI
    // uses (load the program from the store), so both surfaces share one path.
    std::string submit_command(const std::string& body) {
        const auto j = nlohmann::json::parse(body, nullptr, false);
        if (!j.is_discarded() && j.value("cmd", "") == "run_app" && j.contains("file")) {
            const auto st = run_app(j.at("file").get<std::string>());
            return st.ok() ? std::string{R"({"ok":true})"}
                           : nlohmann::json{{"ok", false}, {"error", st.message()}}.dump();
        }
        return gw_.submit_command(body);
    }

private:
    Status apply(const nlohmann::json& cmd) {
        const auto r = nlohmann::json::parse(gw_.submit_command(cmd.dump()), nullptr, false);
        if (!r.is_discarded() && r.value("ok", false)) return Status::success();
        return Status::failure(r.is_discarded() ? "bad response"
                                                : r.value("error", "command rejected"));
    }

    CellGateway gw_;
    AppStore store_;
};

}  // namespace robonode
