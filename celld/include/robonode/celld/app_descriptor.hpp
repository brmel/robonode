#pragma once

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/status.hpp"

namespace robonode {

// One step of an application's program (#56/#64): a facade verb + its args.
// v0 verbs map to the Platform: run · family <name> · swap <node> <driver>.
// The full BehaviorTree program engine (branch/wait/detect/pick) lands with
// #64; this is the linear-sequence starting point.
struct AppStep {
    std::string verb;
    std::map<std::string, std::string> args;
};

// An Application as data (#56): a name, the cell it runs on (nodes + stations),
// and a program (task steps). Chosen capability versions/overrides (#60) attach
// here too. An app is a COMPOSITION of the node layer, not new machinery — the
// Platform facade runs it; the library (#57) lists it; the editor (#58) edits it.
struct AppDescriptor {
    std::string name;
    std::string cell;  // cell descriptor filename this app targets
    std::vector<AppStep> program;
};

inline Status load_app_descriptor(const std::string& path, AppDescriptor& out) {
    std::ifstream f{path};
    if (!f) return Status::failure("cannot open app: " + path);
    const auto j = nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return Status::failure("invalid JSON: " + path);

    try {
        out.name = j.at("name").get<std::string>();
        out.cell = j.value("cell", "");
        for (const auto& s : j.at("program")) {
            AppStep step;
            step.verb = s.at("verb").get<std::string>();
            if (auto a = s.find("args"); a != s.end() && a->is_object()) {
                for (const auto& [k, v] : a->items()) {
                    step.args[k] = v.is_string() ? v.get<std::string>() : v.dump();
                }
            }
            out.program.push_back(std::move(step));
        }
    } catch (const nlohmann::json::exception& e) {
        return Status::failure(std::string{path} + ": " + e.what());
    }
    if (out.program.empty()) return Status::failure(path + ": empty program");
    return Status::success();
}

}  // namespace robonode
