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

// The one program parser. A program arrives either inside a saved app or
// directly on the wire; both land here, so the two paths cannot disagree about
// what a step is.
inline std::vector<AppStep> parse_program(const nlohmann::json& program) {
    std::vector<AppStep> out;
    if (!program.is_array()) return out;
    for (const auto& s : program) {
        AppStep step;
        step.verb = s.value("verb", "");
        if (const auto args = s.find("args"); args != s.end() && args->is_object()) {
            for (const auto& [k, v] : args->items()) {
                step.args[k] = v.is_string() ? v.get<std::string>() : v.dump();
            }
        }
        if (!step.verb.empty()) out.push_back(std::move(step));
    }
    return out;
}

inline Status parse_app_descriptor(const nlohmann::json& j, AppDescriptor& out,
                                   const std::string& source = "app") {
    if (j.is_discarded() || !j.is_object()) return Status::failure("invalid JSON: " + source);
    if (!j.contains("program")) return Status::failure(source + ": no program");
    out.name = j.value("name", source);
    out.cell = j.value("cell", "");
    out.program = parse_program(j.at("program"));
    if (out.program.empty()) return Status::failure(source + ": empty program");
    return Status::success();
}

inline Status load_app_descriptor(const std::string& path, AppDescriptor& out) {
    std::ifstream f{path};
    if (!f) return Status::failure("cannot open app: " + path);
    return parse_app_descriptor(nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false), out,
                                path);
}

}  // namespace robonode
