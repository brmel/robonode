#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "robonode/core/status.hpp"
#include "robonode/gateway/command_bus.hpp"

namespace robonode {

// A queued unit of work: validated and built on the calling thread, run on the
// worker. `key` coalesces repeats of the same idempotent command.
struct Command {
    std::string name;
    std::string key;
    std::function<Status()> run;
    nlohmann::json ack;  // extra fields the verb wants in its acknowledgement

    // Stop and e-stop do their work on the calling thread and queue only an
    // acknowledgement — so the thing that reaches the worker is the ack for a
    // latch that has ALREADY been set. Sending it through the same gate as new
    // motion means the latch refuses its own receipt: `robonode estop` did
    // exactly what it was asked, and reported failure, and left the cell
    // faulted instead of held.
    bool starts_motion{true};
};

using CommandAck = CommandBus<Command>::Ack;

// The reply to an accepted (or refused) command. Defined once: a caller must
// get the same shape whether the command arrived on the wire or through a
// typed facade verb, or `applied_id` is unobservable on one of those paths.
inline std::string ack_json(const CommandAck& ack, nlohmann::json extra = nlohmann::json::object()) {
    if (!ack.accepted) return nlohmann::json{{"ok", false}, {"error", ack.reason}}.dump();
    nlohmann::json out{{"ok", true}, {"id", ack.id}, {"queued", ack.depth}};
    if (extra.is_object()) {
        for (auto& el : extra.items()) out[el.key()] = el.value();
    }
    return out.dump();
}

// The one place the wire contract lives. Verbs register a builder that
// validates arguments and produces the work; nothing downstream parses JSON.
class CommandRouter {
public:
    using Builder = std::function<Status(const nlohmann::json&, Command&)>;
    using Submit = std::function<CommandAck(Command)>;

    explicit CommandRouter(Submit submit) : submit_{std::move(submit)} {}

    void on(std::string verb, Builder build) { builders_[std::move(verb)] = std::move(build); }

    [[nodiscard]] std::string dispatch(const std::string& body) const {
        const auto j = nlohmann::json::parse(body, nullptr, false);
        if (j.is_discarded() || !j.contains("cmd")) return error("bad command");
        return dispatch(j.value("cmd", ""), j);
    }

    [[nodiscard]] std::string dispatch(const std::string& verb, const nlohmann::json& args) const {
        const auto it = builders_.find(verb);
        if (it == builders_.end()) return error("unknown cmd '" + verb + "'");

        Command cmd;
        cmd.name = verb;
        const auto& payload = args.is_object() ? args : kNoArgs;
        if (const auto st = it->second(payload, cmd); !st.ok()) return error(st.message());

        nlohmann::json extra = std::move(cmd.ack);
        return ack_json(submit_(std::move(cmd)), std::move(extra));
    }

private:
    inline static const nlohmann::json kNoArgs = nlohmann::json::object();

    static std::string error(const std::string& why) {
        return nlohmann::json{{"ok", false}, {"error", why}}.dump();
    }

    Submit submit_;
    std::map<std::string, Builder> builders_;
};

}  // namespace robonode
