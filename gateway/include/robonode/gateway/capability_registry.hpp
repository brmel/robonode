#pragma once

#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/settings.hpp"
#include "robonode/gateway/capability.hpp"

namespace robonode {

// The capabilities this platform offers, as one addressable set. Adding a
// swappable capability is one add() — no branch in the gateway, no switch in
// the CLI, no constant table in the web app.
class CapabilityRegistry {
public:
    void add(CapabilityBase& capability) { caps_[capability.descriptor().id] = &capability; }

    [[nodiscard]] CapabilityBase* find(const std::string& id) const {
        const auto it = caps_.find(id);
        return it == caps_.end() ? nullptr : it->second;
    }

    [[nodiscard]] std::vector<std::string> ids() const {
        std::vector<std::string> out;
        out.reserve(caps_.size());
        for (const auto& [id, _] : caps_) out.push_back(id);
        return out;
    }

    // Which application exercises a capability when two of its versions are
    // compared. It comes from settings and is published with the capability, so
    // a surface that offers "A vs B" never decides for itself what the work is.
    void publish_trials(Settings::Compare trials) { trials_ = std::move(trials); }

    [[nodiscard]] std::string json() {
        auto arr = nlohmann::json::array();
        for (const auto& [id, _] : caps_) arr.push_back(view(id));
        return arr.dump();
    }

    [[nodiscard]] std::string json_of(const std::string& id) { return view(id).dump(); }

private:
    [[nodiscard]] nlohmann::json view(const std::string& id) {
        auto* cap = find(id);
        if (cap == nullptr) return nlohmann::json::object();
        auto j = nlohmann::json::parse(cap->json(), nullptr, false);
        if (j.is_discarded()) return nlohmann::json::object();
        const auto named = trials_.trial.find(id);
        j["trial"] = named == trials_.trial.end() ? trials_.fallback : named->second;
        return j;
    }

    std::map<std::string, CapabilityBase*> caps_;
    Settings::Compare trials_;
};

}  // namespace robonode
