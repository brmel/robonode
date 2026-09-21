#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/status.hpp"
#include "robonode/motion/module_registry.hpp"
#include "robonode/sandbox/compiler.hpp"
#include "robonode/sandbox/program.hpp"

namespace robonode {

// What a swappable capability IS, as data: how it presents, whether users may
// author versions, and the sandbox ABI they author against. Served to every
// surface (GET /capabilities) so no client hardcodes it.
struct CapabilityDescriptor {
    std::string id, title, icon, hint, source_template;
    bool authorable{false};
    std::vector<std::string> abi_inputs;
    std::size_t abi_outputs{0};

    [[nodiscard]] nlohmann::json to_json() const {
        return {{"id", id},
                {"title", title},
                {"icon", icon},
                {"hint", hint},
                {"authorable", authorable},
                {"template", source_template},
                {"abi", {{"inputs", abi_inputs}, {"outputs", abi_outputs}}}};
    }
};

// Where a version came from — what makes a Compare result reproducible.
struct VersionInfo {
    std::string id, origin, source_hash;

    [[nodiscard]] nlohmann::json to_json() const {
        return {{"id", id}, {"origin", origin}, {"source_hash", source_hash}};
    }
};

// FNV-1a: identifies a source revision. Not a security digest, not named as one.
inline std::string source_hash(const std::string& s) {
    std::uint64_t h = 1469598103934665603ULL;
    for (const unsigned char c : s) h = (h ^ c) * 1099511628211ULL;
    std::ostringstream os;
    os << std::hex << h;
    return os.str();
}

// Whether a freshly installed version becomes the live one. A caller that
// reads `install(id, program, Select::kNow)` needs no comment to know.
enum class Select { kNow, kLater };

// The non-template face of a capability: what the registry, the command router
// and every surface talk to.
class CapabilityBase {
public:
    virtual ~CapabilityBase() = default;
    [[nodiscard]] virtual const CapabilityDescriptor& descriptor() const = 0;
    virtual Status select(const std::string& version) = 0;
    virtual void install(const std::string& version, sandbox::Program program,
                         Select select) = 0;
    virtual std::string json() = 0;

    // Compile user source against this capability's declared ABI, on the
    // caller's thread, so a syntax error is reported synchronously.
    Status compile(const std::string& source, sandbox::Program& out) const {
        const auto& d = descriptor();
        if (!d.authorable) return Status::failure("capability '" + d.id + "' is not authorable");
        return sandbox::Compiler::compile(source, d.abi_inputs, d.abi_outputs, out);
    }
};

template <class T, class Ctx>
class Capability : public CapabilityBase {
public:
    [[nodiscard]] const CapabilityDescriptor& descriptor() const override { return desc_; }

    std::string json() override {
        std::lock_guard<std::mutex> lk{mtx_};
        return snap_;
    }

protected:
    Capability(CapabilityDescriptor descriptor, std::string version)
        : version_{std::move(version)}, desc_{std::move(descriptor)} {}

    void remember(VersionInfo info) { versions_[info.id] = std::move(info); }

    void publish(nlohmann::json extra = nlohmann::json::object()) {
        nlohmann::json j{{"id", desc_.id},
                         {"version", version_},
                         {"available", registry_.names()},
                         {"descriptor", desc_.to_json()},
                         {"provenance", provenance()}};
        for (auto& el : extra.items()) j[el.key()] = el.value();
        std::lock_guard<std::mutex> lk{mtx_};
        snap_ = j.dump();
    }

    ModuleRegistry<T, Ctx> registry_;
    std::string version_;

private:
    [[nodiscard]] nlohmann::json provenance() const {
        auto arr = nlohmann::json::array();
        for (const auto& [_, v] : versions_) arr.push_back(v.to_json());
        return arr;
    }

    CapabilityDescriptor desc_;
    std::map<std::string, VersionInfo> versions_;
    std::mutex mtx_;
    std::string snap_{R"({"version":"","available":[]})"};
};

}  // namespace robonode
