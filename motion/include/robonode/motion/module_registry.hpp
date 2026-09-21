#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/core/status.hpp"

namespace robonode {

// Generic version registry for ANY capability (#30): name -> factory<T(Ctx)>.
// Every node — MotionAxis, Kinematics, Planner, Vision, Station — gets the same
// "list the versions / build one / swap live" mechanism the axis drivers had,
// so "try each version" is real for every capability, not just axes.
//
// Deliberately NOT a global: an owner holds one registry per capability and
// populates it with exactly the versions it links. Versions register from the
// outside (their register_* helpers); the owner only ever sees T.
template <class T, class Ctx>
class ModuleRegistry {
public:
    using Factory = std::function<std::unique_ptr<T>(const Ctx&)>;

    void add(std::string name, Factory factory) {
        factories_[std::move(name)] = std::move(factory);
    }

    [[nodiscard]] bool has(const std::string& name) const {
        return factories_.find(name) != factories_.end();
    }

    // Registered version names, sorted — the versions a node can be swapped to.
    [[nodiscard]] std::vector<std::string> names() const {
        std::vector<std::string> out;
        out.reserve(factories_.size());
        for (const auto& [name, _] : factories_) out.push_back(name);
        return out;  // std::map iterates sorted
    }

    Status make(const std::string& name, const Ctx& ctx, std::unique_ptr<T>& out) const {
        const auto it = factories_.find(name);
        if (it == factories_.end()) return Status::failure("unknown version '" + name + "'");
        out = it->second(ctx);
        if (!out) return Status::failure("version '" + name + "' factory returned null");
        return Status::success();
    }

private:
    std::map<std::string, Factory> factories_;
};

}  // namespace robonode
