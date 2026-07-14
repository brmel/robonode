#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "robonode/core/limits.hpp"
#include "robonode/core/status.hpp"
#include "robonode/motion/axis_adapter.hpp"

namespace robonode {

// Everything a factory needs to build an adapter — core types only, never a
// vendor type. `config` carries driver-specific strings (e.g. robot_ip) so
// the registry surface stays uniform across sim, UR, EtherCAT, …
struct DriverContext {
    std::string id;
    AxisLimits limits;
    std::map<std::string, std::string> config;
};

using DriverFactory = std::function<std::unique_ptr<AxisAdapter>(const DriverContext&)>;

// Driver-name → adapter factory. Deliberately NOT a global: a Cell owns one
// and apps populate it with exactly the drivers they link. This is the seam
// that keeps celld vendor-blind — adapters register from the outside (their
// register_* helpers), celld only ever sees AxisAdapter. New hardware = a
// new module that registers here; celld is untouched.
class DriverRegistry {
public:
    void register_driver(std::string name, DriverFactory factory) {
        factories_[std::move(name)] = std::move(factory);
    }

    [[nodiscard]] bool has(const std::string& name) const {
        return factories_.find(name) != factories_.end();
    }

    // Registered driver names, sorted — the versions a node can be swapped to.
    [[nodiscard]] std::vector<std::string> names() const {
        std::vector<std::string> out;
        out.reserve(factories_.size());
        for (const auto& [name, _] : factories_) out.push_back(name);
        return out;  // std::map iterates sorted
    }

    // Builds an adapter for `name` from `ctx`; failure Status if the driver
    // is unknown or its factory returned null.
    Status make(const std::string& name, const DriverContext& ctx,
                std::unique_ptr<AxisAdapter>& out) const {
        const auto it = factories_.find(name);
        if (it == factories_.end()) {
            return Status::failure("unknown driver '" + name + "'");
        }
        out = it->second(ctx);
        if (!out) return Status::failure("driver '" + name + "' factory returned null");
        return Status::success();
    }

private:
    std::map<std::string, DriverFactory> factories_;
};

}  // namespace robonode
