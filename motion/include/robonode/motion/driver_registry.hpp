#pragma once

#include <map>
#include <memory>
#include <string>

#include "robonode/core/limits.hpp"
#include "robonode/motion/axis_adapter.hpp"
#include "robonode/motion/module_registry.hpp"

namespace robonode {

// Everything a factory needs to build an adapter — core types only, never a
// vendor type. `config` carries driver-specific strings (e.g. robot_ip) so the
// registry surface stays uniform across sim, UR, EtherCAT, …
struct DriverContext {
    std::string id;
    AxisLimits limits;
    std::map<std::string, std::string> config;
};

using DriverFactory = ModuleRegistry<AxisAdapter, DriverContext>::Factory;

// The MotionAxis version registry: a ModuleRegistry (#30) specialised for axis
// drivers. `register_driver` is kept as the axis-flavoured name the register_*
// helpers use; everything else (has/names/make) is the generic mechanism, so
// Kinematics/Planner/Vision get the same "try each version" for free.
class DriverRegistry : public ModuleRegistry<AxisAdapter, DriverContext> {
public:
    void register_driver(std::string name, DriverFactory factory) {
        add(std::move(name), std::move(factory));
    }
};

}  // namespace robonode
