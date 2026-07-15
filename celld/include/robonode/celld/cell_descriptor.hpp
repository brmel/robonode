#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/limits.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// One node in a cell descriptor. The unit is explicit (mm / rad), so the limit
// numbers stay unitless (ADR/#53); driver-specific placement (joint/actuator/
// scale) is data too. The driver itself is chosen at build time (family swap),
// not baked here.
struct CellNodeSpec {
    std::string id, joint, actuator, unit;
    double units_per_m{1.0};
    AxisLimits limits{};
};

// A whole cell as data (#28): the node tree the gateway builds, loaded from a
// file instead of a hardcoded array. No limits, ids, or placement live in code.
struct CellDescriptor {
    std::vector<CellNodeSpec> nodes;
};

namespace detail {
inline AxisLimits parse_limits(const nlohmann::json& l) {
    return {l.at("position_min").get<double>(), l.at("position_max").get<double>(),
            l.at("velocity_max").get<double>(), l.at("acceleration_max").get<double>(),
            l.value("jerk_max", 0.0)};
}
}  // namespace detail

inline Status load_cell_descriptor(const std::string& path, CellDescriptor& out) {
    std::ifstream f{path};
    if (!f) return Status::failure("cannot open cell descriptor: " + path);
    const auto j = nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return Status::failure("invalid JSON: " + path);

    try {
        for (const auto& n : j.at("nodes")) {
            CellNodeSpec s;
            s.id = n.at("id").get<std::string>();
            s.joint = n.value("joint", "");
            s.actuator = n.value("actuator", "");
            s.unit = n.value("unit", "");
            s.units_per_m = n.value("units_per_m", 1.0);
            s.limits = detail::parse_limits(n.at("limits"));
            out.nodes.push_back(std::move(s));
        }
    } catch (const nlohmann::json::exception& e) {
        return Status::failure(std::string{path} + ": " + e.what());
    }
    if (out.nodes.empty()) return Status::failure(path + ": no nodes");
    return Status::success();
}

}  // namespace robonode
