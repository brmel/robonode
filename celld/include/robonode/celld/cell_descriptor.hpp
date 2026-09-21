#pragma once

#include <algorithm>
#include <fstream>
#include <functional>
#include <map>
#include <set>
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

// A station node in the cell (#29): a moving fixture the robot works with —
// conveyor / deck / pallet. A capability MODULE like any node (its physics +
// versions come with #31); here it is data alongside the robot's joints.
struct StationSpec {
    std::string id, type;                          // type: conveyor | deck | pallet
    std::map<std::string, std::string> config;     // e.g. speed, joint, actuator
};

// A robot as a NODE (the product model): a whole arm, whose joints are its
// internal detail. `joints` are the axis ids it articulates, in kinematic
// order; `carrier` are the axes that place it (a 7th-axis rail, a gantry) and
// are held during a Cartesian move. Naming the axes by id is what lets the
// platform stop indexing `nodes[1..6]` — a second robot is a second entry.
struct RobotSpec {
    std::string id;
    std::vector<std::string> carrier;
    std::vector<std::string> joints;
    std::string tcp_site{"tcp"};
};

// A named coordinated move, keyed by axis id so each axis carries its own unit
// and nothing depends on node ordering.
struct MotionSpec {
    std::string id;
    double settle_s{0.5};
    std::map<std::string, std::vector<double>> waypoints;
};

// A driver family: the name a surface offers ("physics", "sim") and the driver
// version every axis is built with when it is selected. Which engine backs a
// family is a property of the cell, not a branch in the gateway.
struct FamilySpec {
    std::string id, driver;
};

// A whole cell as data (#28/#29): the node tree the gateway builds, loaded from
// a file instead of a hardcoded array — robot joints AND stations. No ids,
// limits, or placement live in code.
struct CellDescriptor {
    std::string world;  // the physics scene this cell lives in (MJCF, relative to the worlds dir)
    std::vector<CellNodeSpec> nodes;
    std::vector<StationSpec> stations;
    std::vector<RobotSpec> robots;
    std::vector<MotionSpec> motions;
    std::vector<FamilySpec> families;

    [[nodiscard]] const FamilySpec* family(const std::string& id) const {
        for (const auto& f : families) {
            if (f.id == id) return &f;
        }
        return nullptr;
    }
};

namespace detail {

inline AxisLimits parse_limits(const nlohmann::json& l) {
    return {l.at("position_min").get<double>(), l.at("position_max").get<double>(),
            l.at("velocity_max").get<double>(), l.at("acceleration_max").get<double>(),
            l.value("jerk_max", 0.0)};
}

inline std::map<std::string, std::string> parse_config(const nlohmann::json& obj) {
    std::map<std::string, std::string> out;
    for (const auto& [k, v] : obj.items()) out[k] = v.is_string() ? v.get<std::string>() : v.dump();
    return out;
}

inline CellNodeSpec parse_node(const nlohmann::json& n) {
    CellNodeSpec s;
    s.id = n.at("id").get<std::string>();
    s.joint = n.value("joint", "");
    s.actuator = n.value("actuator", "");
    s.unit = n.value("unit", "");
    s.units_per_m = n.value("units_per_m", 1.0);
    s.limits = parse_limits(n.at("limits"));
    return s;
}

inline StationSpec parse_station(const nlohmann::json& s) {
    StationSpec st;
    st.id = s.at("id").get<std::string>();
    st.type = s.at("type").get<std::string>();
    if (const auto c = s.find("config"); c != s.end() && c->is_object()) {
        st.config = parse_config(*c);
    }
    return st;
}

inline RobotSpec parse_robot(const nlohmann::json& r) {
    RobotSpec rs;
    rs.id = r.at("id").get<std::string>();
    rs.carrier = r.value("carrier", std::vector<std::string>{});
    rs.joints = r.at("joints").get<std::vector<std::string>>();
    rs.tcp_site = r.value("tcp_site", std::string{"tcp"});
    return rs;
}

inline FamilySpec parse_family(const nlohmann::json& f) {
    return {f.at("id").get<std::string>(), f.at("driver").get<std::string>()};
}

inline MotionSpec parse_motion(const nlohmann::json& m) {
    MotionSpec ms;
    ms.id = m.at("id").get<std::string>();
    ms.settle_s = m.value("settle_s", 0.5);
    for (const auto& [axis, wp] : m.at("waypoints").items()) {
        ms.waypoints[axis] = wp.get<std::vector<double>>();
    }
    return ms;
}

// A document is edited by hand, so a parse error is a message to a person: it
// has to say WHERE. nlohmann reports the missing key and nothing else, which
// reads like a bug in the platform rather than a mistake in the file — so each
// element is parsed inside its own position ("nodes[2] 'j2'") and any throw is
// re-raised with it.
template <class T, class Parse>
void parse_each(const nlohmann::json& j, const char* key, std::vector<T>& out, Parse parse) {
    const auto it = j.find(key);
    if (it == j.end() || !it->is_array()) return;
    std::size_t index = 0;
    for (const auto& e : *it) {
        const std::string named =
            e.is_object() && e.contains("id") && e.at("id").is_string()
                ? " '" + e.at("id").get<std::string>() + "'"
                : "";
        const std::string where = std::string{key} + "[" + std::to_string(index++) + "]" + named;
        try {
            out.push_back(parse(e));
        } catch (const nlohmann::json::exception& ex) {
            throw nlohmann::json::other_error::create(501, where + ": " + ex.what(), &e);
        }
    }
}

inline bool has_axis(const CellDescriptor& d, const std::string& id) {
    return std::any_of(d.nodes.begin(), d.nodes.end(),
                       [&](const CellNodeSpec& n) { return n.id == id; });
}

inline Status check_axis_refs(const CellDescriptor& d, const std::string& owner,
                              const std::vector<std::string>& ids) {
    for (const auto& id : ids) {
        if (!has_axis(d, id)) return Status::failure(owner + " names unknown axis '" + id + "'");
    }
    return Status::success();
}

inline Status validate(const CellDescriptor& d) {
    if (d.world.empty()) return Status::failure("no world declared");
    if (d.nodes.empty()) return Status::failure("no nodes");
    if (d.families.empty()) return Status::failure("no driver families declared");

    // A descriptor names its axes in four places (the node list, a robot's
    // carrier and joints, a motion's waypoints). Nothing else checks they agree,
    // so a hand-edited robot that drops an axis from one section loads happily
    // and fails somewhere unrelated later. Every reference is resolved here.
    std::set<std::string> seen;
    for (const auto& n : d.nodes) {
        if (!seen.insert(n.id).second) return Status::failure("two nodes share the id '" + n.id + "'");
    }
    for (const auto& r : d.robots) {
        const std::string owner = "robot '" + r.id + "'";
        if (const auto st = check_axis_refs(d, owner, r.carrier); !st.ok()) return st;
        if (const auto st = check_axis_refs(d, owner, r.joints); !st.ok()) return st;
        if (r.joints.empty()) return Status::failure(owner + " articulates no joints");
        // An axis that both carries the arm and belongs to it would be driven
        // twice by one Cartesian solve.
        for (const auto& c : r.carrier) {
            if (std::find(r.joints.begin(), r.joints.end(), c) != r.joints.end()) {
                return Status::failure(owner + " lists '" + c + "' as both carrier and joint");
            }
        }
    }
    for (const auto& m : d.motions) {
        std::size_t steps = 0;
        for (const auto& [axis, waypoints] : m.waypoints) {
            if (!has_axis(d, axis)) {
                return Status::failure("motion '" + m.id + "' names unknown axis '" + axis + "'");
            }
            // Waypoints are driven as one coordinated move: axes that disagree
            // about how many steps there are cannot be blended together.
            if (steps == 0) steps = waypoints.size();
            if (waypoints.size() != steps) {
                return Status::failure("motion '" + m.id + "': axis '" + axis + "' has " +
                                       std::to_string(waypoints.size()) + " waypoints, others have " +
                                       std::to_string(steps));
            }
        }
    }
    return Status::success();
}

}  // namespace detail

// The one descriptor parser. A robot arrives from a file at boot and from the
// document store when a user saves one; both land here, so the two paths cannot
// disagree about what a valid cell is.
inline Status parse_cell_descriptor(const nlohmann::json& j, CellDescriptor& out,
                                    const std::string& source = "cell") {
    if (j.is_discarded() || !j.is_object()) return Status::failure("invalid JSON: " + source);
    try {
        out.world = j.value("world", "");
        detail::parse_each(j, "nodes", out.nodes, detail::parse_node);
        detail::parse_each(j, "stations", out.stations, detail::parse_station);
        detail::parse_each(j, "robots", out.robots, detail::parse_robot);
        detail::parse_each(j, "motions", out.motions, detail::parse_motion);
        detail::parse_each(j, "families", out.families, detail::parse_family);
    } catch (const nlohmann::json::exception& e) {
        return Status::failure(source + ": " + e.what());
    }
    if (const auto st = detail::validate(out); !st.ok()) {
        return Status::failure(source + ": " + st.message());
    }
    return Status::success();
}

inline Status load_cell_descriptor(const std::string& path, CellDescriptor& out) {
    std::ifstream f{path};
    if (!f) return Status::failure("cannot open cell descriptor: " + path);
    return parse_cell_descriptor(nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false), out,
                                 path);
}

}  // namespace robonode
