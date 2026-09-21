#pragma once

#include <fstream>
#include <map>
#include <string>

#include <nlohmann/json.hpp>

#include "robonode/core/limits.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// The runtime face of robonode-idl's NodeDescriptor (v0 subset: one
// MotionAxis capability). This is FR-1.3 made real: limits enter the
// process as data from the descriptor file — change the JSON, get a
// different envelope, no recompile.
struct Descriptor {
    std::string id;
    std::string driver;
    AxisLimits limits{};
    double command_rate_hz{};
    bool simulated{};
    // Driver-specific config (e.g. robot_ip for a hardware adapter). Opaque
    // to celld — passed straight through to the driver factory (FR-1.3: even
    // connection details are data, not code).
    std::map<std::string, std::string> config;
};

inline Status load_descriptor(const std::string& path, Descriptor& out) {
    std::ifstream f{path};
    if (!f) return Status::failure("cannot open descriptor: " + path);

    nlohmann::json j = nlohmann::json::parse(f, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return Status::failure("invalid JSON: " + path);

    try {
        out.id = j.at("id").get<std::string>();
        out.driver = j.at("driver").at("name").get<std::string>();
        out.simulated = j.value("simulated", false);
        const auto& cap = j.at("capabilities").at(0);
        if (cap.at("type").get<std::string>() != "MotionAxis@1") {
            return Status::failure(path + ": first capability is not MotionAxis@1");
        }
        out.command_rate_hz = cap.at("command_rate_hz").get<double>();
        const auto& lim = cap.at("limits");
        // The in-memory AxisLimits is unitless (a joint's fields hold radians);
        // the DESCRIPTOR keys keep the shop unit hint (mm) — units live in the
        // descriptor, not the type (FR-1.3, ADR/#53).
        out.limits.position_min = lim.at("position_min_mm").get<double>();
        out.limits.position_max = lim.at("position_max_mm").get<double>();
        out.limits.velocity_max = lim.at("velocity_max_mm_s").get<double>();
        out.limits.acceleration_max = lim.at("acceleration_max_mm_s2").get<double>();
        out.limits.jerk_max = lim.value("jerk_max_mm_s3", 0.0);
        // Optional driver config: string values as-is, others stringified.
        if (auto it = j.find("config"); it != j.end() && it->is_object()) {
            for (const auto& [k, v] : it->items()) {
                out.config[k] = v.is_string() ? v.get<std::string>() : v.dump();
            }
        }
    } catch (const nlohmann::json::exception& e) {
        return Status::failure(path + ": " + e.what());
    }
    if (out.limits.velocity_max <= 0.0 || out.limits.acceleration_max <= 0.0 ||
        out.limits.position_max <= out.limits.position_min || out.command_rate_hz <= 0.0) {
        return Status::failure(path + ": non-physical limits");
    }
    return Status::success();
}

}  // namespace robonode
