#pragma once

#include <fstream>
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
        out.limits.position_min_mm = lim.at("position_min_mm").get<double>();
        out.limits.position_max_mm = lim.at("position_max_mm").get<double>();
        out.limits.velocity_max_mm_s = lim.at("velocity_max_mm_s").get<double>();
        out.limits.acceleration_max_mm_s2 = lim.at("acceleration_max_mm_s2").get<double>();
        out.limits.jerk_max_mm_s3 = lim.value("jerk_max_mm_s3", 0.0);
    } catch (const nlohmann::json::exception& e) {
        return Status::failure(path + ": " + e.what());
    }
    if (out.limits.velocity_max_mm_s <= 0.0 || out.limits.acceleration_max_mm_s2 <= 0.0 ||
        out.limits.position_max_mm <= out.limits.position_min_mm || out.command_rate_hz <= 0.0) {
        return Status::failure(path + ": non-physical limits");
    }
    return Status::success();
}

}  // namespace robonode
