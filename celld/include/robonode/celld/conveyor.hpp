#pragma once

#include <cstdlib>
#include <string>

#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/celld/scene.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// A conveyor as station behaviour: it runs the belt and recycles the workpiece.
// Everything it needs — the drive, the joint, the speed, where to send the part
// back to — is station config, so a second line is a descriptor entry.
//
// This is what makes the picking problem hard and honest: the target is moving
// while the robot plans, so an algorithm must predict, not just reach.
class Conveyor {
public:
    Conveyor(const StationSpec& spec, Scene& scene) : spec_{spec}, scene_{scene} {}

    [[nodiscard]] const std::string& id() const { return spec_.id; }
    [[nodiscard]] double speed_m_s() const { return num("speed_m_s", 0.0); }

    Status start() { return scene_.drive(text("actuator"), speed_m_s()); }
    Status stop() { return scene_.drive(text("actuator"), 0.0); }

    // Deliver a fresh workpiece at the head of the line, and rewind the belt so
    // it has its whole travel to carry it. The part then moves because friction
    // moves it — the station never writes its pose again.
    Status recycle() {
        if (const auto st = scene_.place(text("joint"), num("rewind_m", 0.0)); !st.ok()) return st;
        const auto head = scene_.site(text("start_site"));
        if (!head) return Status::failure("conveyor '" + id() + "': no start site");
        return scene_.place_body(text("workpiece"), *head + Vec3{0.0, 0.0, num("drop_m", 0.03)});
    }

    // A running line keeps presenting parts. Without this the workpiece reaches
    // the end of the belt and stops there — and a tracker that cannot predict
    // wins by simply waiting for it, which is what made the whole moving-target
    // scenario undecidable (docs/CHALLENGES.md). Travel is toward the end site;
    // past it, the part goes back to the head.
    Status recirculate() {
        const auto part = scene_.site(text("workpiece"));
        const auto head = scene_.site(text("start_site"));
        const auto end = scene_.site(text("end_site"));
        if (!part || !head || !end) return Status::success();
        const bool travels_negative = end->x < head->x;
        const bool past_the_end = travels_negative ? part->x <= end->x : part->x >= end->x;
        return past_the_end ? recycle() : Status::success();
    }

private:
    [[nodiscard]] std::string text(const char* key) const {
        const auto it = spec_.config.find(key);
        return it == spec_.config.end() ? std::string{} : it->second;
    }
    [[nodiscard]] double num(const char* key, double fallback) const {
        const auto it = spec_.config.find(key);
        return it == spec_.config.end() ? fallback : std::strtod(it->second.c_str(), nullptr);
    }

    const StationSpec& spec_;
    Scene& scene_;
};

}  // namespace robonode
