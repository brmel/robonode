#pragma once

#include <cmath>
#include <cstdlib>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "robonode/celld/app_descriptor.hpp"
#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/celld/station.hpp"
#include "robonode/celld/tool_node.hpp"
#include "robonode/core/geometry.hpp"
#include "robonode/core/settings.hpp"
#include "robonode/core/status.hpp"
#include "robonode/gateway/motion_service.hpp"
#include "robonode/gateway/tracking_capability.hpp"
#include "robonode/gateway/vision_capability.hpp"
#include "robonode/log.hpp"

namespace robonode {

// Runs an application program: one verb table, one handler each. Adding a verb
// is one entry — no branch chain, no gateway edit.
class ProgramRunner {
public:
    using FamilyFn = std::function<Status(const std::string&)>;
    using VersionFn = std::function<Status(const std::string&, const std::string&)>;
    using ClockFn = std::function<double()>;
    using ConveyorFn = std::function<Status(const std::string&, Belt)>;
    using DeliverFn = std::function<Status(const std::string&)>;
    using WaitFn = std::function<void(double)>;
    // What the program is doing right now, for surfaces that show progress.
    // The log alone answers "what happened"; this answers "what is happening".
    using StepFn = std::function<void(const std::string&)>;

    ProgramRunner(MotionService& motion, VisionCapability& vision, TrackingCapability& tracking,
                  ToolNode& tool, const CellDescriptor& cell, FamilyFn set_family,
                  VersionFn set_version, ClockFn now, ConveyorFn conveyor, DeliverFn deliver,
                  WaitFn wait, Settings::App cfg = {})
        : motion_{motion}, vision_{vision}, tracking_{tracking}, tool_{tool}, cell_{cell},
          set_family_{std::move(set_family)}, set_version_{std::move(set_version)},
          now_{std::move(now)}, conveyor_{std::move(conveyor)}, deliver_{std::move(deliver)},
          wait_{std::move(wait)}, cfg_{cfg} {
        install_verbs();
    }

    void set_step_sink(StepFn on_step) { on_step_ = std::move(on_step); }

    Status run(const std::vector<AppStep>& program) {
        RN_LOG_INFO("app: {} step(s)", program.size());
        slots_.clear();
        const auto report_step = [&](const std::string& what) {
            if (on_step_) on_step_(what);
        };
        for (std::size_t i = 0; i < program.size(); ++i) {
            const auto& step = program[i];
            const auto verb = verbs_.find(step.verb);
            if (verb == verbs_.end()) {
                RN_LOG_WARN("app step {}/{} '{}': unknown verb", i + 1, program.size(), step.verb);
                return Status::failure("unknown verb '" + step.verb + "'");
            }
            // Numbering every step is what lets an operator answer "which step
            // failed" and "how long did it take" from the log alone.
            RN_LOG_INFO("app step {}/{}: {}", i + 1, program.size(), describe(step));
            report_step(std::to_string(i + 1) + "/" + std::to_string(program.size()) + " " +
                        describe(step));
            if (const auto st = verb->second.run(step); !st.ok()) {
                report_step({});
                RN_LOG_WARN("app step {}/{} '{}' failed: {}", i + 1, program.size(), step.verb,
                            st.message());
                return st;
            }
        }
        RN_LOG_INFO("app: complete");
        report_step({});
        return Status::success();
    }

    // The verb table IS the program contract. A surface that lets a user author
    // a step reads it instead of keeping its own list, which is the only way the
    // two cannot drift apart (ADR-8).
    [[nodiscard]] nlohmann::json catalogue() const {
        nlohmann::json out = nlohmann::json::array();
        for (const auto& [name, verb] : verbs_) out.push_back({{"verb", name}, {"args", verb.args}});
        return out;
    }

    // Refuse a program before it is stored, not halfway through running it.
    [[nodiscard]] Status check(const std::vector<AppStep>& program) const {
        for (const auto& step : program) {
            if (!verbs_.contains(step.verb)) {
                return Status::failure("unknown verb '" + step.verb + "'");
            }
        }
        return Status::success();
    }

private:
    using Handler = std::function<Status(const AppStep&)>;

    struct Verb {
        std::vector<std::string> args;
        Handler run;
    };

    static double num(const AppStep& s, const char* key, double fallback = 0.0) {
        const auto it = s.args.find(key);
        return it == s.args.end() ? fallback : std::strtod(it->second.c_str(), nullptr);
    }
    static std::string text(const AppStep& s, const char* key, std::string fallback = {}) {
        const auto it = s.args.find(key);
        return it == s.args.end() ? std::move(fallback) : it->second;
    }

    // Degrees in, unit quaternion out. The conversion lives here because the
    // verb accepts both and only one of them can be what the platform stores.
    static Quat quat_from_rpy(double roll_deg, double pitch_deg, double yaw_deg) {
        const double to_rad = 3.14159265358979323846 / 180.0;
        const double cr = std::cos(roll_deg * to_rad / 2), sr = std::sin(roll_deg * to_rad / 2);
        const double cp = std::cos(pitch_deg * to_rad / 2), sp = std::sin(pitch_deg * to_rad / 2);
        const double cy = std::cos(yaw_deg * to_rad / 2), sy = std::sin(yaw_deg * to_rad / 2);
        return Quat{cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                    cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy}
            .normalized();
    }

    static constexpr double kSightPollS = 0.05;

    static std::string describe(const AppStep& s) {
        std::string out = s.verb;
        for (const auto& [k, v] : s.args) out += " " + k + "=" + v;
        return out;
    }

    void add(std::string name, std::vector<std::string> args, Handler run) {
        verbs_.emplace(std::move(name), Verb{std::move(args), std::move(run)});
    }

    void install_verbs() {
        add("family", {"family"}, [this](const AppStep& s) { return set_family_(text(s, "family")); });
        add("version", {"capability", "version"}, [this](const AppStep& s) {
            return set_version_(text(s, "capability"), text(s, "version"));
        });
        add("motion", {"id"},
            [this](const AppStep& s) { return motion_.run_motion(text(s, "id")); });
        add("move_l", {"x", "y", "z"}, [this](const AppStep& s) {
            return motion_.move_l({num(s, "x"), num(s, "y"), num(s, "z")});
        });
        // The platform speaks quaternions, but a person authoring a step has a
        // drawing with angles on it. Both are accepted: roll/pitch/yaw in
        // degrees when they are given, the quaternion otherwise.
        add("move_pose", {"x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg", "qw", "qx", "qy", "qz"},
            [this](const AppStep& s) {
                const bool by_angle = s.args.count("roll_deg") > 0 ||
                                      s.args.count("pitch_deg") > 0 || s.args.count("yaw_deg") > 0;
                const Quat q =
                    by_angle ? quat_from_rpy(num(s, "roll_deg"), num(s, "pitch_deg"),
                                             num(s, "yaw_deg"))
                             : Quat{num(s, "qw", 1.0), num(s, "qx"), num(s, "qy"), num(s, "qz")};
                return motion_.move_pose({{num(s, "x"), num(s, "y"), num(s, "z")}, q.normalized()});
            });
        add("grasp", {}, [this](const AppStep&) { return grasp(); });
        add("release", {}, [this](const AppStep&) { return release(); });
        add("pick", {"approach"}, [this](const AppStep& s) { return pick(s); });
        add("place", {"station", "approach", "slot"},
            [this](const AppStep& s) { return place(s); });
        add("intercept", {"approach", "attempts", "lead"},
            [this](const AppStep& s) { return intercept(s); });
        add("wait", {"seconds"}, [this](const AppStep& s) {
            wait_(num(s, "seconds", 1.0));
            return Status::success();
        });
        add("conveyor", {"station", "run"}, [this](const AppStep& s) {
            const bool run = text(s, "run", "true") != "false";
            return conveyor_(text(s, "station"), run ? Belt::kRunning : Belt::kStopped);
        });
        add("deliver", {"station"},
            [this](const AppStep& s) { return deliver_(text(s, "station")); });
    }

    // Approach above the target, descend, act, retract — the motion shape every
    // pick and place shares. The descent aims at a FRESH sighting where one is
    // available: a part is a real body, and reaching past it can nudge it, so
    // the pose that was true above the bin need not be true on the way down.
    Status visit(Vec3 target, double approach_m, const std::function<Status()>& act,
                 bool re_look = false) {
        const Vec3 above{target.x, target.y, target.z + approach_m};
        if (const auto st = motion_.move_l(above); !st.ok()) return st;
        if (re_look) {
            if (const auto seen = vision_.observe(); seen) target = *seen;
        }
        if (const auto st = motion_.move_l(target); !st.ok()) return st;
        if (const auto st = act(); !st.ok()) return st;
        return motion_.move_l(Vec3{target.x, target.y, target.z + approach_m});
    }

    // The tool opening and closing is the state change a pick-and-place is
    // about; it was the one thing the log never mentioned.
    Status grasp() {
        const auto st = tool_.grasp();
        if (st.ok()) RN_LOG_INFO("tool: grasped");
        return st;
    }

    Status release() {
        const auto st = tool_.release();
        if (st.ok()) RN_LOG_INFO("tool: released");
        return st;
    }

    Status pick(const AppStep& s) {
        const auto seen = vision_.observe();
        if (!seen) return Status::failure("pick: no part detected");
        return visit(*seen, num(s, "approach", cfg_.approach_m), [this] { return grasp(); }, true);
    }

    // Pick a part that is MOVING. The robot cannot reach where the part is —
    // by the time it arrives the part has gone — so it plans to where the
    // tracking capability says the part *will* be, and re-aims with what the
    // last attempt taught it about how long a reach actually takes.
    //
    // This is the platform's hard problem in one function: everything it needs
    // is behind a seam a user can replace, and a naive tracker visibly fails.
    Status intercept(const AppStep& s) {
        const double approach = num(s, "approach", cfg_.approach_m);
        const int attempts = static_cast<int>(num(s, "attempts", cfg_.intercept_attempts));
        double lead = num(s, "lead", cfg_.intercept_lead_s);

        for (int attempt = 1; attempt <= attempts; ++attempt) {
            const auto seen = await_sighting(num(s, "sight_timeout", cfg_.sight_timeout_s));
            if (!seen) return Status::failure("intercept: no part came into view");
            // The sighting is already old. Feeding it in at its capture time is
            // what lets the tracker extrapolate across the sensor's latency
            // instead of treating a stale pose as current.
            tracking_.observe(seen->captured_s, seen->position);

            const auto target = tracking_.predict(now_() + lead);
            if (!target) return Status::failure("intercept: tracker has no prediction");
            RN_LOG_INFO("intercept try {}/{}: aiming {:.3f}m ahead at ({:.3f}, {:.3f}, {:.3f})",
                        attempt, attempts, lead * tracking_.velocity().norm(), target->x, target->y,
                        target->z);

            const double started = now_();
            const Vec3 velocity = tracking_.velocity();
            const Vec3 above{target->x, target->y, target->z + approach};
            // The goal carries how the part is moving, so a rendezvous planner
            // can arrive travelling with it instead of across it.
            if (const auto st = motion_.move_to(above, velocity); !st.ok()) return st;
            if (const auto st = motion_.move_to(*target, velocity); !st.ok()) return st;

            // What the reach actually cost is the best estimate of the next one.
            lead = now_() - started;

            if (const auto st = grasp(); st.ok()) {
                return motion_.move_l({target->x, target->y, target->z + approach});
            }
            RN_LOG_WARN("intercept try {}/{} missed — re-aiming with lead {:.2f}s", attempt,
                        attempts, lead);
        }
        return Status::failure("intercept: missed after " + std::to_string(attempts) + " attempts");
    }

    // A camera has latency and a line has gaps: the part may simply not be in
    // view yet. Waiting for it is what a real cell does, so the platform does
    // it too rather than failing on the first empty frame.
    //
    // This is a poll, and it stays one: `wait_` ADVANCES the world (a program
    // step runs while the idle ticker is paused, so nothing else moves the
    // belt). The loop is the simulation clock, not a spin on someone else's
    // state — there is no publication to wait for, because nothing would
    // publish until this loop lets time pass.
    std::optional<Detection> await_sighting(double timeout_s) {
        for (double waited = 0.0; waited <= timeout_s; waited += kSightPollS) {
            if (const auto seen = vision_.look()) return seen;
            wait_(kSightPollS);
        }
        return std::nullopt;
    }

    Status place(const AppStep& s) {
        const std::string id = text(s, "station");
        const auto* station = find_station(id);
        if (station == nullptr) return Status::failure("unknown station '" + id + "'");
        if (station->type != "pallet") {
            return Status::failure("station '" + id + "' type '" + station->type +
                                   "' has no place behaviour");
        }
        // The next free slot by default; a program that is stacking says which
        // one, because "on top of what is already there" is a choice, not a
        // counter.
        const auto named = s.args.find("slot");
        const int index = named == s.args.end() ? slots_[id]++
                                                : static_cast<int>(num(s, "slot"));
        const Vec3 slot = pallet_slot(*station, index);
        RN_LOG_INFO("place -> {} slot {}", id, index);
        return visit(slot, num(s, "approach", cfg_.approach_m), [this] { return release(); });
    }

    const StationSpec* find_station(const std::string& id) const {
        for (const auto& s : cell_.stations) {
            if (s.id == id) return &s;
        }
        return nullptr;
    }

    MotionService& motion_;
    VisionCapability& vision_;
    TrackingCapability& tracking_;
    ToolNode& tool_;
    const CellDescriptor& cell_;
    FamilyFn set_family_;
    VersionFn set_version_;
    ClockFn now_;
    ConveyorFn conveyor_;
    DeliverFn deliver_;
    WaitFn wait_;
    Settings::App cfg_;
    StepFn on_step_;
    std::map<std::string, Verb> verbs_;
    std::map<std::string, int> slots_;
};

}  // namespace robonode
