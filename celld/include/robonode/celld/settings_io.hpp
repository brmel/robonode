#pragma once

#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "robonode/core/settings.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// The JSON names of every tunable, declared once next to nothing else. The
// mature library does the work: NLOHMANN_..._WITH_DEFAULT leaves a field that
// the file does not name at the value the struct already had, which is exactly
// "a partial settings file overrides only what it names" (ADR-14).
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Settings::Ik, lambda, tol_m, max_iters,
                                                max_step_rad, carrier_weight, line_steps,
                                                sandbox_line_steps, lead_in_s, tol_rad,
                                                orientation_weight, clearance_m)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Settings::Motion, rate_hz, settle_s, stop_time_s,
                                                max_duration_s, limit_tolerance)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Settings::App, approach_m, grasp_reach_m,
                                                intercept_attempts, intercept_lead_s,
                                                sight_timeout_s, grip_constraint)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Settings::Tracking, window, gate_m, min_span_s,
                                                smoothing)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Settings::Gateway, queue_capacity,
                                                telemetry_decimation, stream_period_ms,
                                                log_every_n_frames, stall_timeout_ms,
                                                max_overrun_fraction, max_jitter_p99_us)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Settings::Recorder, enabled, dir)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Settings::Compare, trial, fallback)

namespace detail {

// A section a file does not carry leaves that whole struct as it was.
template <class T>
void read_section(const nlohmann::json& j, const char* name, T& slot) {
    const auto it = j.find(name);
    if (it != j.end() && it->is_object()) slot = it->get<T>();
}

// The workspace bound is a raw triple, which the macro cannot describe.
inline void read_triple(const nlohmann::json& j, const char* section, const char* key,
                        double (&slot)[3]) {
    const auto s = j.find(section);
    if (s == j.end() || !s->is_object()) return;
    const auto v = s->find(key);
    if (v == s->end() || !v->is_array() || v->size() != 3) return;
    for (std::size_t i = 0; i < 3; ++i) slot[i] = (*v)[i].get<double>();
}

}  // namespace detail

inline void apply_settings(const nlohmann::json& j, Settings& s) {
    using detail::read_section;
    read_section(j, "ik", s.ik);
    read_section(j, "motion", s.motion);
    read_section(j, "app", s.app);
    read_section(j, "tracking", s.tracking);
    read_section(j, "gateway", s.gateway);
    read_section(j, "recorder", s.recorder);
    read_section(j, "compare", s.compare);
    detail::read_triple(j, "workspace", "lo", s.workspace.lo);
    detail::read_triple(j, "workspace", "hi", s.workspace.hi);
}

inline Status load_settings(const std::string& path, Settings& out) {
    std::ifstream f{path};
    if (!f) return Status::failure("cannot open settings: " + path);
    const auto j = nlohmann::json::parse(f, nullptr, false);
    if (j.is_discarded()) return Status::failure("invalid JSON: " + path);
    apply_settings(j, out);
    return Status::success();
}

// Load `path` if present, else keep the built-in defaults; `note` records which.
inline Settings settings_or_defaults(const std::string& path, std::string* note = nullptr) {
    Settings s;
    const auto st = load_settings(path, s);
    if (note != nullptr) *note = st.ok() ? path : "built-in defaults (" + st.message() + ")";
    return s;
}

}  // namespace robonode
