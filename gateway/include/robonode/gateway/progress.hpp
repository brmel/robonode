#pragma once

#include <string>

#include <nlohmann/json.hpp>

namespace robonode {

// What one telemetry frame says about whether the cell is done.
//
// ADR-12: a command reply means *accepted*; completion is observed. So every
// surface that waits needs the same answer to "is it done yet", and there was
// one definition per surface — the facade's and the CLI's over HTTP — which is
// two chances to disagree about the only thing a caller is waiting on.
//
// `token` changes while the cell is making progress. A wait is bounded by
// SILENCE, not by a guess at how long the work takes: a cell still reporting
// has not failed, however slow the machine is.
struct Progress {
    bool settled{};
    std::string error;  // set only when settled and the work failed
    std::string token;
};

inline Progress progress_of(const nlohmann::json& telemetry) {
    if (!telemetry.is_object()) return {};
    const auto applied = telemetry.value("applied_id", 0ULL);
    const auto accepted = telemetry.value("accepted_id", 0ULL);
    if (applied >= accepted && !telemetry.value("running", false)) {
        return {true, telemetry.value("last_error", ""), {}};
    }
    return {false,
            {},
            std::to_string(applied) + ":" + telemetry.value("state", "") + ":" +
                std::to_string(telemetry.value("t", 0.0))};
}

}  // namespace robonode
