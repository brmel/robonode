#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/telemetry.hpp"
#include "robonode/log.hpp"
#include "robonode/recorder/mcap_recorder.hpp"

namespace robonode {

// Writes every run to MCAP (Foxglove opens it with no plugins) together with
// the versions that produced it. Provenance is the point: a Compare result you
// cannot reproduce is an anecdote.
class RunRecorder {
public:
    explicit RunRecorder(std::string dir) : dir_{std::move(dir)} {}

    void record(const std::string& label, const std::vector<std::string>& axis_ids,
                const std::vector<std::vector<TelemetryRow>>& rows, double rate_hz,
                std::map<std::string, std::string> versions) {
        if (rows.empty()) return;
        std::error_code ec;
        std::filesystem::create_directories(dir_, ec);

        const std::string id = label + "-" + std::to_string(++seq_);
        const std::string path = (std::filesystem::path{dir_} / (id + ".mcap")).string();
        std::vector<std::string> topics;
        topics.reserve(axis_ids.size());
        for (const auto& a : axis_ids) topics.push_back("rn/cell/" + a + "/MotionAxis/telemetry");

        if (const auto st = McapRecorder::write(path, topics, rows, rate_hz); !st.ok()) {
            RN_LOG_WARN("run recorder: {}", st.message());
            return;
        }
        summary_ = {{"run_id", id}, {"mcap", path}, {"cycles", rows.front().size()},
                    {"rate_hz", rate_hz}, {"versions", versions}};
        RN_LOG_INFO("run recorded: {}", path);
    }

    [[nodiscard]] std::string summary_json() const { return summary_.dump(); }

private:
    std::string dir_;
    std::uint64_t seq_{0};
    nlohmann::json summary_ = nlohmann::json::object();
};

}  // namespace robonode
