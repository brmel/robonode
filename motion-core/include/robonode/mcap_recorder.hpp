#pragma once

#include <cstdio>
#include <string>
#include <vector>

#include <mcap/writer.hpp>

#include "robonode/executive.hpp"  // TelemetryRow

namespace robonode {

// Flight-recorder v0 (FR-6.2/6.3): telemetry rows → MCAP, one channel per
// node topic, JSON-encoded so Foxglove opens the file with zero plugins.
// Raw mcap writer (STACK.md verified pin); the foxglove-sdk spike may
// replace the implementation later — the wire format stays MCAP either way.
//
// v0 records post-run from the executive's row buffer. The ring-buffer
// always-on recorder arrives with celld; the file format and topic naming
// are already final here.
class McapRecorder {
public:
    // topic per axis, e.g. "rn/dev-cell/rail-x/MotionAxis/telemetry".
    static bool write(const std::string& path, const std::vector<std::string>& topics,
                      const std::vector<std::vector<TelemetryRow>>& rows_per_axis,
                      double rate_hz) {
        if (topics.size() != rows_per_axis.size()) return false;

        mcap::McapWriter writer;
        mcap::McapWriterOptions opts{"robonode"};
        opts.compression = mcap::Compression::None;  // deps-free build (no lz4/zstd)
        if (!writer.open(path, opts).ok()) {
            std::fprintf(stderr, "warning: could not open %s — recording not saved\n",
                         path.c_str());
            return false;
        }

        mcap::Schema schema{
            "robonode.v0.AxisTelemetry", "jsonschema",
            R"({"type":"object","properties":{
                "t_s":{"type":"number"},
                "target_position_mm":{"type":"number"},
                "target_velocity_mm_s":{"type":"number"},
                "governed_position_mm":{"type":"number"},
                "actual_position_mm":{"type":"number"},
                "actual_velocity_mm_s":{"type":"number"},
                "following_error_mm":{"type":"number"}}})"};
        writer.addSchema(schema);

        const auto ns_per_cycle = static_cast<std::uint64_t>(1e9 / rate_hz);
        char buf[512];
        for (std::size_t i = 0; i < topics.size(); ++i) {
            mcap::Channel ch{topics[i], "json", schema.id};
            writer.addChannel(ch);
            std::uint64_t t_ns = 0;
            for (const auto& r : rows_per_axis[i]) {
                const int len = std::snprintf(
                    buf, sizeof buf,
                    R"({"t_s":%.4f,"target_position_mm":%.3f,"target_velocity_mm_s":%.3f,)"
                    R"("governed_position_mm":%.3f,"actual_position_mm":%.3f,)"
                    R"("actual_velocity_mm_s":%.3f,"following_error_mm":%.4f})",
                    r.t_s, r.target_position_mm, r.target_velocity_mm_s,
                    r.governed_position_mm, r.actual_position_mm, r.actual_velocity_mm_s,
                    r.following_error_mm);
                if (len <= 0 || len >= static_cast<int>(sizeof buf)) continue;
                mcap::Message msg;
                msg.channelId = ch.id;
                msg.logTime = t_ns;
                msg.publishTime = t_ns;
                msg.data = reinterpret_cast<const std::byte*>(buf);
                msg.dataSize = static_cast<std::size_t>(len);
                if (!writer.write(msg).ok()) {
                    writer.close();
                    return false;
                }
                t_ns += ns_per_cycle;
            }
        }
        writer.close();
        return true;
    }
};

}  // namespace robonode
