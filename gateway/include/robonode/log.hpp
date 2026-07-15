#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <spdlog/logger.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace robonode {

// Logging seam (#42/#44): spdlog — mature, not reinvented. Two sinks:
//   - a ring buffer that keeps the last N records so any surface (UI, CLI) can
//     tail them as a followable stream (recent_json),
//   - a colour STDERR sink so records show in `docker compose logs` without
//     polluting stdout (the CLI's JSON output).
// Structured + levelled. Product code logs through the RN_LOG_* macros, never
// printf/iostream. The RT motion loop does not log (ADR-9: no blocking I/O on
// the 1 kHz path); this is for the non-RT surfaces (gateway/apps).
class Log {
public:
    static Log& instance() {
        static Log l;
        return l;
    }

    spdlog::logger& logger() { return *logger_; }

    // Recent records, newest last, as a JSON array of formatted lines.
    [[nodiscard]] std::string recent_json(std::size_t max = 100) const {
        auto lines = ring_->last_formatted(max);
        auto arr = nlohmann::json::array();
        for (auto& line : lines) {
            while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
            arr.push_back(line);
        }
        return arr.dump();
    }

private:
    Log() {
        ring_ = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(256);
        ring_->set_pattern("%H:%M:%S.%e %l %v");  // no colour codes in the ring
        auto out = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
        out->set_pattern("%H:%M:%S.%e %^%l%$ %v");
        logger_ = std::make_shared<spdlog::logger>(
            "robonode", spdlog::sinks_init_list{ring_, out});
        logger_->set_level(spdlog::level::info);
        logger_->flush_on(spdlog::level::info);
    }

    std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> ring_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace robonode

#define RN_LOG_INFO(...) ::robonode::Log::instance().logger().info(__VA_ARGS__)
#define RN_LOG_WARN(...) ::robonode::Log::instance().logger().warn(__VA_ARGS__)
#define RN_LOG_ERROR(...) ::robonode::Log::instance().logger().error(__VA_ARGS__)
