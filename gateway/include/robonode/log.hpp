#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <spdlog/logger.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace robonode {

// Logging seam (#42/#44): spdlog — mature, not reinvented. Two sinks:
//   - a ring that keeps the last N records so any surface (UI, CLI) can tail
//     them as a followable stream (recent_json),
//   - a colour STDERR sink so records show in `docker compose logs` without
//     polluting stdout (the CLI's JSON output).
// Structured + levelled. Product code logs through the RN_LOG_* macros, never
// printf/iostream. The RT motion loop does not log (ADR-9: no blocking I/O on
// the 1 kHz path); this is for the non-RT surfaces (gateway/apps).
//
// A record remembers which cell produced it. Without that, `GET /logs?cell=x`
// answered with every cell's records — a view claiming a scope the platform
// could not back (ADR-17 recorded it as the one global left). A thread says
// which cell it is working for; a record made outside any cell — boot, a store,
// the HTTP layer — belongs to the platform and shows in every cell's view,
// because it is equally true of all of them.
class Log {
public:
    // The cell the calling thread is working for. RAII, so a worker sets it
    // once and a nested command cannot leave it naming the wrong robot.
    class Scope {
    public:
        explicit Scope(std::string cell) : previous_{current()} { current() = std::move(cell); }
        ~Scope() { current() = std::move(previous_); }
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;

    private:
        std::string previous_;
    };

    static std::string& current() {
        static thread_local std::string cell;
        return cell;
    }

    static Log& instance() {
        static Log l;
        return l;
    }

    spdlog::logger& logger() { return *logger_; }

    // Recent records, newest last. Naming a cell gives that cell's records plus
    // the platform's; naming none gives every record there is.
    [[nodiscard]] std::string recent_json(const std::string& cell = {},
                                          std::size_t max = 100) const {
        auto arr = nlohmann::json::array();
        for (auto& [from, line] : ring_->recent(cell, max)) arr.push_back(std::move(line));
        return arr.dump();
    }

private:
    // spdlog's own ringbuffer keeps formatted strings and nothing else, so it
    // cannot answer "which cell". This one keeps the pair.
    class CellRing final : public spdlog::sinks::base_sink<std::mutex> {
    public:
        using Record = std::pair<std::string, std::string>;

        explicit CellRing(std::size_t capacity) : capacity_{capacity} {}

        [[nodiscard]] std::vector<Record> recent(const std::string& cell, std::size_t max) {
            std::lock_guard<std::mutex> lk{base_sink<std::mutex>::mutex_};
            std::vector<Record> out;
            for (const auto& record : records_) {
                if (cell.empty() || record.first.empty() || record.first == cell) {
                    out.push_back(record);
                }
            }
            if (out.size() > max) out.erase(out.begin(), out.end() - static_cast<long>(max));
            return out;
        }

    protected:
        void sink_it_(const spdlog::details::log_msg& msg) override {
            spdlog::memory_buf_t buf;
            base_sink<std::mutex>::formatter_->format(msg, buf);
            std::string line = fmt::to_string(buf);
            while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
            records_.emplace_back(current(), std::move(line));
            if (records_.size() > capacity_) records_.pop_front();
        }
        void flush_() override {}

    private:
        std::size_t capacity_;
        std::deque<Record> records_;
    };

    Log() {
        ring_ = std::make_shared<CellRing>(256);
        ring_->set_pattern("%H:%M:%S.%e %l %v");  // no colour codes in the ring
        auto out = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
        out->set_pattern("%H:%M:%S.%e %^%l%$ %v");
        logger_ = std::make_shared<spdlog::logger>("robonode",
                                                   spdlog::sinks_init_list{ring_, out});
        logger_->set_level(spdlog::level::info);
        logger_->flush_on(spdlog::level::info);
    }

    std::shared_ptr<CellRing> ring_;
    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace robonode

#define RN_LOG_INFO(...) ::robonode::Log::instance().logger().info(__VA_ARGS__)
#define RN_LOG_WARN(...) ::robonode::Log::instance().logger().warn(__VA_ARGS__)
#define RN_LOG_ERROR(...) ::robonode::Log::instance().logger().error(__VA_ARGS__)
