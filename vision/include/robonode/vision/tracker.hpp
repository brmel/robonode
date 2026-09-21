#pragma once

#include <algorithm>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/motion/module_registry.hpp"

namespace robonode {

// One timestamped sighting. A detector says *where*; only a sequence of these
// says *where next*, which is what picking from a moving belt needs.
struct Observation {
    double t_s{};
    Vec3 position;
};

// The tracking seam: consume sightings, answer "where will it be at time t?".
//
// This is the capability the moving-target problem lives in. A robot that
// plans to where the part *was* arrives late by exactly the belt speed times
// its own cycle time; solving that is the user's job, and this interface is
// the whole surface they need to replace.
class Tracker {
public:
    virtual ~Tracker() = default;

    virtual void observe(const Observation& seen) = 0;

    // Predicted position at `t_s`, or nothing when the tracker cannot say.
    [[nodiscard]] virtual std::optional<Vec3> predict(double t_s) const = 0;

    // Estimated velocity (m/s). Reported for telemetry; not every tracker has one.
    [[nodiscard]] virtual Vec3 velocity() const { return {}; }
};

struct TrackerContext {
    std::size_t window{12};   // how many sightings a version may keep
    double gate_m{0.25};      // a sighting further than this is a different part
    double min_span_s{0.15};  // sightings must span this long before a speed is claimed
    double smoothing{0.4};    // how much of each disagreement a smoothed estimate takes
    std::map<std::string, std::string> config;
};

using TrackerRegistry = ModuleRegistry<Tracker, TrackerContext>;

// The naive baseline: the part is wherever it was last seen. Correct for a
// static bin, and visibly wrong the moment the belt moves — which is the point
// of shipping it as a selectable version.
class SnapshotTracker final : public Tracker {
public:
    void observe(const Observation& seen) override { last_ = seen; }

    [[nodiscard]] std::optional<Vec3> predict(double) const override {
        if (!last_) return std::nullopt;
        return last_->position;
    }

private:
    std::optional<Observation> last_;
};

// Least-squares constant velocity over a sliding window, extrapolated to the
// requested time. Enough to intercept a belt; blind to acceleration.
class ConstantVelocityTracker final : public Tracker {
public:
    ConstantVelocityTracker(std::size_t window, double gate_m = 0.25, double min_span_s = 0.15)
        : window_{std::max<std::size_t>(2, window)}, gate_m_{gate_m}, min_span_s_{min_span_s} {}

    void observe(const Observation& seen) override {
        if (!history_.empty() && seen.t_s <= history_.back().t_s) return;  // stale or repeated
        // Association: a sighting far from where this part should be is a
        // DIFFERENT part — a new one arriving, or the line reset. Folding it
        // into the same track would invent a velocity that never happened.
        if (const auto expected = predict(seen.t_s)) {
            if ((seen.position - *expected).norm() > gate_m_) history_.clear();
        }
        history_.push_back(seen);
        while (history_.size() > window_) history_.pop_front();
    }

    [[nodiscard]] std::optional<Vec3> predict(double t_s) const override {
        if (history_.empty()) return std::nullopt;
        const auto& last = history_.back();
        const Vec3 v = velocity();
        const double dt = t_s - last.t_s;
        return Vec3{last.position.x + v.x * dt, last.position.y + v.y * dt,
                    last.position.z + v.z * dt};
    }

    [[nodiscard]] Vec3 velocity() const override {
        if (history_.size() < 2) return {};
        // Speed needs a baseline in TIME. Sightings crowded into one instant fit
        // a slope through noise and report a part moving metres per second —
        // which an interception then aims at, off the end of the world.
        if (history_.back().t_s - history_.front().t_s < min_span_s_) return {};
        // Slope of position against time, per axis: cov(t,p) / var(t).
        double t_mean = 0.0;
        for (const auto& o : history_) t_mean += o.t_s;
        t_mean /= static_cast<double>(history_.size());

        Vec3 cov{};
        double var = 0.0;
        for (const auto& o : history_) {
            const double dt = o.t_s - t_mean;
            var += dt * dt;
            cov.x += dt * o.position.x;
            cov.y += dt * o.position.y;
            cov.z += dt * o.position.z;
        }
        if (var <= 1e-12) return {};
        return {cov.x / var, cov.y / var, cov.z / var};
    }

private:
    std::size_t window_;
    double gate_m_, min_span_s_;
    std::deque<Observation> history_;
};

// Exponentially-smoothed velocity: each sighting corrects the estimate by a
// fraction of what it disagreed with, instead of refitting a window. Cheaper
// than the least-squares fit and steadier under noisy detections — and it
// lags a real acceleration, which is exactly the trade a user comparing
// versions should be able to see.
class SmoothedTracker final : public Tracker {
public:
    SmoothedTracker(double alpha, double gate_m, double min_span_s)
        : alpha_{std::clamp(alpha, 0.0, 1.0)}, gate_m_{gate_m}, min_span_s_{min_span_s} {}

    void observe(const Observation& seen) override {
        if (last_ && seen.t_s <= last_->t_s) return;  // stale or repeated
        // Same association rule as the window tracker: a sighting far from the
        // prediction is a DIFFERENT part, so the track starts over rather than
        // inventing the velocity that would join them.
        if (const auto expected = predict(seen.t_s)) {
            if ((seen.position - *expected).norm() > gate_m_) reset();
        }
        if (last_) {
            const double dt = seen.t_s - last_->t_s;
            const Vec3 measured{(seen.position.x - last_->position.x) / dt,
                                (seen.position.y - last_->position.y) / dt,
                                (seen.position.z - last_->position.z) / dt};
            velocity_ = {velocity_.x + alpha_ * (measured.x - velocity_.x),
                         velocity_.y + alpha_ * (measured.y - velocity_.y),
                         velocity_.z + alpha_ * (measured.z - velocity_.z)};
            span_s_ += dt;
        }
        last_ = seen;
    }

    [[nodiscard]] std::optional<Vec3> predict(double t_s) const override {
        if (!last_) return std::nullopt;
        const Vec3 v = velocity();
        const double dt = t_s - last_->t_s;
        return Vec3{last_->position.x + v.x * dt, last_->position.y + v.y * dt,
                    last_->position.z + v.z * dt};
    }

    // A speed claim still needs a real baseline in time; before that this is a
    // tracker that knows where the part is and admits it does not know where
    // it is going.
    [[nodiscard]] Vec3 velocity() const override {
        return span_s_ < min_span_s_ ? Vec3{} : velocity_;
    }

private:
    void reset() {
        last_.reset();
        velocity_ = {};
        span_s_ = 0.0;
    }

    double alpha_, gate_m_, min_span_s_;
    std::optional<Observation> last_;
    Vec3 velocity_{};
    double span_s_{0.0};
};

inline void register_basic_trackers(TrackerRegistry& reg) {
    reg.add("robonode.snapshot", [](const TrackerContext&) -> std::unique_ptr<Tracker> {
        return std::make_unique<SnapshotTracker>();
    });
    reg.add("robonode.constant-velocity", [](const TrackerContext& c) -> std::unique_ptr<Tracker> {
        return std::make_unique<ConstantVelocityTracker>(c.window, c.gate_m, c.min_span_s);
    });
    reg.add("robonode.smoothed", [](const TrackerContext& c) -> std::unique_ptr<Tracker> {
        return std::make_unique<SmoothedTracker>(c.smoothing, c.gate_m, c.min_span_s);
    });
}

}  // namespace robonode
