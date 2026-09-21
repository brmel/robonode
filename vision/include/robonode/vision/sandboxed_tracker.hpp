#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "robonode/sandbox/compiler.hpp"
#include "robonode/sandbox/program.hpp"
#include "robonode/vision/tracker.hpp"

namespace robonode {

// A user-authored motion model. The host does the bookkeeping every tracker
// needs — keep a history, measure the elapsed time, estimate a raw velocity —
// and hands the algorithm the numbers, so the sandbox program is exactly the
// interesting part: where will it be in `dt` seconds?
//
// ABI: x y z vx vy vz dt -> predicted x y z. Output is validated (finite, and
// within a sane travel of the last sighting) before any motion is planned.
class SandboxedTracker final : public Tracker {
public:
    SandboxedTracker(sandbox::Program program, sandbox::SandboxLimits limits, std::size_t window,
                     double gate_m, double max_travel_m)
        : inner_{window, gate_m}, program_{std::move(program)}, limits_{limits},
          max_travel_m_{max_travel_m} {}

    void observe(const Observation& seen) override {
        inner_.observe(seen);
        last_ = seen;
    }

    [[nodiscard]] std::optional<Vec3> predict(double t_s) const override {
        if (!last_) return std::nullopt;
        const Vec3 v = inner_.velocity();
        std::vector<double> out;
        const auto st = sandbox::run(program_,
                                     {last_->position.x, last_->position.y, last_->position.z, v.x,
                                      v.y, v.z, t_s - last_->t_s},
                                     limits_, out);
        if (!st.ok() || out.size() != 3) return std::nullopt;
        const Vec3 predicted{out[0], out[1], out[2]};
        if (!finite(predicted)) return std::nullopt;
        // A prediction is a claim about the near future, not a teleport: refuse
        // anything absurdly far from where the part actually is.
        if ((predicted - last_->position).norm() > max_travel_m_) return std::nullopt;
        return predicted;
    }

    [[nodiscard]] Vec3 velocity() const override { return inner_.velocity(); }

private:
    static bool finite(const Vec3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    ConstantVelocityTracker inner_;  // the history + raw velocity the ABI needs
    std::optional<Observation> last_;
    sandbox::Program program_;
    sandbox::SandboxLimits limits_;
    double max_travel_m_;
};

inline void register_program_tracker(TrackerRegistry& reg, const std::string& version,
                                     sandbox::Program program, double max_travel_m = 1.0,
                                     sandbox::SandboxLimits limits = {}) {
    reg.add(version, [program = std::move(program), limits, max_travel_m](
                         const TrackerContext& c) -> std::unique_ptr<Tracker> {
        return std::make_unique<SandboxedTracker>(program, limits, c.window, c.gate_m, max_travel_m);
    });
}

}  // namespace robonode
