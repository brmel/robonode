#pragma once

#include <functional>
#include <string>
#include <vector>

#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/celld/conveyor.hpp"
#include "robonode/celld/scene.hpp"
#include "robonode/core/state.hpp"
#include "robonode/core/status.hpp"
#include "robonode/log.hpp"

namespace robonode {

// What the cell can ask a station to do. It knows two things: how to reach the
// live scene safely (the caller hands it that, so the lock discipline stays
// with whoever owns the physics) and which stations the descriptor declares.
//
// It lives outside the gateway because running a belt is station behaviour, not
// gateway plumbing — a second kind of station is a class here, not another pair
// of methods on the class that already does six other jobs.
class StationOps {
public:
    using WithScene = std::function<Status(const std::function<Status(Scene&)>&)>;
    using Stations = std::function<const std::vector<StationSpec>&()>;

    StationOps(WithScene with_scene, Stations stations)
        : with_scene_{std::move(with_scene)}, stations_{std::move(stations)} {}

    Status run(const std::string& id, Belt want) {
        return with_conveyor(id, [&](Conveyor& belt) {
            if (want == Belt::kStopped) return belt.stop();
            if (const auto st = belt.recycle(); !st.ok()) return st;
            RN_LOG_INFO("conveyor {}: running at {:.3f} m/s", belt.id(), belt.speed_m_s());
            return belt.start();
        });
    }

    // Put a fresh workpiece at the head of the line without running the belt —
    // what a station does between cycles when the part is picked at rest.
    Status deliver(const std::string& id) {
        return with_conveyor(id, [&](Conveyor& belt) {
            const auto st = belt.recycle();
            if (st.ok()) RN_LOG_INFO("conveyor {}: workpiece delivered", belt.id());
            return st;
        });
    }

private:
    Status with_conveyor(const std::string& id, const std::function<Status(Conveyor&)>& fn) {
        return with_scene_([&](Scene& live) {
            for (const auto& spec : stations_()) {
                if (spec.id != id && !id.empty()) continue;
                if (spec.type != "conveyor") continue;
                Conveyor belt{spec, live};
                return fn(belt);
            }
            return Status::failure("no conveyor station '" + id + "'");
        });
    }

    WithScene with_scene_;
    Stations stations_;
};

}  // namespace robonode
