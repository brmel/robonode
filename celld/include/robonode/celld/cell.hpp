#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/celld/descriptor.hpp"
#include "robonode/core/lifecycle.hpp"
#include "robonode/core/status.hpp"
#include "robonode/core/telemetry.hpp"
#include "robonode/motion/axis_adapter.hpp"
#include "robonode/motion/driver_registry.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/sync_blend.hpp"
#include "robonode/motion/sync_executive.hpp"

namespace robonode {

// celld v0: the node tree brought to life — descriptors in, lifecycle-
// managed nodes out, cell-coherent execution across them (SPEC §2.3, §3.1
// coordination plane). No transport yet: this is the object model the
// Zenoh surface will expose.
struct Node {
    std::string id;
    Descriptor descriptor;
    std::unique_ptr<AxisAdapter> adapter;
    std::unique_ptr<Governor> governor;
};

class Cell {
public:
    // The registry supplies adapters by driver name; celld never names a
    // concrete driver or includes a vendor header (boundary lint). Apps
    // populate the registry with exactly the drivers they link.
    explicit Cell(const DriverRegistry& registry) : registry_{registry} {}

    Status add_node(const Descriptor& d) {
        Node n;
        n.id = d.id;
        n.descriptor = d;
        const DriverContext ctx{d.id, d.limits, d.config};
        if (const auto st = registry_.make(d.driver, ctx, n.adapter); !st.ok()) {
            return Status::failure(d.id + ": " + st.message());
        }
        n.governor = std::make_unique<Governor>(d.limits);
        nodes_.push_back(std::move(n));
        return Status::success();
    }

    // Swap ONE node's implementation to a different registered driver, live
    // — the same descriptor (limits, config) rebuilt behind the AxisAdapter
    // seam. The rest of the cell is untouched. This is "try each version of a
    // node" at node granularity; an unknown/incompatible driver fails closed
    // and the old node stays.
    Status replace_node(const std::string& id, const std::string& new_driver) {
        for (auto& n : nodes_) {
            if (n.id != id) continue;
            (void)n.adapter->deactivate();
            Descriptor d = n.descriptor;
            d.driver = new_driver;
            std::unique_ptr<AxisAdapter> adapter;
            const DriverContext ctx{d.id, d.limits, d.config};
            if (const auto st = registry_.make(new_driver, ctx, adapter); !st.ok()) {
                (void)n.adapter->activate();  // restore the old one
                return Status::failure(id + ": " + st.message());
            }
            if (const auto st = adapter->configure(); !st.ok()) return Status::failure(st.message());
            if (const auto st = adapter->activate(); !st.ok()) return Status::failure(st.message());
            n.adapter = std::move(adapter);
            n.descriptor = d;
            return Status::success();
        }
        return Status::failure("no node '" + id + "'");
    }

    // Lifecycle over the whole tree (FR-1.2): first failure aborts and
    // reports which node; already-transitioned nodes are left as-is for the
    // caller to inspect — celld's reconciler will handle rollback.
    Status configure_all() { return for_each("configure", &AxisAdapter::configure); }
    Status activate_all() { return for_each("activate", &AxisAdapter::activate); }
    Status deactivate_all() { return for_each("deactivate", &AxisAdapter::deactivate); }

    // Cell-coherent synchronized waypoint run across every node (FR-2.3/2.4).
    // waypoints[i] belongs to nodes()[i]; all nodes must be kActive.
    // settle_s runs the loop past the plan end so a physical plant converges.
    Status run_waypoints(const std::vector<std::vector<double>>& waypoints, double rate_hz,
                         std::vector<std::vector<TelemetryRow>>& rows, CycleStats& stats,
                         double settle_s = 0.5, const CycleHook& hook = {}) {
        if (waypoints.size() != nodes_.size()) {
            return Status::failure("waypoint lists != node count");
        }
        std::vector<AxisAdapter*> adapters;
        std::vector<Governor*> governors;
        std::vector<AxisLimits> limits;
        for (auto& n : nodes_) {
            if (n.adapter->lifecycle() != Lifecycle::kActive) {
                return Status::failure(n.id + ": not active");
            }
            adapters.push_back(n.adapter.get());
            governors.push_back(n.governor.get());
            limits.push_back(n.descriptor.limits);
        }
        try {
            const auto plan = SyncBlendPlan::plan(waypoints, limits);
            SyncExecutive exec{adapters, governors, rate_hz};
            stats = exec.execute(plan, rows, settle_s, hook);
        } catch (const std::exception& e) {
            return Status::failure(std::string{"plan/execute: "} + e.what());
        }
        return Status::success();
    }

    [[nodiscard]] const std::vector<Node>& nodes() const { return nodes_; }
    [[nodiscard]] std::vector<Node>& nodes() { return nodes_; }

private:
    template <typename Verb>
    Status for_each(const char* what, Verb verb) {
        for (auto& n : nodes_) {
            if (const auto st = (n.adapter.get()->*verb)(); !st.ok()) {
                return Status::failure(n.id + ": " + what + ": " + st.message());
            }
        }
        return Status::success();
    }

    const DriverRegistry& registry_;
    std::vector<Node> nodes_;
};

}  // namespace robonode
