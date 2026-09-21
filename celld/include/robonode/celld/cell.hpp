#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/celld/descriptor.hpp"
#include "robonode/core/cancel.hpp"
#include "robonode/core/settings.hpp"
#include "robonode/core/lifecycle.hpp"
#include "robonode/core/status.hpp"
#include "robonode/core/telemetry.hpp"
#include "robonode/motion/axis_adapter.hpp"
#include "robonode/motion/driver_registry.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/otg.hpp"
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
                         double settle_s, double stop_time_s, const CycleHook& hook = {},
                         const std::vector<Controller*>& controllers = {},
                         const CancelToken* cancel = nullptr) {
        if (waypoints.size() != nodes_.size()) {
            return Status::failure("motion has " + std::to_string(waypoints.size()) +
                                   " waypoint lists, the cell has " + std::to_string(nodes_.size()) +
                                   " nodes");
        }
        Axes axes;
        if (const auto st = gather(controllers, axes); !st.ok()) return st;
        return drive("plan/execute", [&] {
            const auto planned = SyncBlendPlan::make(waypoints, axes.limits);
            if (!planned) return planned.error();
            const auto& plan = *planned;
            SyncExecutive exec{axes.adapters, axes.governors, rate_hz, controllers, stop_time_s};
            stats = exec.execute(plan, rows, settle_s, hook, cancel);
            return Status::success();
        });
    }

    // Stream per-axis setpoint sources (an OTG, a teleop feed) through the same
    // executive, governor and safety gate a planned move uses.
    Status run_sources(const std::vector<SetpointSource*>& sources, double rate_hz,
                       std::vector<std::vector<TelemetryRow>>& rows, CycleStats& stats,
                       double run_for_s, double stop_time_s, const CycleHook& hook = {},
                       const std::vector<Controller*>& controllers = {},
                       const CancelToken* cancel = nullptr) {
        if (sources.size() != nodes_.size()) {
            return Status::failure(std::to_string(sources.size()) + " setpoint sources for " +
                                   std::to_string(nodes_.size()) + " nodes");
        }
        Axes axes;
        if (const auto st = gather(controllers, axes); !st.ok()) return st;
        return drive("stream", [&] {
            SyncExecutive exec{axes.adapters, axes.governors, rate_hz, controllers, stop_time_s};
            stats = exec.execute(sources, rows, run_for_s, hook, cancel);
            return Status::success();
        });
    }

    [[nodiscard]] const std::vector<Node>& nodes() const { return nodes_; }
    [[nodiscard]] std::vector<Node>& nodes() { return nodes_; }

private:
    // What the executive needs from the cell, and the checks that must pass
    // before anything moves — shared by every way of running the cell.
    struct Axes {
        std::vector<AxisAdapter*> adapters;
        std::vector<Governor*> governors;
        std::vector<AxisLimits> limits;
    };

    Status gather(const std::vector<Controller*>& controllers, Axes& out) {
        if (!controllers.empty() && controllers.size() != nodes_.size()) {
            return Status::failure(std::to_string(controllers.size()) + " controllers for " +
                                   std::to_string(nodes_.size()) + " nodes");
        }
        for (auto& n : nodes_) {
            if (n.adapter->lifecycle() != Lifecycle::kActive) {
                return Status::failure(n.id + ": not active");
            }
            out.adapters.push_back(n.adapter.get());
            out.governors.push_back(n.governor.get());
            out.limits.push_back(n.descriptor.limits);
        }
        return Status::success();
    }

    // The executive throws on a malformed plan; a seam reports instead (ADR-7).
    template <typename Run>
    // Runs the work and ANSWERS with it. It used to call `run()` and discard
    // the result, which was invisible while every caller returned void — and
    // silently swallowed the first one that had something to say (a plan that
    // could not be built reported success and never executed).
    static Status drive(const char* what, Run run) {
        try {
            return run();
        } catch (const std::exception& e) {
            return Status::failure(std::string{what} + ": " + e.what());
        }
    }

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
