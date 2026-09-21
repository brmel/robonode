#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "robonode/core/status.hpp"

#include "robonode/gateway/cell_gateway.hpp"

namespace robonode {

// Owns the running cells — one per robot / station group. There is one "main"
// cell today; a second robot is `add(id, world, cell)`. Each CellGateway keeps
// its own worker thread + telemetry, so cells run fully independently; the
// Platform facade addresses them by id (defaulting to "main"), so every surface
// is multi-cell-ready without a one-cell assumption baked in.
class CellManager {
public:
    CellManager(std::string worlds_dir, const std::string& cell_path, Settings settings = {},
                CellGateway::DriverHook vendor_drivers = {},
                CellGateway::VisionHook vision_versions = {}, std::string scenes_dir = {})
        : settings_{settings}, vendor_drivers_{std::move(vendor_drivers)},
          vision_versions_{std::move(vision_versions)}, scenes_{std::move(scenes_dir)} {
        (void)add("main", std::move(worlds_dir), cell_path);
    }

    // Adding a cell is how a second robot joins: its own worker, telemetry and
    // capabilities, addressed by id.
    Status add(const std::string& id, std::string worlds_dir, const std::string& cell_path) {
        if (id.empty()) return Status::failure("cell id required");
        auto gateway = std::make_shared<CellGateway>(std::move(worlds_dir), cell_path, settings_,
                                                     vendor_drivers_, vision_versions_, scenes_,
                                                     id);
        std::lock_guard<std::mutex> lk{mtx_};
        if (cells_.count(id) > 0) return Status::failure("cell '" + id + "' already exists");
        cells_[id] = std::move(gateway);
        return Status::success();
    }

    Status remove(const std::string& id) {
        if (id == "main") return Status::failure("the main cell cannot be removed");
        std::lock_guard<std::mutex> lk{mtx_};
        return cells_.erase(id) > 0 ? Status::success()
                                    : Status::failure("no cell '" + id + "'");
    }

    [[nodiscard]] bool has(const std::string& id) const {
        std::lock_guard<std::mutex> lk{mtx_};
        return cells_.count(id) > 0;
    }

    // A SHARED pointer, not a reference: a request can be reading a cell while
    // another drops it, and a reference would be a use-after-free the moment the
    // map let go. Holding the cell alive for as long as the caller uses it is
    // the ownership rule made structural — the map may forget it, the work in
    // flight may not.
    std::shared_ptr<CellGateway> cell(const std::string& id = "main") {
        std::lock_guard<std::mutex> lk{mtx_};
        const auto it = cells_.find(id);
        return it == cells_.end() ? nullptr : it->second;
    }

    // A list of ids says which cells exist and nothing about WHAT they are. Two
    // robots differ in their chain, so the chain is what a surface needs to show
    // that a second one is a different machine rather than a second copy.
    std::string cells_json() {
        auto arr = nlohmann::json::array();
        for (auto& [id, gateway] : snapshot()) {
            const auto tree = nlohmann::json::parse(gateway->nodes_json(), nullptr, false);
            arr.push_back({{"id", id},
                           {"nodes", tree.is_object() ? tree.value("nodes", nlohmann::json::array()).size() : 0},
                           {"family", tree.is_object() ? tree.value("family", "") : ""}});
        }
        return arr.dump();
    }

private:
    // Copy the map, then talk to the cells outside the lock: building each
    // entry's answer means calling into a gateway, which must not happen with
    // the manager's mutex held.
    std::map<std::string, std::shared_ptr<CellGateway>> snapshot() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return cells_;
    }

    mutable std::mutex mtx_;
    Settings settings_;
    CellGateway::DriverHook vendor_drivers_;
    CellGateway::VisionHook vision_versions_;
    std::string scenes_;
    std::map<std::string, std::shared_ptr<CellGateway>> cells_;
};

}  // namespace robonode
