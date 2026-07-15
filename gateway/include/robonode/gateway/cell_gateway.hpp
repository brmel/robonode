#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/celld/cell.hpp"
#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/motion/byo_axis.hpp"
#include "robonode/motion/sim_driver.hpp"
#include "robonode/sim_mujoco/mujoco_driver.hpp"

namespace robonode {

// The live bridge between a running cell and the web app. A single worker
// thread OWNS the cell exclusively (so there is never concurrent access to
// MuJoCo's data); HTTP handlers only enqueue commands and read thread-safe
// JSON snapshots. This is the seam the roadmap Zenoh/gRPC surface replaces —
// the gateway shape (node tree + telemetry out, commands in) stays.
//
// Driver families demonstrate the forced interface: every node is rebuilt
// through the DriverRegistry under a chosen driver ("physics" = mujoco-axis,
// "sim" = sim-axis). Both satisfy AxisAdapter, so a node's implementation is
// swapped without the UI or motion code knowing — and an unregistered driver
// is simply refused.
class CellGateway {
public:
    // world_path = the MJCF the physics driver loads; cell_path = the node tree
    // as data (#28). The cell is descriptor-driven — no node specs in code.
    CellGateway(std::string world_path, const std::string& cell_path)
        : world_{std::move(world_path)} {
        register_sim_axis(registry_);       // robonode.sim-axis  (filter, fast)
        register_sim_axis_soft(registry_);  // robonode.sim-axis-soft (sluggish)
        register_mujoco_axis(registry_);    // robonode.mujoco-axis (physics)
        register_byo_axis(registry_);       // robonode.byo-example (bring-your-own template, #23)
        (void)load_cell_descriptor(cell_path, cell_desc_);  // empty on failure → empty cell
        build_cell("physics");
        worker_ = std::thread([this] { worker_loop(); });
    }

    ~CellGateway() {
        stop_.store(true);
        cmd_cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

    CellGateway(const CellGateway&) = delete;
    CellGateway& operator=(const CellGateway&) = delete;

    // --- HTTP-handler-facing (thread-safe) ---

    std::string nodes_json() {
        std::lock_guard<std::mutex> lk{snap_mtx_};
        return nodes_snap_;
    }
    std::string telemetry_json() {
        std::lock_guard<std::mutex> lk{snap_mtx_};
        return telem_snap_;
    }

    // Parse a command body and enqueue it. Returns a JSON result (accepted /
    // error). Recognised: {"cmd":"run"} and {"cmd":"driver","family":"sim"|"physics"}.
    std::string submit_command(const std::string& body) {
        auto j = nlohmann::json::parse(body, nullptr, false);
        if (j.is_discarded() || !j.contains("cmd")) {
            return R"({"ok":false,"error":"bad command"})";
        }
        const std::string cmd = j.value("cmd", "");
        if (cmd == "run") {
            enqueue({Command::kRun, {}});
            return R"({"ok":true})";
        }
        if (cmd == "driver") {
            const std::string fam = j.value("family", "");
            if (fam != "sim" && fam != "physics") {
                return R"({"ok":false,"error":"unknown family"})";
            }
            enqueue({Command::kSwap, fam, {}, {}});
            return R"({"ok":true})";
        }
        if (cmd == "set_driver") {
            const std::string node = j.value("node", "");
            const std::string driver = j.value("driver", "");
            if (node.empty() || driver.empty()) {
                return R"({"ok":false,"error":"need node+driver"})";
            }
            enqueue({Command::kSetNodeDriver, {}, node, driver});
            return R"({"ok":true})";
        }
        return R"({"ok":false,"error":"unknown cmd"})";
    }

private:
    struct Command {
        enum Kind { kRun, kSwap, kSetNodeDriver } kind;
        std::string family;  // kSwap
        std::string node;    // kSetNodeDriver
        std::string driver;  // kSetNodeDriver
    };
    struct NodeInfo {
        std::string id, unit;
        double lo, hi;
    };

    void enqueue(Command c) {
        {
            std::lock_guard<std::mutex> lk{cmd_mtx_};
            cmds_.push_back(std::move(c));
        }
        cmd_cv_.notify_one();
    }

    void worker_loop() {
        while (!stop_.load()) {
            Command c;
            {
                std::unique_lock<std::mutex> lk{cmd_mtx_};
                cmd_cv_.wait(lk, [this] { return stop_.load() || !cmds_.empty(); });
                if (stop_.load()) return;
                c = std::move(cmds_.front());
                cmds_.pop_front();
            }
            if (c.kind == Command::kRun) {
                run_move();
            } else if (c.kind == Command::kSwap) {
                build_cell(c.family);
            } else {  // kSetNodeDriver
                {
                    std::lock_guard<std::mutex> lk{cell_mtx_};
                    if (cell_) (void)cell_->replace_node(c.node, c.driver);
                }
                publish_nodes();
                publish_telemetry(0.0);
            }
        }
    }

    // (Re)build the cell from the descriptor under a driver family. Worker
    // thread only. The node tree is data (cell_desc_), not code (#28).
    void build_cell(const std::string& family) {
        const std::string driver = family == "sim" ? "robonode.sim-axis" : "robonode.mujoco-axis";
        auto cell = std::make_unique<Cell>(registry_);
        infos_.clear();

        for (const auto& s : cell_desc_.nodes) {
            Descriptor d;
            d.id = s.id;
            d.driver = driver;
            d.limits = s.limits;
            d.command_rate_hz = 1000;
            // Always carry the MuJoCo config so ANY node can later be swapped
            // to the physics driver (sim drivers ignore it). This is what lets
            // per-node version-swapping work in both directions.
            d.config = {{"world", world_}, {"joint", s.joint}, {"actuator", s.actuator},
                        {"units_per_m", std::to_string(s.units_per_m)}};
            if (const auto st = cell->add_node(d); !st.ok()) return;  // leaves old cell
            infos_.push_back({s.id, s.unit, s.limits.position_min, s.limits.position_max});
        }
        if (cell_desc_.nodes.empty() || !cell->configure_all().ok() || !cell->activate_all().ok()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lk{cell_mtx_};
            cell_ = std::move(cell);
            family_ = family;
        }
        publish_nodes();
        publish_telemetry(0.0);  // home pose
    }

    void run_move() {
        std::lock_guard<std::mutex> lk{cell_mtx_};
        if (!cell_) return;
        // A visibly articulating reach: rail traverses while the shoulder
        // drops and the elbow bends into an L, all on one clock.
        const std::vector<std::vector<double>> waypoints = {
            {0, 700, 400},   {0, 0.8, 0.5},   {0, -1.2, -0.9}, {0, 1.6, 1.2},
            {0, 0.6, 0.4},   {0, -0.8, -0.5}, {0, 0.5, 0.3}};
        std::vector<std::vector<TelemetryRow>> rows;
        CycleStats stats{};
        std::uint64_t k = 0;
        // Stream ~50 Hz: publish every 20th cycle of the 1 kHz loop.
        const CycleHook hook = [this, &k](double t,
                                          const std::vector<std::vector<TelemetryRow>>& rws) {
            if (k++ % 20 == 0) publish_io(t, rws);
        };
        cell_->run_waypoints(waypoints, 1000.0, rows, stats, /*settle_s=*/1.5, hook);
    }

    // Worker-thread only (reads cell_). Advertises the driver versions a node
    // can be swapped to (registry_.names()) and each node's current driver.
    void publish_nodes() {
        nlohmann::json j;
        j["family"] = family_;
        j["available"] = registry_.names();
        auto& arr = j["nodes"] = nlohmann::json::array();
        if (cell_) {
            const auto& nodes = cell_->nodes();
            for (std::size_t i = 0; i < nodes.size() && i < infos_.size(); ++i) {
                arr.push_back({{"id", nodes[i].id},
                               {"driver", nodes[i].descriptor.driver},
                               {"unit", infos_[i].unit},
                               {"lo", infos_[i].lo},
                               {"hi", infos_[i].hi},
                               {"state", "active"}});
            }
        }
        std::lock_guard<std::mutex> lk{snap_mtx_};
        nodes_snap_ = j.dump();
    }

    // Live per-node I/O during a run: actual (out), target (governed in), and
    // following error — the clean input/output the UI inspector shows.
    void publish_io(double t, const std::vector<std::vector<TelemetryRow>>& rws) {
        nlohmann::json j;
        j["t"] = t;
        j["family"] = family_;
        auto& pos = j["pos"] = nlohmann::json::array();
        auto& tgt = j["target"] = nlohmann::json::array();
        auto& err = j["err"] = nlohmann::json::array();
        for (const auto& r : rws) {
            const auto& row = r.back();
            pos.push_back(row.actual_position);
            tgt.push_back(row.governed_position);
            err.push_back(row.following_error);
        }
        std::lock_guard<std::mutex> lk{snap_mtx_};
        telem_snap_ = j.dump();
    }

    // Home-pose telemetry (no run in progress): actual only, target = actual,
    // error = 0.
    void publish_telemetry(double t) {
        nlohmann::json j;
        j["t"] = t;
        j["family"] = family_;
        auto& pos = j["pos"] = nlohmann::json::array();
        auto& tgt = j["target"] = nlohmann::json::array();
        auto& err = j["err"] = nlohmann::json::array();
        for (auto& n : cell_->nodes()) {
            const double p = n.adapter->read().position;
            pos.push_back(p);
            tgt.push_back(p);
            err.push_back(0.0);
        }
        std::lock_guard<std::mutex> lk{snap_mtx_};
        telem_snap_ = j.dump();
    }

    std::string world_;
    CellDescriptor cell_desc_;
    DriverRegistry registry_;
    std::mutex cell_mtx_;
    std::unique_ptr<Cell> cell_;
    std::string family_{"physics"};
    std::vector<NodeInfo> infos_;

    std::mutex cmd_mtx_;
    std::condition_variable cmd_cv_;
    std::deque<Command> cmds_;
    std::atomic<bool> stop_{false};

    std::mutex snap_mtx_;
    std::string nodes_snap_{R"({"family":"physics","nodes":[]})"};
    std::string telem_snap_{
        R"({"t":0,"family":"physics","pos":[0,0,0,0,0,0,0],)"
        R"("target":[0,0,0,0,0,0,0],"err":[0,0,0,0,0,0,0]})"};

    std::thread worker_;
};

}  // namespace robonode
