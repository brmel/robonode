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
    explicit CellGateway(std::string world_path) : world_{std::move(world_path)} {
        register_sim_axis(registry_);
        register_mujoco_axis(registry_);
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
            enqueue({Command::kSwap, fam});
            return R"({"ok":true})";
        }
        return R"({"ok":false,"error":"unknown cmd"})";
    }

private:
    struct Command {
        enum Kind { kRun, kSwap } kind;
        std::string family;
    };
    struct NodeInfo {
        std::string id, driver, unit;
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
            } else {
                build_cell(c.family);
            }
        }
    }

    // (Re)build the 7-node cell under a driver family. Worker thread only.
    void build_cell(const std::string& family) {
        const std::string driver = family == "sim" ? "robonode.sim-axis" : "robonode.mujoco-axis";
        auto cell = std::make_unique<Cell>(registry_);
        infos_.clear();

        struct Spec {
            const char* id;
            const char* joint;
            const char* act;
            double lo, hi, v, a, jk;
            const char* unit;
            const char* units_per_m;
        };
        static const Spec specs[] = {
            {"rail-x", "rail", "rail_servo", 0, 1450, 1200, 8000, 120000, "mm", "1000"},
            {"j1", "j1", "j1_servo", -6.28, 6.28, 3, 15, 150, "rad", "1"},
            {"j2", "j2", "j2_servo", -6.28, 6.28, 3, 15, 150, "rad", "1"},
            {"j3", "j3", "j3_servo", -3.14, 3.14, 3, 15, 150, "rad", "1"},
            {"j4", "j4", "j4_servo", -6.28, 6.28, 3, 15, 150, "rad", "1"},
            {"j5", "j5", "j5_servo", -6.28, 6.28, 3, 15, 150, "rad", "1"},
            {"j6", "j6", "j6_servo", -6.28, 6.28, 3, 15, 150, "rad", "1"},
        };
        for (const auto& s : specs) {
            Descriptor d;
            d.id = s.id;
            d.driver = driver;
            d.limits = {s.lo, s.hi, s.v, s.a, s.jk};
            d.command_rate_hz = 1000;
            if (family == "physics") {
                d.config = {{"world", world_}, {"joint", s.joint}, {"actuator", s.act},
                            {"units_per_m", s.units_per_m}};
            }
            if (const auto st = cell->add_node(d); !st.ok()) return;  // leaves old cell
            infos_.push_back({s.id, driver, s.unit, s.lo, s.hi});
        }
        if (!cell->configure_all().ok() || !cell->activate_all().ok()) return;

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
        const CycleHook hook = [this, &k](double t, const std::vector<AxisAdapter*>& ad) {
            if (k++ % 20 == 0) publish_telemetry_from(t, ad);
        };
        cell_->run_waypoints(waypoints, 1000.0, rows, stats, /*settle_s=*/1.5, hook);
    }

    void publish_nodes() {
        nlohmann::json j;
        j["family"] = family_;
        j["nodes"] = nlohmann::json::array();
        for (const auto& n : infos_) {
            j["nodes"].push_back({{"id", n.id},
                                  {"driver", n.driver},
                                  {"unit", n.unit},
                                  {"lo", n.lo},
                                  {"hi", n.hi},
                                  {"state", "active"}});
        }
        std::lock_guard<std::mutex> lk{snap_mtx_};
        nodes_snap_ = j.dump();
    }

    void publish_telemetry_from(double t, const std::vector<AxisAdapter*>& ad) {
        nlohmann::json j;
        j["t"] = t;
        j["family"] = family_;
        auto& pos = j["pos"] = nlohmann::json::array();
        for (auto* a : ad) pos.push_back(a->read().position_mm);
        std::lock_guard<std::mutex> lk{snap_mtx_};
        telem_snap_ = j.dump();
    }

    // Home-pose telemetry (no run in progress).
    void publish_telemetry(double t) {
        std::vector<AxisAdapter*> ad;
        for (auto& n : cell_->nodes()) ad.push_back(n.adapter.get());
        publish_telemetry_from(t, ad);
    }

    std::string world_;
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
    std::string telem_snap_{R"({"t":0,"family":"physics","pos":[0,0,0,0,0,0,0]})"};

    std::thread worker_;
};

}  // namespace robonode
