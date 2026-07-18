#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/celld/app_descriptor.hpp"
#include "robonode/celld/cell.hpp"
#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/log.hpp"
#include "robonode/motion/byo_axis.hpp"
#include "robonode/motion/cartesian.hpp"
#include "robonode/motion/sim_driver.hpp"
#include "robonode/sim_mujoco/mujoco_driver.hpp"
#include "robonode/sim_mujoco/mujoco_kinematics.hpp"
#include "robonode/vision/toy_detector.hpp"

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
        // Kinematics over the 6 arm joints for Cartesian moves (#22). Scratch
        // world; the rail stays at its default, so moveL is relative to rail home.
        if (std::unique_ptr<MujocoKinematics> k;
            MujocoKinematics::create(world_, {"j1", "j2", "j3", "j4", "j5", "j6"}, "tcp", k).ok()) {
            kin_ = std::move(k);
        } else {
            RN_LOG_WARN("kinematics unavailable — Cartesian moves disabled");
        }
        register_toy_detector(vision_reg_);  // basic vision version (ADR-11)
        build_detector();
        run_vision();  // seed the vision snapshot before the worker starts
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
    // Recent log records (newest last) — the followable surface the UI + CLI tail.
    std::string logs_json() { return Log::instance().recent_json(); }
    // Cached vision snapshot (worker runs the detector; kin_ stays worker-only).
    // Carries the chosen version, the versions you can swap to, and detections.
    std::string vision_json() {
        std::lock_guard<std::mutex> lk{snap_mtx_};
        return vision_snap_;
    }
    // The cell's stations (#31) as data — conveyor / deck / pallet capability
    // modules the robot works with, from the descriptor (not code).
    std::string stations_json() {
        auto arr = nlohmann::json::array();
        for (const auto& s : cell_desc_.stations) {
            arr.push_back({{"id", s.id}, {"type", s.type}, {"config", s.config}});
        }
        return arr.dump();
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
        if (cmd == "move_l") {
            if (!kin_) return R"({"ok":false,"error":"kinematics unavailable"})";
            Command c;
            c.kind = Command::kMoveL;
            c.x = j.value("x", 0.0);
            c.y = j.value("y", 0.0);
            c.z = j.value("z", 0.0);
            enqueue(std::move(c));
            return R"({"ok":true})";
        }
        if (cmd == "set_version") {
            const std::string cap = j.value("capability", ""), ver = j.value("version", "");
            if (cap.empty() || ver.empty()) return R"({"ok":false,"error":"need capability+version"})";
            Command c;
            c.kind = Command::kSetVersion;
            c.family = cap;
            c.driver = ver;
            enqueue(std::move(c));
            return R"({"ok":true})";
        }
        if (cmd == "run_app") {
            if (!j.contains("program")) return R"({"ok":false,"error":"empty program"})";
            Command c;
            c.kind = Command::kRunApp;
            for (const auto& s : j.at("program")) {
                AppStep step;
                step.verb = s.value("verb", "");
                if (const auto a = s.find("args"); a != s.end() && a->is_object()) {
                    for (const auto& [k, v] : a->items()) {
                        step.args[k] = v.is_string() ? v.get<std::string>() : v.dump();
                    }
                }
                c.program.push_back(std::move(step));
            }
            if (c.program.empty()) return R"({"ok":false,"error":"empty program"})";
            enqueue(std::move(c));
            return R"({"ok":true})";
        }
        return R"({"ok":false,"error":"unknown cmd"})";
    }

private:
    struct Command {
        enum Kind { kRun, kSwap, kSetNodeDriver, kMoveL, kRunApp, kSetVersion } kind;
        std::string family;            // kSwap · capability name for kSetVersion
        std::string node;              // kSetNodeDriver
        std::string driver;            // kSetNodeDriver
        double x{}, y{}, z{};          // kMoveL (TCP target, metres)
        std::vector<AppStep> program;  // kRunApp (#61/#64)
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
            running_.store(true);  // held across the whole command (a program is one)
            if (c.kind == Command::kRun) {
                RN_LOG_INFO("run: coordinated move");
                run_move();
                RN_LOG_INFO("run: complete");
            } else if (c.kind == Command::kSwap) {
                RN_LOG_INFO("family -> {}", c.family);
                build_cell(c.family);
            } else if (c.kind == Command::kMoveL) {
                run_move_l(c.x, c.y, c.z);
            } else if (c.kind == Command::kRunApp) {
                run_program(c.program);
            } else if (c.kind == Command::kSetVersion) {
                if (c.family == "vision") {
                    vision_version_ = c.driver;
                    build_detector();
                    run_vision();
                    RN_LOG_INFO("vision version -> {}", c.driver);
                } else {
                    RN_LOG_WARN("set_version: unknown capability '{}'", c.family);
                }
            } else {  // kSetNodeDriver
                {
                    std::lock_guard<std::mutex> lk{cell_mtx_};
                    const auto st = cell_ ? cell_->replace_node(c.node, c.driver)
                                          : Status::failure("no cell");
                    if (st.ok()) {
                        RN_LOG_INFO("swap {} -> {}", c.node, c.driver);
                    } else {
                        RN_LOG_WARN("swap {} -> {} rejected: {}", c.node, c.driver, st.message());
                    }
                }
                publish_nodes();
            }
            running_.store(false);
            publish_telemetry(0.0);  // resting snapshot marks the command done (running=false)
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
        RN_LOG_INFO("cell built: {} nodes ({} family)", cell_desc_.nodes.size(), family);
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

    // Cartesian move (#22): straight TCP line from the current arm pose to
    // (x,y,z) in metres. moveL resolves the 6 arm joints (rail held); the plan
    // flows through the same blend/governor/executive as everything else.
    void run_move_l(double x, double y, double z) {
        std::lock_guard<std::mutex> lk{cell_mtx_};
        if (!cell_ || !kin_) return;
        const auto& nodes = cell_->nodes();
        if (nodes.size() != 7) return;
        std::vector<double> q0(6);
        for (int i = 0; i < 6; ++i) q0[i] = nodes[i + 1].adapter->read().position;  // arm joints (rad)
        std::vector<std::vector<double>> jwp;                                        // [joint][step]
        if (const auto st = plan_move_l(*kin_, q0, {x, y, z}, 30, jwp); !st.ok()) {
            RN_LOG_WARN("move_l ({:.3f},{:.3f},{:.3f}) unreachable: {}", x, y, z, st.message());
            return;
        }
        const std::size_t steps = jwp.empty() ? 0 : jwp[0].size();
        const double rail = nodes[0].adapter->read().position;  // held across the move (mm)
        std::vector<std::vector<double>> wp;
        wp.reserve(7);
        wp.emplace_back(steps, rail);
        for (auto& j : jwp) wp.push_back(std::move(j));
        RN_LOG_INFO("move_l -> ({:.3f}, {:.3f}, {:.3f}) m", x, y, z);
        std::vector<std::vector<TelemetryRow>> rows;
        CycleStats stats{};
        std::uint64_t k = 0;
        const CycleHook hook = [this, &k](double t,
                                          const std::vector<std::vector<TelemetryRow>>& rws) {
            if (k++ % 20 == 0) publish_io(t, rws);
        };
        cell_->run_waypoints(wp, 1000.0, rows, stats, /*settle_s=*/1.5, hook);
        RN_LOG_INFO("move_l complete");
    }

    // The program engine (#64): run an app's steps in order on the worker. Each
    // verb reuses a sub-runner that takes cell_mtx_ itself, so steps run one at
    // a time with no lock held here. "pick" is the vision→motion primitive (#61)
    // — move the TCP onto the detected part. The bin-picking app is just data:
    // family → pick → move_l(place).
    void run_program(const std::vector<AppStep>& program) {
        auto num = [](const AppStep& s, const char* k) {
            const auto it = s.args.find(k);
            return it == s.args.end() ? 0.0 : std::strtod(it->second.c_str(), nullptr);
        };
        RN_LOG_INFO("app: {} step(s)", program.size());
        for (const auto& s : program) {
            if (s.verb == "family") {
                const auto it = s.args.find("family");
                build_cell(it == s.args.end() ? "physics" : it->second);
            } else if (s.verb == "run") {
                run_move();
            } else if (s.verb == "move_l") {
                run_move_l(num(s, "x"), num(s, "y"), num(s, "z"));
            } else if (s.verb == "pick") {
                const Vec3 p = run_vision();  // vision → the detected part
                run_move_l(p.x, p.y, p.z);
            } else {
                RN_LOG_WARN("app: unknown verb '{}'", s.verb);
            }
        }
        RN_LOG_INFO("app: complete");
    }

    // Build the selected vision detector (ADR-11). Its scene oracle is the sim's
    // ground-truth site poses — the toy detector's stand-in for a camera; a real
    // or user-sandboxed version swaps in through the registry, nothing above the
    // Detector seam changes. Worker-thread only (kin_ scratch world is not shared).
    void build_detector() {
        VisionContext ctx;
        if (kin_) {
            auto* kin = kin_.get();
            ctx.site_pose = [kin](const std::string& name) { return kin->site_position(name); };
        }
        ctx.config = {{"target", "part"}};
        if (!vision_reg_.make(vision_version_, ctx, detector_).ok()) {
            RN_LOG_WARN("vision version '{}' unavailable", vision_version_);
        }
    }

    // Run the detector, cache the vision snapshot (version + swappable versions +
    // detections), and return the primary part pose for telemetry. Worker-thread
    // only; vision_json reads the cache.
    Vec3 run_vision() {
        const auto ds = detector_ ? detector_->detect() : std::vector<Detection>{};
        const Vec3 part = ds.empty() ? Vec3{} : ds.front().position;
        nlohmann::json j{{"version", vision_version_}, {"available", vision_reg_.names()}};
        auto& arr = j["detections"] = nlohmann::json::array();
        for (const auto& d : ds) {
            arr.push_back({{"label", d.label}, {"pos", {d.position.x, d.position.y, d.position.z}}});
        }
        j["part"] = {part.x, part.y, part.z};
        std::lock_guard<std::mutex> lk{snap_mtx_};
        vision_snap_ = j.dump();
        return part;
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

    // Live per-node I/O during a run: actual (out), governed target (in), and
    // following error — the clean input/output the UI inspector shows.
    void publish_io(double t, const std::vector<std::vector<TelemetryRow>>& rws) {
        std::vector<double> pos, tgt, err;
        for (const auto& r : rws) {
            const auto& row = r.back();
            pos.push_back(row.actual_position);
            tgt.push_back(row.governed_position);
            err.push_back(row.following_error);
        }
        write_telem(t, pos, tgt, err, arm_joints(rws));
    }

    // Resting telemetry (no run): actual only, target = actual, error = 0.
    void publish_telemetry(double t) {
        std::vector<double> pos, err, q;
        for (auto& n : cell_->nodes()) {
            const double p = n.adapter->read().position;
            pos.push_back(p);
            err.push_back(0.0);
            q.push_back(p);
        }
        std::vector<double> arm = q.size() >= 7 ? std::vector<double>{q.begin() + 1, q.begin() + 7}
                                                : std::vector<double>{};
        write_telem(t, pos, /*target=*/pos, err, arm);
    }

    // Assemble + publish one telemetry snapshot: per-node I/O, the TCP (FK of the
    // arm joints, #22), the vision target (#6), and whether a command is running.
    void write_telem(double t, const std::vector<double>& pos, const std::vector<double>& tgt,
                     const std::vector<double>& err, const std::vector<double>& arm_q) {
        const Vec3 part = run_vision();  // worker-side detector, also caches vision_snap_
        nlohmann::json j{{"t", t}, {"family", family_}, {"pos", pos}, {"target", tgt}, {"err", err},
                         {"vision", {{"part", {part.x, part.y, part.z}}}},
                         {"running", running_.load()}};
        if (kin_ && arm_q.size() == 6) {
            const auto p = kin_->tcp_position(arm_q);
            j["tcp"] = {p.x, p.y, p.z};
        }
        std::lock_guard<std::mutex> lk{snap_mtx_};
        telem_snap_ = j.dump();
    }
    static std::vector<double> arm_joints(const std::vector<std::vector<TelemetryRow>>& rws) {
        std::vector<double> q;
        for (std::size_t i = 1; i < rws.size() && i < 7; ++i) q.push_back(rws[i].back().actual_position);
        return q;
    }

    std::string world_;
    CellDescriptor cell_desc_;
    std::unique_ptr<MujocoKinematics> kin_;  // arm kinematics for Cartesian moves (#22)
    VisionRegistry vision_reg_;              // swappable vision versions (ADR-11, #6)
    std::unique_ptr<Detector> detector_;     // the selected version (worker-thread only)
    std::string vision_version_{"robonode.toy-detector"};
    DriverRegistry registry_;
    std::mutex cell_mtx_;
    std::unique_ptr<Cell> cell_;
    std::string family_{"physics"};
    std::vector<NodeInfo> infos_;

    std::mutex cmd_mtx_;
    std::condition_variable cmd_cv_;
    std::deque<Command> cmds_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> running_{false};  // a command (incl. a whole app program) is executing

    std::mutex snap_mtx_;
    std::string nodes_snap_{R"({"family":"physics","nodes":[]})"};
    std::string telem_snap_{
        R"({"t":0,"family":"physics","pos":[0,0,0,0,0,0,0],)"
        R"("target":[0,0,0,0,0,0,0],"err":[0,0,0,0,0,0,0]})"};
    std::string vision_snap_{R"({"version":"robonode.toy-detector","available":[],"detections":[],"part":[0,0,0]})"};

    std::thread worker_;
};

}  // namespace robonode
