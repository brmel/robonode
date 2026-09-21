#pragma once

#include <map>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/celld/app_descriptor.hpp"
#include "robonode/gateway/camera_capability.hpp"
#include "robonode/gateway/capability_registry.hpp"
#include "robonode/gateway/cell_runtime.hpp"
#include "robonode/gateway/cell_supervisor.hpp"
#include "robonode/gateway/command_bus.hpp"
#include "robonode/gateway/command_router.hpp"
#include "robonode/gateway/control_capability.hpp"
#include "robonode/gateway/model_view.hpp"
#include "robonode/gateway/motion_service.hpp"
#include "robonode/gateway/program_runner.hpp"
#include "robonode/gateway/run_recorder.hpp"
#include "robonode/gateway/sim_gripper.hpp"
#include "robonode/gateway/scene_view.hpp"
#include "robonode/gateway/station_ops.hpp"
#include "robonode/vision/bmp.hpp"
#include "robonode/gateway/telemetry_publisher.hpp"
#include "robonode/gateway/tracking_capability.hpp"
#include "robonode/gateway/trajectory_capability.hpp"
#include "robonode/gateway/vision_capability.hpp"
#include "robonode/core/settings.hpp"
#include "robonode/log.hpp"
#include "robonode/motion/byo_axis.hpp"
#include "robonode/motion/sim_driver.hpp"
#include "robonode/sim_mujoco/mujoco_driver.hpp"
#include "robonode/celld/conveyor.hpp"
#include "robonode/sim_mujoco/mujoco_kinematics.hpp"
#include "robonode/sim_mujoco/mujoco_scene.hpp"

namespace robonode {

// The live cell behind every surface. Composition only: the runtime owns the
// cell, the capabilities own the algorithms, the router owns the wire contract,
// the bus owns the single worker thread. This class wires them together and
// assembles telemetry.
class CellGateway {
public:
    // Vendor drivers are the composition root's choice, not the gateway's: an
    // app links exactly the hardware it supports and registers it here.
    using DriverHook = std::function<void(DriverRegistry&)>;
    using VisionHook = std::function<void(VisionRegistry&)>;

    // `id` is what this cell is called on the wire. It is not decoration: every
    // record this gateway's threads produce is tagged with it, which is what
    // makes `GET /logs?cell=x` an answer about x rather than about everything.
    CellGateway(std::string worlds_dir, const std::string& cell_path, Settings settings = {},
                DriverHook vendor_drivers = {}, VisionHook vision_versions = {},
                std::string scenes_dir = {}, std::string id = "main")
        : id_{std::move(id)}, cfg_{settings}, runtime_{std::move(worlds_dir), registry_},
          tracking_{cfg_.tracking}, workpiece_{cfg_.app.grasp_reach_m},
          motion_{runtime_, trajectory_, control_, cfg_.motion},
          gripper_{workpiece_,
                   [this](Grip grip) {
                       return scene_.with([this, grip](Scene& live) {
                           return live.constrain(cfg_.app.grip_constraint, grip);
                       });
                   }},
          program_{motion_,
                   vision_,
                   tracking_,
                   gripper_,
                   runtime_.descriptor(),
                   [this](const std::string& f) { return build(f); },
                   [this](const std::string& cap, const std::string& v) { return select(cap, v); },
                   [this] { return clock_s(); },
                   [this](const std::string& id, Belt belt) { return stations_.run(id, belt); },
                   [this](const std::string& id) { return stations_.deliver(id); },
                   [this](double seconds) { advance_scene(seconds); },
                   cfg_.app},
          stations_{[this](const std::function<Status(Scene&)>& fn) { return scene_.with(fn); },
                    [this]() -> const std::vector<StationSpec>& {
                        return runtime_.descriptor().stations;
                    }},
          recorder_{cfg_.recorder.dir},
          router_{[this](Command c) { return accept(std::move(c)); }} {
        const Log::Scope mine{id_};
        runtime_.set_carrier_weight(cfg_.ik.carrier_weight);
        runtime_.set_scenes_dir(std::move(scenes_dir));
        trajectory_.configure(cfg_);
        register_drivers();
        if (vendor_drivers) vendor_drivers(registry_);
        if (vision_versions) vision_.add_versions(vision_versions);
        if (const auto st = runtime_.load(cell_path); !st.ok()) {
            RN_LOG_ERROR("cell descriptor '{}' unusable: {}", cell_path, st.message());
            supervisor_.fault(st.message());
        }
        attach_kinematics();
        // Vision reads the LIVE scene: a scratch copy never moves, so a detector
        // built on one could never see a part travel down a belt.
        vision_.set_site_pose([this](const std::string& n) { return scene_.site(n); });
        vision_.set_clock([this] { return clock_s(); });
        vision_.set_clutter(clutter_sites());
        camera_.set_scene([this](const std::string& n) { return scene_.site(n); },
                          [this] { return clock_s(); });
        camera_.set_view(vision_.target_site(), clutter_sites());
        vision_.set_camera_source([this] { return camera_.make(); });
        program_.set_step_sink([this](const std::string& what) {
            note_progress(what, running_app());
            publish_telemetry(0.0);
        });
        register_capabilities();
        register_verbs();
        motion_.set_hook(stream_hook());
        // Recording is a wiring decision, not a branch every run pays: when it
        // is off, nothing is connected to the run sink at all.
        if (cfg_.recorder.enabled) {
            motion_.set_run_sink([this](const std::string& label,
                                        const std::vector<std::string>& axes,
                                        const std::vector<std::vector<TelemetryRow>>& rows,
                                        double hz) {
                recorder_.record(label, axes, rows, hz, active_versions());
            });
        }
        motion_.set_cancel(&supervisor_.token());
        motion_.set_quality_sink([this](const MotionService::RunQuality& q) {
            std::lock_guard<std::mutex> lk{quality_mtx_};
            quality_ = q;
            ++quality_run_;  // this record belongs to a motion that just ran
        });
        // Sample before the first detector runs: a capability publishes what it
        // saw, and what it saw of an unsampled world is nothing. No lock yet —
        // the cell is still being built, and there is no other thread.
        scene_.sample_now();
        vision_.rebuild();
        trajectory_.rebuild();
        (void)build(default_family());
        scene_.sample_now();
        bus_.emplace(
            [this](Command& c, std::uint64_t id) {
                const Log::Scope mine{id_};
                execute(c, id);
            },
            [this] { publish_telemetry(0.0); }, cfg_.gateway.queue_capacity);
        idle_ = std::thread{[this] {
            const Log::Scope mine{id_};
            keep_scene_alive();
        }};
    }

    ~CellGateway() {
        stop_idle_.store(true);
        if (idle_.joinable()) idle_.join();
    }

    CellGateway(const CellGateway&) = delete;
    CellGateway& operator=(const CellGateway&) = delete;

    std::string nodes_json() { return telemetry_.nodes_json(); }
    std::string telemetry_json() { return telemetry_.telemetry_json(); }
    std::string logs_json() { return Log::instance().recent_json(id_); }
    std::string capabilities_json() { return caps_.json(); }
    [[nodiscard]] bool has_capability(const std::string& id) const {
        return caps_.find(id) != nullptr;
    }
    std::string capability_json(const std::string& id) { return caps_.json_of(id); }
    // The model's own kinematic chain (see model_view.hpp).
    std::string model_json() {
        return robonode::model_json(runtime_.world(), tcp_site(), vision_.target_site());
    }

    std::string stations_json() {
        auto arr = nlohmann::json::array();
        for (const auto& s : runtime_.descriptor().stations) {
            arr.push_back({{"id", s.id}, {"type", s.type}, {"config", s.config}});
        }
        return arr.dump();
    }

    // The frame the detector reads, as an image, with a crosshair where the
    // platform currently believes the part is. Tuning a detector without seeing
    // its input is guesswork; this is the sensor's own view, not a render of
    // what we already decided.
    //
    // Its own camera instance: the detector owns the one it reads, and a feed
    // that stole frames from it would change the behaviour it exists to show.
    std::string camera_bmp() {
        std::lock_guard<std::mutex> lk{feed_mtx_};
        if (!feed_) feed_ = camera_.make();
        if (!feed_) return {};
        // A camera has latency: it delivers the newest frame old enough to have
        // been processed, so a sensor that has just been built has nothing to
        // give yet. Waiting out one exposure beats answering "no camera".
        auto frame = feed_->grab();
        for (int attempt = 0; frame.empty() && attempt < 8; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds{25});
            frame = feed_->grab();
        }
        if (frame.empty()) return {};
        if (const auto seen = vision_.last_seen()) {
            if (const auto at = feed_->model().project(*seen)) mark(frame, at->first, at->second);
        }
        return bmp_of(frame);
    }

    // What the last run cost: where it tracked worst, how many cycles it took,
    // whether the RT loop kept up. The recorder's own summary (if recording is
    // on) rides along, because a comparison you cannot reproduce is an anecdote.
    std::string last_run_json() {
        std::lock_guard<std::mutex> lk{quality_mtx_};
        auto j = nlohmann::json::parse(recorder_.summary_json(), nullptr, false);
        if (j.is_discarded() || !j.is_object()) j = nlohmann::json::object();
        // Which motion this record is FROM. Without it the record is just "the
        // last one", and a caller that asks after a trial which never moved
        // gets a previous run's numbers with nothing marking them as such —
        // evidence about one algorithm, printed under another one's name.
        j["run"] = quality_run_;
        j["label"] = quality_.label;
        j["worst_axis"] = {{"id", quality_.worst.id},
                           {"unit", quality_.worst.unit},
                           {"following_error", quality_.worst.error}};
        j["cycles"] = quality_.stats.cycles;
        j["overruns"] = quality_.stats.overruns;
        j["jitter_p99_us"] = quality_.stats.p99_jitter_us;
        j["stopped_early"] = quality_.stats.stopped_early;
        // Did the loop keep up? The numbers were always here; nothing said what
        // they were supposed to be, so a regression was a log line nobody read.
        const double cycles = static_cast<double>(quality_.stats.cycles);
        const double overrun_fraction =
            cycles > 0 ? static_cast<double>(quality_.stats.overruns) / cycles : 0.0;
        j["overrun_fraction"] = overrun_fraction;
        j["within_budget"] = cycles == 0 || (overrun_fraction <= cfg_.gateway.max_overrun_fraction &&
                                             quality_.stats.p99_jitter_us <=
                                                 cfg_.gateway.max_jitter_p99_us);
        return j.dump();
    }

    // What a program may say, straight from the table that executes it.
    std::string verbs_json() { return program_.catalogue().dump(); }
    [[nodiscard]] Status check_program(const std::vector<AppStep>& program) const {
        return program_.check(program);
    }

    // Wait for the next telemetry publication (see TelemetryPublisher).
    std::uint64_t await_telemetry(std::uint64_t seen, std::chrono::milliseconds timeout) {
        return telemetry_.await_change(seen, timeout);
    }
    [[nodiscard]] std::uint64_t telemetry_sequence() { return telemetry_.sequence(); }

    std::string submit_command(const std::string& body) { return router_.dispatch(body); }
    std::string submit(const std::string& verb, const nlohmann::json& args) {
        return router_.dispatch(verb, args);
    }

    // Typed twin of the define_module verb — the store path needs the resolved
    // version name back, which the wire form returns in its acknowledgement.
    Status define_module(const std::string& capability, const std::string& name,
                         const std::string& source, Select select, std::string& version_out,
                         CommandAck* ack_out = nullptr) {
        Command cmd;
        cmd.name = "define_module";
        if (const auto st = build_define(capability, name, source, select, cmd); !st.ok()) return st;
        version_out = cmd.ack.value("version", std::string{});
        const auto ack = accept(std::move(cmd));
        if (ack_out != nullptr) *ack_out = ack;
        return ack.accepted ? Status::success() : Status::failure(ack.reason);
    }

    // Typed twin of the run_app verb, for a program already parsed from a store.
    // Returns the acknowledgement so this path is as observable as the wire one.
    CommandAck run_program(std::vector<AppStep> program, std::string label = {}) {
        if (program.empty()) return {0, false, "empty program", 0};
        Command cmd;
        cmd.name = "run_app";
        // Which application is running is state a surface should show, not
        // something it has to remember from the command it sent.
        cmd.run = [this, program = std::move(program), label = std::move(label)] {
            note_progress(step(), label);
            const auto st = program_.run(program);
            note_progress({}, {});
            return st;
        };
        return accept(std::move(cmd));
    }

private:
    // Acknowledge and immediately refresh the progress fields, so a caller that
    // reads telemetry right after submitting never sees a pre-command snapshot.
    CommandAck accept(Command c) {
        const std::string key = c.key;
        const auto ack = bus_->submit(std::move(c), key);
        telemetry_.update_progress(progress());
        return ack;
    }

    void register_drivers() {
        register_sim_axis(registry_);
        register_sim_axis_soft(registry_);
        pool_ = register_mujoco_axis(registry_);
        register_byo_axis(registry_);
    }

    // WHICH physics engine this build runs is decided here and in nothing else
    // above it. The seams (Scene, Kinematics) have always been abstract, but the
    // gateway used to name MujocoScene, MujocoWorld and MujocoKinematics
    // directly and store the concrete kinematics type — so the engine was
    // swappable in principle and not in practice. These two functions are the
    // whole of the coupling now; everything else holds the seam.
    static std::unique_ptr<Kinematics> open_kinematics(const std::string& world,
                                                       const std::vector<std::string>& joints,
                                                       const std::string& tcp) {
        std::unique_ptr<MujocoKinematics> k;
        if (!MujocoKinematics::create(world, joints, tcp, k).ok()) return nullptr;
        return k;
    }

    // The live world belongs to SceneView; what stays here is the cell's part of
    // the arrangement — its lock, which world it is on, and how one is opened.
    SceneView make_scene_view() {
        return SceneView{
            [this](const std::function<Status()>& fn) {
                return runtime_.with_cell([&](Cell&) { return fn(); });
            },
            [this] { return runtime_.world(); },
            [this](const std::string& world) -> std::unique_ptr<Scene> {
                std::shared_ptr<MujocoWorld> live;
                if (pool_ == nullptr || !pool_->get(world, live).ok()) return nullptr;
                return std::make_unique<MujocoScene>(std::move(live));
            }};
    }

    void tick_scene(double dt) {
        (void)scene_.with([&](Scene& live) {
            live.advance(dt);
            keep_lines_fed(live);
            return Status::success();
        });
        publish_telemetry(0.0);
    }

    // Stations get a say on every tick: a conveyor that has carried its part off
    // the end puts a fresh one at the head, so the line keeps presenting work.
    // Never while the gripper holds it — recirculating a part out of the tool
    // would be the station undoing the robot.
    void keep_lines_fed(Scene& live) {
        if (workpiece_.held()) return;
        for (const auto& spec : runtime_.descriptor().stations) {
            if (spec.type != "conveyor") continue;
            Conveyor belt{spec, live};
            (void)belt.recirculate();
        }
    }

    // Seconds since this cell booted — the clock a tracker timestamps with.
    [[nodiscard]] double clock_s() const {
        return std::chrono::duration<double>{std::chrono::steady_clock::now() - born_}.count();
    }

    // Between commands nothing drives the physics, so a belt would freeze and a
    // "moving" target would not move. This advances the world while the cell is
    // idle; the executive owns it during a run, and the cell lock keeps the two
    // from overlapping.
    void keep_scene_alive() {
        const auto period = std::chrono::milliseconds{cfg_.gateway.stream_period_ms};
        auto last = std::chrono::steady_clock::now();
        while (!stop_idle_.load()) {
            std::this_thread::sleep_for(period);
            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>{now - last}.count();
            last = now;
            if (supervisor_.state_name() != "idle") continue;
            tick_scene(dt);
        }
    }

    // Let time pass mid-program. The idle ticker only runs between commands, so
    // a step that waits (for a part to come into view, for a belt to bring one)
    // must advance the world itself or nothing would ever change.
    void advance_scene(double seconds) {
        if (seconds <= 0.0) return;
        const auto step = std::chrono::milliseconds{cfg_.gateway.stream_period_ms};
        const double dt = std::chrono::duration<double>{step}.count();
        for (double elapsed = 0.0; elapsed < seconds; elapsed += dt) {
            tick_scene(dt);
            std::this_thread::sleep_for(step);
        }
    }

    // A planner holds a REFERENCE to the kinematics it solves against, so
    // replacing the chain (a scene swap builds a new world) without rebuilding
    // the planner leaves it pointing at a destroyed object — a segfault on the
    // next Cartesian move. Whoever changes the kinematics rebuilds what reads
    // them; that is why both live in this one function.
    void attach_kinematics() {
        auto k = open_kinematics(runtime_.world(), joint_names(), tcp_site());
        if (!k) {
            RN_LOG_WARN("kinematics unavailable — Cartesian moves disabled");
            return;
        }
        // Telemetry asks the chain where the tool is on every frame while this
        // replaces it. Shared, and swapped under a lock: a reader holds the
        // chain it started with until it is done, and the planner is rebuilt
        // against the new one before anything can plan against the old.
        std::shared_ptr<Kinematics> chain = std::move(k);
        {
            std::lock_guard<std::mutex> lk{kin_mtx_};
            kin_ = chain;
        }
        trajectory_.set_kinematics(chain.get());
        trajectory_.rebuild();
    }

    [[nodiscard]] std::shared_ptr<Kinematics> kinematics() const {
        std::lock_guard<std::mutex> lk{kin_mtx_};
        return kin_;
    }

    // The model joint names of the first robot's chain, carrier first.
    [[nodiscard]] std::vector<std::string> joint_names() const {
        const auto& robots = runtime_.descriptor().robots;
        if (robots.empty()) return {};
        std::vector<std::string> chain = robots.front().carrier;
        chain.insert(chain.end(), robots.front().joints.begin(), robots.front().joints.end());
        std::vector<std::string> out;
        for (const auto& id : chain) {
            for (const auto& n : runtime_.descriptor().nodes) {
                if (n.id == id) out.push_back(n.joint.empty() ? n.id : n.joint);
            }
        }
        return out;
    }

    // Stations may declare decoys the camera sees — clutter is cell data, not a
    // constant in a detector.
    [[nodiscard]] std::string clutter_sites() const {
        std::string sites;
        for (const auto& st : runtime_.descriptor().stations) {
            const auto it = st.config.find("clutter_site");
            if (it == st.config.end()) continue;
            if (!sites.empty()) sites += ",";
            sites += it->second;
        }
        return sites;
    }

    [[nodiscard]] std::string tcp_site() const {
        const auto& robots = runtime_.descriptor().robots;
        return robots.empty() ? "tcp" : robots.front().tcp_site;
    }

    void register_capabilities() {
        caps_.publish_trials(cfg_.compare);
        caps_.add(camera_);
        caps_.add(vision_);
        caps_.add(tracking_);
        caps_.add(trajectory_);
        caps_.add(control_);
    }

    void register_verbs() {
        register_motion_verbs();
        register_capability_verbs();
        register_scene_verbs();
        register_program_verbs();
        // Verbs that are pure delegation live with the thing they command, not
        // in the class that happens to own it.
        robonode::register_supervisor_verbs(router_, supervisor_, [this] {
            return bus_ ? bus_->discard_pending() : std::size_t{0};
        });
        robonode::register_tool_verbs(router_, gripper_);
    }

    void register_motion_verbs() {
        router_.on("run", [this](const nlohmann::json& a, Command& c) {
            const std::string id = a.value("motion", default_motion());
            c.run = [this, id] { return motion_.run_motion(id); };
            return id.empty() ? Status::failure("cell declares no motions") : Status::success();
        });
        router_.on("driver", [this](const nlohmann::json& a, Command& c) {
            const std::string family = a.value("family", "");
            if (runtime_.descriptor().family(family) == nullptr) {
                return Status::failure("unknown family '" + family + "'");
            }
            c.key = "driver";
            c.run = [this, family] { return build(family); };
            return Status::success();
        });
        router_.on("set_driver", [this](const nlohmann::json& a, Command& c) {
            const std::string node = a.value("node", ""), driver = a.value("driver", "");
            if (node.empty() || driver.empty()) return Status::failure("need node+driver");
            c.key = "set_driver:" + node;
            c.run = [this, node, driver] { return swap_node(node, driver); };
            return Status::success();
        });
        router_.on("move_pose", [this](const nlohmann::json& a, Command& c) {
            if (!kinematics()) return Status::failure("kinematics unavailable");
            const Pose target{{a.value("x", 0.0), a.value("y", 0.0), a.value("z", 0.0)},
                              Quat{a.value("qw", 1.0), a.value("qx", 0.0), a.value("qy", 0.0),
                                   a.value("qz", 0.0)}
                                  .normalized()};
            c.run = [this, target] { return motion_.move_pose(target); };
            return Status::success();
        });
        router_.on("move_l", [this](const nlohmann::json& a, Command& c) {
            if (!kinematics()) return Status::failure("kinematics unavailable");
            const Vec3 target{a.value("x", 0.0), a.value("y", 0.0), a.value("z", 0.0)};
            c.run = [this, target] { return motion_.move_l(target); };
            return Status::success();
        });
        router_.on("jog", [this](const nlohmann::json& a, Command& c) {
            const std::string axis = a.value("axis", "");
            if (axis.empty()) return Status::failure("need axis");
            if (!a.contains("target")) return Status::failure("need target");
            const double target = a.value("target", 0.0);
            c.run = [this, axis, target] { return motion_.jog(axis, target); };
            return Status::success();
        });
        router_.on("run_app", [this](const nlohmann::json& a, Command& c) {
            auto program = a.contains("program") ? parse_program(a.at("program"))
                                                 : std::vector<AppStep>{};
            if (program.empty()) return Status::failure("empty program");
            c.run = [this, program = std::move(program)] { return program_.run(program); };
            return Status::success();
        });
    }

    void register_capability_verbs() {
        router_.on("set_version", [this](const nlohmann::json& a, Command& c) {
            auto* cap = caps_.find(a.value("capability", ""));
            const std::string version = a.value("version", "");
            if (cap == nullptr) return Status::failure("unknown capability");
            if (version.empty()) return Status::failure("need version");
            c.key = "set_version:" + cap->descriptor().id;
            c.run = [cap, version] { return cap->select(version); };
            return Status::success();
        });
        router_.on("define_module", [this](const nlohmann::json& a, Command& c) {
            return build_define(a.value("capability", ""), a.value("name", ""),
                                a.value("source", ""),
                                a.value("select", true) ? Select::kNow : Select::kLater, c);
        });
    }

    // Every task step is also a command. An operator (or an agent) debugging a
    // cell should not have to author an application to try one pick, run the
    // belt, or deliver a fresh part — and a step that works here is the same
    // step an app runs, because it is the same code.
    void register_program_verbs() {
        for (const char* verb : {"pick", "place", "intercept", "conveyor", "deliver", "motion"}) {
            router_.on(verb, [this, verb](const nlohmann::json& a, Command& c) {
                AppStep step{verb, {}};
                for (const auto& [key, value] : a.items()) {
                    if (key == "cmd") continue;
                    step.args[key] = value.is_string() ? value.get<std::string>() : value.dump();
                }
                c.run = [this, step] { return program_.run({step}); };
                return Status::success();
            });
        }
    }

    // Change the scenario on a running platform: the scene is data, so trying
    // another one is a command like any other — identified, queued behind
    // whatever is moving, and reported when it has actually taken effect.
    void register_scene_verbs() {
        router_.on("load_scene", [this](const nlohmann::json& a, Command& c) {
            const std::string scene = a.value("scene", ""), label = a.value("label", "scene");
            if (scene.empty()) return Status::failure("need a scene descriptor");
            c.key = "load_scene";
            c.run = [this, scene, label] { return swap_scene(scene, label); };
            return Status::success();
        });
    }

    Status swap_scene(const std::string& json, const std::string& label) {
        if (const auto st = runtime_.set_scene(json, label); !st.ok()) return st;
        (void)runtime_.with_cell([this](Cell&) {
            scene_.invalidate();  // the live world is a different model now
            return Status::success();
        });
        const auto family = family_;
        family_.clear();  // the world changed, so this is never "already live"
        if (const auto st = build(family); !st.ok()) return st;
        attach_kinematics();
        vision_.set_clutter(clutter_sites());
        camera_.set_view(vision_.target_site(), clutter_sites());
        vision_.rebuild();
        RN_LOG_INFO("scene '{}' is live: {}", label, runtime_.world());
        return Status::success();
    }

    Status build_define(const std::string& capability, const std::string& name,
                        const std::string& source, Select select, Command& c) {
        auto* cap = caps_.find(capability);
        if (cap == nullptr) return Status::failure("unknown capability '" + capability + "'");
        if (name.empty() || source.empty()) return Status::failure("need name+source");
        sandbox::Program program;
        if (const auto st = cap->compile(source, program); !st.ok()) return st;

        const std::string id = "user." + name;
        c.ack = {{"version", id}};
        c.run = [this, cap, id, program = std::move(program), select, source] {
            cap->install(id, program, select);
            note_source(cap, id, source);
            RN_LOG_INFO("{} module defined: {}", cap->descriptor().id, id);
            return Status::success();
        };
        return Status::success();
    }

    // The first family the descriptor declares is the one a cell boots into.
    [[nodiscard]] std::string default_family() const {
        const auto families = runtime_.families();
        return families.empty() ? std::string{} : families.front();
    }

    [[nodiscard]] std::string default_motion() const {
        const auto& motions = runtime_.descriptor().motions;
        return motions.empty() ? std::string{} : motions.front().id;
    }

    void note_source(CapabilityBase* cap, const std::string& id, const std::string& source) {
        if (auto* v = dynamic_cast<VisionCapability*>(cap)) v->remember_source(id, source);
        if (auto* t = dynamic_cast<TrajectoryCapability*>(cap)) t->remember_source(id, source);
        if (auto* k = dynamic_cast<TrackingCapability*>(cap)) k->remember_source(id, source);
    }

    void execute(Command& c, std::uint64_t id) {
        if (const auto st = c.starts_motion ? supervisor_.begin(CellState::kMoving)
                                            : Status::success();
            !st.ok()) {
            RN_LOG_WARN("#{} {} refused: {}", id, c.name, st.message());
            supervisor_.fault(st.message());
            publish_telemetry(0.0);
            return;
        }
        const auto started = std::chrono::steady_clock::now();
        const auto st = c.run ? c.run() : Status::success();
        // A command that moved the world (delivered a part, ran a belt, swapped
        // a scene) leaves the snapshot behind it; refresh it here so the next
        // step sees what it did.
        scene_.resample();
        const auto secs = std::chrono::duration<double>{std::chrono::steady_clock::now() - started};
        if (!st.ok()) {
            RN_LOG_WARN("#{} {} failed after {:.2f}s: {}", id, c.name, secs.count(), st.message());
            supervisor_.fault(st.message());
        } else {
            RN_LOG_INFO("#{} {} ok in {:.2f}s", id, c.name, secs.count());
            if (c.starts_motion) supervisor_.end();
        }
    }

    Status build(const std::string& family) {
        if (family == family_ && !arm_.slots.empty()) return Status::success();  // already live
        (void)supervisor_.begin(CellState::kBuilding);
        const auto st = runtime_.build(family);
        if (!st.ok()) {
            supervisor_.fault(st.message());
            return st;
        }
        control_.rebuild(runtime_.limits());
        arm_ = runtime_.joint_map();
        family_ = family;
        trajectory_.set_bounds(runtime_.joint_bounds());
        trajectory_.set_weights(arm_.weights);
        trajectory_.rebuild();
        supervisor_.end();
        publish_nodes();
        publish_telemetry(0.0);
        return Status::success();
    }

    Status select(const std::string& capability, const std::string& version) {
        auto* cap = caps_.find(capability);
        if (cap == nullptr) return Status::failure("unknown capability '" + capability + "'");
        const auto st = cap->select(version);
        // A detector holds its sensor, so changing the camera rebuilds the
        // detector around the new one.
        if (st.ok() && capability == "camera") {
            vision_.rebuild();
            std::lock_guard<std::mutex> lk{feed_mtx_};
            feed_.reset();
        }
        return st;
    }

    Status swap_node(const std::string& node, const std::string& driver) {
        const auto st = runtime_.replace_node(node, driver);
        RN_LOG_INFO("swap {} -> {}: {}", node, driver, st.ok() ? "ok" : st.message());
        publish_nodes();
        return st;
    }

    CycleHook stream_hook() {
        return [this, k = std::uint64_t{0}, decimation_ = std::max(1, cfg_.gateway.telemetry_decimation)](
                   double t, const std::vector<std::vector<TelemetryRow>>& rows) mutable {
            if (k++ % decimation_ != 0) return;
            // On the cycle thread, which owns the physics: take the copy here so
            // every reader elsewhere has one.
            scene_.sample_now();
            publish_io(t, rows);
        };
    }

    void publish_nodes() {
        std::vector<TelemetryPublisher::NodeView> views;
        for (const auto& v : runtime_.node_views()) {
            views.push_back({v.id, v.driver, v.unit, v.joint, v.lo, v.hi});
        }
        telemetry_.publish_nodes(runtime_.family(), runtime_.families(), registry_.names(), views);
    }

    void publish_io(double t, const std::vector<std::vector<TelemetryRow>>& rows) {
        TelemetryPublisher::Frame f;
        f.t = t;
        for (const auto& axis : rows) {
            const auto& row = axis.back();
            f.pos.push_back(row.actual_position);
            f.target.push_back(row.governed_position);
            f.err.push_back(row.following_error);
        }
        publish(f);
    }

    void publish_telemetry(double t) {
        TelemetryPublisher::Frame f;
        f.t = t;
        f.pos = runtime_.positions();
        f.target = f.pos;
        f.err.assign(f.pos.size(), 0.0);
        publish(f);
    }

    // Runs inside the worker's cell lock (the RT hook), so it must not touch
    // CellRuntime: everything it needs was cached at build time.
    void publish(TelemetryPublisher::Frame& f) {
        f.family = family_;
        f.tcp = tcp_of(f.pos);
        f.tcp_orientation = tcp_orientation(f.pos);
        if (f.tcp) workpiece_.track_tool(*f.tcp);
        // A sighting is timestamped with the frame's capture time, so the
        // tracker knows how old it is and can extrapolate from *then*.
        const auto sighting = vision_.look();
        f.part = sighting ? std::optional<Vec3>{sighting->position} : std::nullopt;
        // Where the part IS, from the sampled world — a held part is carried by
        // the weld, so this is also what proves a pick moved something.
        const auto held = scene_.site(vision_.target_site());
        workpiece_.observe(held.norm() > 0.0 ? std::optional<Vec3>{held} : f.part);
        f.contacts = scene_.contacts();
        report_contacts(f.contacts);
        if (sighting) tracking_.observe(sighting->captured_s, sighting->position);
        tracking_.publish_state();
        f.workpiece = workpiece_.pose();
        f.holding = gripper_.holding();
        f.progress = progress();
        telemetry_.publish(f);
    }

    void note_progress(std::string step, std::string app) {
        std::lock_guard<std::mutex> lk{progress_mtx_};
        step_ = std::move(step);
        running_app_ = std::move(app);
    }
    [[nodiscard]] std::string step() const {
        std::lock_guard<std::mutex> lk{progress_mtx_};
        return step_;
    }
    [[nodiscard]] std::string running_app() const {
        std::lock_guard<std::mutex> lk{progress_mtx_};
        return running_app_;
    }

    // Contacts are the cell's account of itself, so a change in what is touching
    // what is an event, not a per-frame field nobody reads. One line when it
    // changes is what turns "the move stopped short" into a diagnosis.
    void report_contacts(const std::vector<std::pair<std::string, std::string>>& now) {
        std::string signature;
        for (const auto& [a, b] : now) signature += a + "|" + b + " ";
        if (signature == contacts_seen_) return;
        contacts_seen_ = signature;
        RN_LOG_INFO("touching: {}", signature.empty() ? "nothing" : signature);
    }

    // The version of every capability at this instant — what a recorded run
    // must carry to be reproducible.
    std::map<std::string, std::string> active_versions() {
        std::map<std::string, std::string> out;
        for (const auto& id : caps_.ids()) {
            const auto j = nlohmann::json::parse(caps_.find(id)->json(), nullptr, false);
            if (!j.is_discarded()) out[id] = j.value("version", "");
        }
        return out;
    }

    TelemetryPublisher::Progress progress() const {
        std::lock_guard<std::mutex> lk{progress_mtx_};
        return {bus_ ? bus_->applied() : 0,
                bus_ ? bus_->accepted() : 0,
                supervisor_.state_name(),
                supervisor_.last_error(),
                supervisor_.latched(),
                supervisor_.working(),
                step_,
                running_app_};
    }

    // Where the tool ACTUALLY is, from the sampled world — the same world the
    // part is in. Forward kinematics runs on a scratch world (it must, to answer
    // "where would the tool be if…"), so using it for the real pose puts the
    // robot and the workpiece in two different universes.
    std::optional<Vec3> tcp_of(const std::vector<double>& cell_positions) {
        if (const auto site = scene_.site(tcp_site()); site.norm() > 0.0) return site;
        if (const auto pose = arm_pose(cell_positions)) return pose->position;
        return std::nullopt;
    }

    // Which way the tool POINTS, from forward kinematics on the arm's own
    // joints. A 6-DoF move is a claim about orientation, so a surface that
    // cannot see the current one cannot show whether it was met — or offer it
    // as the starting point for the next one.
    std::optional<Quat> tcp_orientation(const std::vector<double>& cell_positions) {
        if (const auto pose = arm_pose(cell_positions)) return pose->orientation;
        return std::nullopt;
    }

    std::optional<Pose> arm_pose(const std::vector<double>& cell_positions) {
        const auto chain = kinematics();
        if (!chain) return std::nullopt;
        std::vector<double> q;
        q.reserve(arm_.slots.size());
        for (std::size_t k = 0; k < arm_.slots.size(); ++k) {
            const auto slot = arm_.slots[k];
            if (slot < cell_positions.size()) q.push_back(cell_positions[slot] / arm_.per_m[k]);
        }
        if (q.size() != chain->dof()) return std::nullopt;
        return chain->tcp_pose(q);
    }

    std::string id_;
    Settings cfg_;
    DriverRegistry registry_;
    CellRuntime runtime_;
    mutable std::mutex kin_mtx_;
    std::shared_ptr<Kinematics> kin_;

    CameraCapability camera_;
    VisionCapability vision_;
    TrackingCapability tracking_;
    TrajectoryCapability trajectory_;
    ControlCapability control_;
    CapabilityRegistry caps_;

    std::shared_ptr<MujocoWorldPool> pool_;
    SceneView scene_{make_scene_view()};
    std::chrono::steady_clock::time_point born_{std::chrono::steady_clock::now()};
    Workpiece workpiece_;
    std::string contacts_seen_;
    MotionService motion_;
    SimGripper gripper_;
    ProgramRunner program_;
    CellSupervisor supervisor_;
    StationOps stations_;
    RunRecorder recorder_;
    TelemetryPublisher telemetry_;

    CellRuntime::JointMap arm_;  // cached: the telemetry path takes no lock
    std::string family_{"physics"};
    // What the program is doing: written by the worker, read by every surface
    // that publishes telemetry. Two plain strings, and therefore invisible to
    // the "who owns this container?" sweep that checked maps and engine
    // handles — ThreadSanitizer found them in one run.
    mutable std::mutex progress_mtx_;
    std::string step_;         // the program step in flight, published with telemetry
    std::string running_app_;  // which application that step belongs to
    std::mutex quality_mtx_;
    MotionService::RunQuality quality_;
    std::uint64_t quality_run_{0};
    std::mutex feed_mtx_;
    std::unique_ptr<Camera> feed_;  // the dashboard's own view of the sensor

    std::atomic<bool> stop_idle_{false};
    std::thread idle_;

    CommandRouter router_;
    std::optional<CommandBus<Command>> bus_;  // last: the worker joins before what it drives
};

}  // namespace robonode
