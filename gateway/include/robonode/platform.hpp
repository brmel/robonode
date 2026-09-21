#pragma once

#include <cctype>
#include <filesystem>
#include <fstream>
#include <map>
#include <chrono>
#include <string>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>

#include "robonode/celld/app_descriptor.hpp"
#include "robonode/celld/json_doc_store.hpp"
#include "robonode/celld/scene_descriptor.hpp"
#include "robonode/celld/settings_io.hpp"
#include "robonode/core/status.hpp"
#include "robonode/gateway/progress.hpp"
#include "robonode/gateway/cell_manager.hpp"
#include "robonode/gateway/workspace.hpp"

namespace robonode {

// The one clean entry (#33): the MIL-style facade the web UI, the CLI and the
// SDK all bind to. Typed verbs over the cell; callers never hand-craft command
// JSON and no gateway internal leaks. Commands are asynchronous and identified:
// every action returns the id the telemetry stream reports as `applied_id`.
class Platform {
public:
    Platform(std::string worlds_dir, const std::string& cell_path, std::string apps_dir = "",
             std::string modules_dir = "", std::string scenes_dir = "",
             std::string sessions_dir = "", Settings settings = {},
             CellGateway::DriverHook vendor_drivers = {},
             CellGateway::VisionHook vision_versions = {}, std::string robots_dir = "")
        : settings_{settings}, worlds_{worlds_dir}, cell_path_{cell_path},
          cells_{std::move(worlds_dir), cell_path, settings, std::move(vendor_drivers),
                 std::move(vision_versions), scenes_dir},
          library_{shipped_library({{"apps", apps_dir},
                                    {"modules", modules_dir},
                                    {"scenes", scenes_dir},
                                    {"robots", robots_dir}})},
          sessions_{std::move(sessions_dir)} {
        reload_modules();
    }

    // Every document kind the platform serves, from the one table that declares
    // them. A surface asks rather than carrying its own list — which is what
    // makes "add a kind" one row instead of a search for every place that
    // spelled the three of them out.
    [[nodiscard]] static std::vector<std::string> kinds() {
        std::vector<std::string> out;
        for (const auto& [kind, _] : Workspace::kKinds) out.emplace_back(kind);
        return out;
    }

    Status run() { return apply("run"); }
    Status run_motion(const std::string& id) { return apply("run", {{"motion", id}}); }
    Status stop() { return apply("stop"); }
    Status estop() { return apply("estop"); }
    Status resume() { return apply("resume"); }
    Status grasp() { return apply("grasp"); }
    Status release() { return apply("release"); }
    Status set_family(const std::string& family) { return apply("driver", {{"family", family}}); }
    Status set_node_driver(const std::string& node, const std::string& driver) {
        return apply("set_driver", {{"node", node}, {"driver", driver}});
    }
    Status set_version(const std::string& capability, const std::string& version,
                       const std::string& cell_id = {}) {
        nlohmann::json args{{"capability", capability}, {"version", version}};
        if (!cell_id.empty()) args["cell"] = cell_id;
        return apply("set_version", args);
    }
    Status jog(const std::string& axis, double target) {
        return apply("jog", {{"axis", axis}, {"target", target}});
    }
    Status move_l(double x, double y, double z) {
        return apply("move_l", {{"x", x}, {"y", y}, {"z", z}});
    }
    // The point AND the angle: a tool that must arrive square to a face says so
    // here instead of hoping the wrist lands well.
    Status move_pose(double x, double y, double z, double qw, double qx, double qy, double qz) {
        return apply("move_pose", {{"x", x}, {"y", y}, {"z", z},
                                   {"qw", qw}, {"qx", qx}, {"qy", qy}, {"qz", qz}});
    }

    // Task steps as single verbs: the same code an application runs, reachable
    // one at a time, so debugging a cell never requires authoring an app.
    Status pick() { return apply("pick"); }
    Status place(const std::string& station) { return apply("place", {{"station", station}}); }
    Status intercept() { return apply("intercept"); }
    Status deliver(const std::string& station) { return apply("deliver", {{"station", station}}); }
    Status run_conveyor(const std::string& station, Belt belt) {
        return apply("conveyor",
                     {{"station", station}, {"run", belt == Belt::kRunning ? "true" : "false"}});
    }

    Status define_module(const std::string& capability, const std::string& name,
                         const std::string& source) {
        std::string version;
        return define_and_persist(capability, name, source, version);
    }

    // Deploy a saved app: parse it once, run the typed program on the worker.
    Status run_app(const std::string& file, const std::string& session = {},
                   const std::string& cell_id = {}) {
        const auto body = doc("apps", file, session);
        if (!body) return body.error();
        AppDescriptor app;
        if (const auto st = parse_app_descriptor(nlohmann::json::parse(*body, nullptr, false), app,
                                                 file);
            !st.ok()) {
            return st;
        }
        const auto on = cell(cell_id);
        if (!on) return Status::failure("no cell '" + cell_id + "'");
        const auto ack = on->run_program(std::move(app.program), file);
        last_ack_ = ack;
        return ack.accepted ? Status::success() : Status::failure(ack.reason);
    }

    // Run a scene: resolve it in the caller's session (their own shadows the
    // shipped one), then hand the descriptor to the cell. Switching scenario is
    // a command, so it queues behind whatever is moving and reports when live.
    Status apply_scene(const std::string& file, const std::string& session = {}) {
        const auto body = doc("scenes", file, session);
        if (!body) return body.error();
        return apply("load_scene", {{"scene", *body}, {"label", file}});
    }

    // Run the work that exercises a capability once per version, and report
    // what happened. THIS is the comparison — the web panel and the CLI both
    // call it, so "A vs B" cannot mean two different things on two surfaces
    // (ADR-8). The trial comes from the capability's own contract, so nothing
    // here decides what the work is either.
    //
    // A version that fails its trial is a result, not an error: that is what
    // "the naive algorithm misses" looks like in a table.
    Status compare(const std::string& capability, const std::vector<std::string>& versions,
                   std::string& out, const std::string& session = {},
                   const std::string& cell_id = {}) {
        const auto trial = trial_for(capability, cell_id);
        if (!trial) return trial.error();

        nlohmann::json results = nlohmann::json::array();
        for (const auto& version : versions) {
            results.push_back(judge(capability, version, trial->app, session, cell_id));
        }
        if (!trial->live_version.empty()) {
            (void)set_version(capability, trial->live_version, cell_id);
            (void)await_settled({}, cell_id);
        }

        auto report = nlohmann::json{{"name", capability + ": " + short_name(versions)},
                                     {"capability", capability},
                                     {"trial", trial->app},
                                     {"results", std::move(results)}};
        // A comparison you cannot look at again is an anecdote. It is a document
        // like any other — listed, read and deleted through the same store, in
        // the session that ran it.
        (void)save_doc("runs", record_name(capability), report.dump(2), session);
        out = report.dump();
        return Status::success();
    }

    // The acknowledgement of the last typed verb — the id a caller watches for.
    [[nodiscard]] const CommandAck& last_ack() const { return last_ack_; }

    // Block until the worker has drained every accepted command and the cell is
    // idle. One predicate for every verb — no surface re-derives "done" from the
    // shape of the motion it asked for.
    //
    // The wait is bounded by STALL, not by a guess at how long the work takes: a
    // cell that is still publishing progress has not failed, however slow the
    // machine is. Only silence is a timeout.
    Status await_settled(std::chrono::milliseconds stall_timeout = std::chrono::milliseconds{0},
                         const std::string& cell_id = {}) {
        const auto on = cell(cell_id);
        if (!on) return Status::failure("no cell '" + cell_id + "'");
        const auto limit = stall_timeout.count() > 0
                               ? stall_timeout
                               : std::chrono::milliseconds{settings_.gateway.stall_timeout_ms};
        std::string last_token;
        auto last_change = std::chrono::steady_clock::now();
        auto seen = on->telemetry_sequence();
        while (std::chrono::steady_clock::now() - last_change < limit) {
            const auto now = progress_of(
                nlohmann::json::parse(telemetry_json(cell_id), nullptr, false));
            if (now.settled) {
                return now.error.empty() ? Status::success() : Status::failure(now.error);
            }
            if (!now.token.empty() && now.token != last_token) {
                last_token = now.token;
                last_change = std::chrono::steady_clock::now();
            }
            // Wake on the next publication, not on a timer: the wait is as
            // responsive as the cell and costs nothing while it is quiet. The
            // bound is still STALL — a cell that reports nothing at all is the
            // only failure, however slow the machine is.
            seen = on->await_telemetry(seen, std::chrono::milliseconds{100});
        }
        return Status::failure("the cell stopped reporting progress");
    }

    //
    // Every live view of one robot, under the name every surface calls it.
    // This table is the only place a view exists: the HTTP routes, the CLI's
    // readers and the typed accessors below are all built from it, so a view
    // that one surface has and another lacks is not expressible (ADR-8).
    // Adding a view is a row.
    struct View {
        const char* name;
        std::string (CellGateway::*read)();
        const char* help;  // the CLI's description: it belongs with the view
    };
    static constexpr View kViews[]{
        {"telemetry", &CellGateway::telemetry_json, "print the live I/O snapshot"},
        {"nodes", &CellGateway::nodes_json, "print the node tree"},
        {"logs", &CellGateway::logs_json, "print recent log records"},
        {"stations", &CellGateway::stations_json, "print the cell's stations"},
        {"model", &CellGateway::model_json, "print the kinematic chain from the model"},
        {"capabilities", &CellGateway::capabilities_json, "print every swappable capability"},
        {"verbs", &CellGateway::verbs_json,
         "print the steps an application program may use"},
        {"last-run", &CellGateway::last_run_json,
         "how the last motion went: cycles, overruns, jitter, worst-tracking axis, and "
         "whether it met the budget"}};

    // A view by name. A robot that is not there is refused, never answered
    // about a different one — the rule that used to be repeated per route.
    Result<std::string> view_json(const std::string& name, const std::string& cell_id = {}) {
        const auto on = cell(cell_id);
        if (!on) return no("no cell '" + cell_id + "'");
        for (const auto& v : kViews) {
            if (name == v.name) return ((*on).*v.read)();
        }
        return no("no view '" + name + "'");
    }

    std::string nodes_json(const std::string& cell_id = {}) { return view_or_empty("nodes", cell_id); }
    std::string telemetry_json(const std::string& cell_id = {}) {
        return view_or_empty("telemetry", cell_id);
    }
    std::string logs_json(const std::string& cell_id = {}) { return view_or_empty("logs", cell_id); }
    std::string stations_json(const std::string& cell_id = {}) {
        return view_or_empty("stations", cell_id);
    }
    std::string model_json(const std::string& cell_id = {}) { return view_or_empty("model", cell_id); }
    std::string cells_json() { return cells_.cells_json(); }

    // Multi-cell: a second robot is one call, reusing this platform's world and
    // descriptor unless the caller names others.
    // A second cell may run a different scenario entirely: its descriptor names
    // its own scene, so only the descriptor has to change.
    // Start another robot from the CATALOGUE: `robot` names a cell descriptor
    // the store resolves (a shipped one, or the user's own). A cell whose
    // descriptor declares a different chain simply has different nodes — which
    // is the whole claim that a robot is data.
    Status add_cell(const std::string& id, const std::string& robot = {},
                    const std::string& session = {}) {
        if (robot.empty()) return cells_.add(id, worlds_, cell_path_);
        const auto body = doc("robots", robot, session);
        if (!body) return body.error();
        // The manager loads descriptors by path; write the resolved document
        // where it can be read, so a session's own robot boots like any other.
        const auto path = std::filesystem::path{worlds_} / (".robot-" + id + ".cell.json");
        std::ofstream out{path, std::ios::trunc};
        if (!out) return Status::failure("cannot stage robot descriptor: " + path.string());
        out << *body;
        out.close();
        return cells_.add(id, worlds_, path.string());
    }
    Status remove_cell(const std::string& id) { return cells_.remove(id); }
    std::string capabilities_json(const std::string& cell_id = {}) {
        return view_or_empty("capabilities", cell_id);
    }
    std::string verbs_json(const std::string& cell_id = {}) { return view_or_empty("verbs", cell_id); }
    std::string camera_bmp(const std::string& cell_id = {}) {
        const auto on = cell(cell_id);
        return on ? on->camera_bmp() : std::string{};
    }
    std::string last_run_json(const std::string& cell_id = {}) {
        return view_or_empty("last-run", cell_id);
    }
    // Reading a capability that does not exist is a caller error, not an empty
    // answer: every surface should say so rather than render nothing.
    Status capability_json(const std::string& id, std::string& out,
                           const std::string& cell_id = {}) {
        const auto on = cell(cell_id);
        if (!on) return Status::failure("no cell '" + cell_id + "'");
        if (!on->has_capability(id)) return Status::failure("unknown capability '" + id + "'");
        out = on->capability_json(id);
        return Status::success();
    }

    // Everyone works in their own space, on the same platform.
    std::string sessions_json() { return sessions_.list_json(); }
    Status delete_session(const std::string& id) { return sessions_.remove(id); }

    // Scenes, applications and algorithms are the same thing with different
    // contents: a library that ships read-only, a session's own copies on top,
    // and one rule that decides whether a document may be stored at all. So
    // they get ONE surface — a fourth kind adds a row, not five more methods.
    [[nodiscard]] bool has_cell(const std::string& id) const {
        return id.empty() || cells_.has(id);
    }

    [[nodiscard]] bool has_kind(const std::string& kind) const { return library_.contains(kind); }

    // The library a session sees: what ships with the platform, plus what that
    // session authored. A user's own document shadows the shipped one, so
    // "start from an existing scene" is: open it, change it, save it.
    std::string docs_json(const std::string& kind, const std::string& session = {}) {
        return merged_json(shipped(kind), mine(kind, session));
    }

    // A session's own document shadows the shipped one, so "start from an
    // existing scene" is: open it, change it, save it.
    [[nodiscard]] Result<std::string> doc(const std::string& kind, const std::string& file,
                                          const std::string& session = {}) {
        if (auto ours = mine(kind, session).read(file)) return ours;
        return shipped(kind).read(file);
    }

    // Writes always land in the session: the shipped library is read-only, so a
    // user can never break the scenario everyone else started from. What cannot
    // be loaded back is never stored — the editor learns its document is wrong
    // when it saves, not when someone deploys it an hour later.
    Status save_doc(const std::string& kind, const std::string& file, const std::string& json,
                    const std::string& session = {}) {
        if (const auto st = validate(kind, json, file); !st.ok()) return st;
        return mine(kind, session).save(file, json);
    }

    Status delete_doc(const std::string& kind, const std::string& file,
                      const std::string& session = {}) {
        return mine(kind, session).remove(file);
    }

    // Fork to start from something that works — the normal way to make a
    // scenario, an application or an algorithm your own.
    Status fork_doc(const std::string& kind, const std::string& from, const std::string& to,
                    const std::string& name, const std::string& session = {}) {
        const auto body = doc(kind, from, session);
        if (!body) return body.error();
        auto j = nlohmann::json::parse(*body, nullptr, false);
        if (j.is_discarded()) return Status::failure("cannot read " + kind + ": " + from);
        if (!name.empty()) j["name"] = name;
        return save_doc(kind, to, j.dump(2), session);
    }

    // Transport escape hatch: HTTP forwards raw bodies here so the wire contract
    // lives in one place. The two verbs that touch a store are resolved first,
    // through the same code path the CLI uses.
    std::string submit_command(const std::string& body) {
        const auto j = nlohmann::json::parse(body, nullptr, false);
        if (j.is_discarded()) return reject("bad command");
        const std::string cmd = j.value("cmd", "");
        // A command may name the robot it is for. Without it, the platform's
        // first cell — so a second machine is reachable without every existing
        // caller learning a new field.
        const std::string on = j.value("cell", "");
        if (!on.empty() && !cells_.has(on)) return reject("no cell '" + on + "'");
        if (cmd == "run_app" && j.contains("file")) {
            const auto st = run_app(j.at("file").get<std::string>(), j.value("session", ""), on);
            return st.ok() ? ack_json(last_ack_) : reject(st.message());
        }
        if (cmd == "load_scene" && j.contains("file")) {
            const auto file = j.at("file").get<std::string>();
            const auto scene = doc("scenes", file, j.value("session", ""));
            if (!scene) return reject(scene.error().message());
            const auto target = cell(on);
            if (!target) return reject("no cell '" + on + "'");
            return target->submit("load_scene", {{"scene", *scene}, {"label", file}});
        }
        if (cmd == "define_module") {
            std::string version;
            const auto st = define_and_persist(j.value("capability", ""), j.value("name", ""),
                                               j.value("source", ""), version);
            if (!st.ok()) return reject(st.message());
            return ack_json(last_ack_, {{"version", version}});
        }
        const auto target = cell(on);
        return target ? target->submit_command(body) : reject("no cell '" + on + "'");
    }

private:
    // What a capability is judged ON, and what it was running before we started.
    struct Trial {
        std::string app;
        std::string live_version;
    };

    Result<Trial> trial_for(const std::string& capability, const std::string& cell_id) {
        std::string view;
        if (const auto st = capability_json(capability, view, cell_id); !st.ok()) return no(st);
        const auto cap = nlohmann::json::parse(view, nullptr, false);
        const std::string app = cap.is_object() ? cap.value("trial", "") : "";
        if (app.empty()) return no("capability '" + capability + "' names no trial");
        return Trial{app, cap.value("version", "")};
    }

    // One version, one trial, one row. Every version starts from the same cell:
    // without that the second one begins where the first finished — for
    // pick-demo, already at the goal — so its trial moves nothing, records
    // nothing, and a comparison in which only one algorithm ran is worse than
    // no comparison at all.
    nlohmann::json judge(const std::string& capability, const std::string& version,
                         const std::string& app, const std::string& session,
                         const std::string& cell_id) {
        if (const auto st = set_version(capability, version, cell_id); !st.ok()) {
            return {{"version", version}, {"ok", false}, {"error", st.message()}};
        }
        // Put the cell back the way the previous version found it. Homing was
        // not enough: a trial that picks something leaves the gripper closed
        // around it, so the NEXT version started already holding a part and
        // failed with "already holding a part" — judged on the mess its
        // predecessor left rather than on its own work.
        (void)apply("release", {{"cell", cell_id}});
        (void)await_settled({}, cell_id);
        (void)apply("run", {{"cell", cell_id}});
        (void)await_settled({}, cell_id);

        const auto before = run_counter(cell_id);
        const auto started = std::chrono::steady_clock::now();
        auto st = run_app(app, session, cell_id);
        if (st.ok()) st = await_settled({}, cell_id);
        const std::chrono::duration<double> took = std::chrono::steady_clock::now() - started;

        nlohmann::json row{{"version", version}, {"ok", st.ok()}, {"seconds", took.count()}};
        if (!st.ok()) row["error"] = st.message();
        // How well it ran, not just whether it did — but only if THIS trial
        // moved. A trial that failed before any motion leaves the previous
        // version's record standing, and attaching it here reports one
        // algorithm's numbers under another's name.
        if (const auto q = nlohmann::json::parse(last_run_json(cell_id), nullptr, false);
            q.is_object() && q.value("run", std::uint64_t{0}) > before) {
            row["quality"] = q;
        }
        return row;
    }

    // How many motions this cell has recorded. A comparison uses it to tell a
    // record made BY a trial from one that merely survived it.
    std::uint64_t run_counter(const std::string& cell_id) {
        const auto q = nlohmann::json::parse(last_run_json(cell_id), nullptr, false);
        return q.is_object() ? q.value("run", std::uint64_t{0}) : 0;
    }

    // Shared, not a reference: the cell a request is using stays alive even if
    // another request drops it mid-call (see CellManager::cell).
    // Which robot a call is for. Empty means "main", so every existing caller
    // keeps working and a second robot is one argument away — the same shape
    // the session parameter already has.
    // The typed accessors answer "" for a robot that is not there, which many
    // callers rely on; the surfaces use view_json and refuse instead.
    std::string view_or_empty(const std::string& name, const std::string& cell_id) {
        auto body = view_json(name, cell_id);
        return body ? std::move(*body) : std::string{};
    }

    std::shared_ptr<CellGateway> cell(const std::string& id = {}) {
        return cells_.cell(id.empty() ? "main" : id);
    }

    // The read-only library each kind ships with. A kind with nowhere to ship
    // from (a session's own run records) still gets a store — an empty one —
    // so every kind is reachable through exactly the same path.
    static std::map<std::string, JsonDocStore> shipped_library(
        const std::map<std::string, std::string>& dirs) {
        std::map<std::string, JsonDocStore> out;
        for (const auto& [kind, suffix] : Workspace::kKinds) {
            const auto dir = dirs.find(kind);
            out.emplace(kind, JsonDocStore{dir == dirs.end() ? std::string{} : dir->second, suffix});
        }
        return out;
    }

    static std::string short_name(const std::vector<std::string>& versions) {
        std::string out;
        for (const auto& v : versions) {
            const auto dot = v.rfind('.');
            out += (out.empty() ? "" : " vs ") + (dot == std::string::npos ? v : v.substr(dot + 1));
        }
        return out;
    }

    // One record per capability: the last comparison is the one worth keeping,
    // and a directory that grows without bound is a leak with a nice name.
    static std::string record_name(const std::string& capability) {
        return capability + ".run.json";
    }

    JsonDocStore& shipped(const std::string& kind) { return library_.at(kind); }
    JsonDocStore& mine(const std::string& kind, const std::string& session) {
        return sessions_.open(session).docs(kind);
    }

    // What makes a document of this kind storable. A kind with no rule stores
    // anything a client sends — which is right for source it never interprets.
    Status validate(const std::string& kind, const std::string& json, const std::string& file) {
        const auto j = nlohmann::json::parse(json, nullptr, false);
        if (kind == "scenes") {
            SceneDescriptor scene;
            return parse_scene_descriptor(j, scene, file);
        }
        if (kind == "apps") {
            AppDescriptor app;
            if (const auto st = parse_app_descriptor(j, app, file); !st.ok()) return st;
            return cell()->check_program(app.program);
        }
        if (kind == "robots") {
            CellDescriptor robot;
            return parse_cell_descriptor(j, robot, file);
        }
        return j.is_discarded() ? Status::failure("invalid JSON: " + file) : Status::success();
    }

    static std::string reject(const std::string& why) {
        return nlohmann::json{{"ok", false}, {"error", why}}.dump();
    }
    Status apply(const std::string& verb, nlohmann::json args = nlohmann::json::object()) {
        const std::string on = args.value("cell", "");
        if (!on.empty()) args.erase("cell");
        const auto target = cell(on);
        if (!target) return Status::failure("no cell '" + on + "'");
        const auto r = nlohmann::json::parse(target->submit(verb, args), nullptr, false);
        if (!r.is_discarded() && r.value("ok", false)) return Status::success();
        return Status::failure(r.is_discarded() ? "bad response"
                                                : r.value("error", "command rejected"));
    }

    static std::string sanitize(const std::string& name) {
        std::string out;
        for (const char ch : name) {
            out.push_back(std::isalnum(static_cast<unsigned char>(ch)) || ch == '-' || ch == '_'
                              ? ch
                              : '-');
        }
        return out.empty() ? "algo" : out;
    }

    Status define_and_persist(const std::string& capability, const std::string& name,
                              const std::string& source, std::string& version_out) {
        if (name.empty() || source.empty()) return Status::failure("need name+source");
        const std::string clean = sanitize(name);
        if (const auto st =
                cell()->define_module(capability, clean, source, Select::kNow, version_out, &last_ack_);
            !st.ok()) {
            return st;
        }
        return shipped("modules").save(clean + ".module.json",
                             nlohmann::json{{"name", clean},
                                            {"capability", capability},
                                            {"source", source}}
                                 .dump());
    }

    void reload_modules() {
        for (const auto& e : shipped("modules").list()) {
            std::string body;
            if (!shipped("modules").load(e.file, body).ok()) continue;
            const auto m = nlohmann::json::parse(body, nullptr, false);
            if (m.is_discarded() || !m.contains("capability") || !m.contains("source")) continue;
            std::string version;
            (void)cell()->define_module(m.at("capability").get<std::string>(),
                                       m.value("name", e.file),
                                       m.at("source").get<std::string>(), Select::kLater, version);
        }
    }

    Settings settings_;
    CommandAck last_ack_{};
    std::string worlds_, cell_path_;
    CellManager cells_;
    std::map<std::string, JsonDocStore> library_;
    Sessions sessions_;

    static std::string merged_json(const JsonDocStore& shipped, const JsonDocStore& mine) {
        std::map<std::string, nlohmann::json> by_file;
        for (const auto& e : shipped.list()) {
            by_file[e.file] = {{"name", e.name}, {"file", e.file}, {"origin", "library"}};
        }
        for (const auto& e : mine.list()) {
            by_file[e.file] = {{"name", e.name}, {"file", e.file}, {"origin", "session"}};
        }
        auto arr = nlohmann::json::array();
        for (const auto& [_, doc] : by_file) arr.push_back(doc);
        return arr.dump();
    }
};

}  // namespace robonode
