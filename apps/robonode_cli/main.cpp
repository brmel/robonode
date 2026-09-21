// robonode — drive the cell headless through the same facade the web UI binds
// to (ADR-8). Every verb maps 1:1 to robonode::Platform, so an agent does
// everything a human does in the browser, with machine-readable JSON output.
//
//   robonode nodes | telemetry | logs | apps | modules | stations
//   robonode robots · cells · add-cell <id> --robot <file> · drop-cell <id>
//   robonode capabilities                   # every swappable capability
//   robonode capability <id>                # one capability + its versions
//   robonode run [motion]                   # run a named coordinated motion
//   robonode movel <x> <y> <z>              # Cartesian move to a TCP target
//   robonode jog <axis> <target>            # jerk-limited single-axis jog (OTG)
//   robonode stop | estop | resume          # cancel on path / latch / clear
//   robonode grasp | release                # end effector
//   robonode pick | intercept               # one task step, on its own
//   robonode place <station>                # place at a station's next slot
//   robonode deliver <station>              # put a fresh workpiece on the line
//   robonode conveyor <station> [--off]     # run/stop a belt
//   robonode contacts                       # what is touching what, right now
//   robonode watch [--for <s>]              # follow telemetry as it changes
//   robonode swap <node> <driver>           # swap one axis driver, live
//   robonode family <physics|sim>           # rebuild under a driver family
//   robonode version <cap> <version>        # swap a capability algorithm
//   robonode define <cap> <name> <source>   # author a sandboxed version
//   robonode app <file>                     # deploy + run an application
//   robonode scenes | scene <file>          # the scene library
//   robonode use-scene <file>               # run this scenario, live
//   robonode movep <x y z qw qx qy qz>      # reach a pose: the point and the angle
//   robonode fork <kind> <from> <to>        # start a document of your own
//   robonode compare <cap> <a> <b>          # two versions, on the work that exercises them
//   robonode runs · run-record <file>       # what those comparisons found, kept
//   robonode sessions                       # who has a workspace here
//   --session <id>                          # work in your own session
//   --server <url>                          # drive a RUNNING cell_server
//
// Commands are asynchronous; every action waits on the one completion predicate
// the whole platform shares. With --server the verbs go to a running cell over
// the same HTTP contract the web app uses, so an agent debugging what is on
// screen is looking at THAT cell, not a private copy of it.

#include <chrono>
#include <deque>
#include <cstdio>
#include <memory>
#include <sstream>
#include <thread>
#include <cstdlib>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include "cell_client.hpp"
#include "robonode/platform.hpp"
#include "robonode/apps/vendors.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must point at the sim-mujoco worlds dir"
#endif
#ifndef ROBONODE_CELL
#error "ROBONODE_CELL must point at the cell descriptor JSON"
#endif

namespace {

int fail(const robonode::Status& st) {
    std::fprintf(stderr, "error: %s\n", st.message().c_str());
    return 1;
}

void print(const std::string& json) { std::printf("%s\n", json.c_str()); }

// `--quiet state,contacts`: what a watcher considers a change.
std::vector<std::string> split_csv(const std::string& csv) {
    std::vector<std::string> out;
    std::string field;
    std::istringstream in{csv};
    while (std::getline(in, field, ',')) {
        if (!field.empty()) out.push_back(field);
    }
    return out;
}

std::string project(const nlohmann::json& frame, const std::vector<std::string>& fields) {
    nlohmann::json picked = nlohmann::json::object();
    for (const auto& f : fields) {
        if (frame.contains(f)) picked[f] = frame.at(f);
    }
    return picked.dump();
}

// Read a view, print it, or say why not.
int show(robonode::cli::CellClient& cell, const std::string& what) {
    std::string body;
    if (const auto st = cell.read(what, body); !st.ok()) return fail(st);
    print(body);
    return 0;
}

// Submit, wait for the cell to settle, print the requested view. There is no
// per-verb timeout: the cell reports progress, and only silence is a failure.
int act(robonode::cli::CellClient& cell, const nlohmann::json& command,
        const std::string& view = "telemetry") {
    if (const auto st = cell.command(command); !st.ok()) return fail(st);
    const auto settled = cell.settled();
    (void)show(cell, view);
    return settled.ok() ? 0 : fail(settled);
}

}  // namespace

int main(int argc, char** argv) {
    CLI::App app{"robonode — headless cell control (same facade as the UI)"};
    app.require_subcommand(1);
    app.fallthrough();  // --session reads the same either side of the verb

    // Every read verb is generated: the live views from the platform's table,
    // the document collections from its kinds. A view the server publishes and
    // the CLI lacks is the ADR-8 bug, and it is no longer expressible — adding
    // one is a row in the table, in one place, for every surface.
    std::vector<std::pair<CLI::App*, std::string>> reads;
    const auto read_verb = [&](const std::string& name, const std::string& help) {
        reads.emplace_back(app.add_subcommand(name, help), name);
    };
    for (const auto& v : robonode::Platform::kViews) read_verb(v.name, v.help);
    for (const auto& kind : robonode::Platform::kinds()) {
        read_verb(kind, "list the " + kind + " library — what ships, plus your own");
    }
    read_verb("cells", "list the running cells");
    read_verb("sessions", "list the sessions on this platform");

    // Reading ONE document out of a collection: the verb names the collection,
    // the argument names the document. It was the same four lines four times,
    // and the fifth would have been the one that read the wrong collection.
    struct ReadOne {
        CLI::App* sub;
        std::string collection, id;
    };
    std::deque<ReadOne> one_of;  // deque: CLI11 holds a reference into id
    const auto read_one = [&](const std::string& verb, std::string collection, const char* help,
                              const char* arg_help) {
        auto& entry = one_of.emplace_back(nullptr, std::move(collection), std::string{});
        entry.sub = app.add_subcommand(verb, help);
        entry.sub->add_option("id", entry.id, arg_help)->required();
    };
    read_one("run-record", "runs", "print one recorded comparison",
             "record file (e.g. control.run.json)");
    read_one("capability", "capabilities", "print one capability", "vision | planner | control");
    read_one("scene", "scenes", "print one scene", "scene file");
    auto* c_logs = app.get_subcommand("logs");

    std::string run_file;

    // Everything you author is scoped to a session; the shipped library shows
    // through underneath it.
    std::string session, server, on_cell;
    app.add_option("--session", session, "work in this session (default: shared)");
    app.add_option("--cell", on_cell,
                   "drive this robot (see `robonode cells`; default: the first)");
    app.add_option("--server", server,
                   "drive a running cell_server (e.g. http://localhost:8080) instead of "
                   "booting a cell in this process");

    // One fork for every kind of document — a scene, an application, an
    // algorithm. What differs is the word, not the operation.
    std::string fork_kind, fork_from, fork_to, fork_name;
    auto* c_fork = app.add_subcommand("fork", "start a document of your own from an existing one");
    c_fork->add_option("kind", fork_kind, "scenes | apps | modules")->required();
    c_fork->add_option("from", fork_from, "document to start from")->required();
    c_fork->add_option("to", fork_to, "your document's filename")->required();
    c_fork->add_option("--name", fork_name, "display name");
    // A command verb is a row: what the user types, what goes on the wire, its
    // arguments in the order they are typed, and the view worth printing once it
    // lands ({key} interpolates a text argument). Adding a verb is a row — there
    // is no declaration and no dispatch line to remember separately, so the two
    // cannot disagree.
    //
    // An argument says whether it is text or a number, because the wire cares:
    // `move_l` wants x as 0.4, not "0.4". That is the whole reason the numeric
    // verbs used to sit outside this table by hand.
    enum class Arg { text, number };
    struct VerbArg {
        std::string name;
        Arg kind;
    };
    struct CommandVerb {
        std::string cmd, refresh;
        std::vector<VerbArg> args;
        std::deque<std::string> texts;  // deque: CLI11 holds references into these
        std::deque<double> numbers;
        CLI::App* sub{};
    };
    std::deque<CommandVerb> commands;
    const auto command_verb = [&app, &commands](const std::string& verb, std::string cmd,
                                                const char* help,
                                                std::vector<VerbArg> args = {},
                                                std::string refresh = {}) {
        auto& cv = commands.emplace_back();
        cv.cmd = std::move(cmd);
        cv.refresh = std::move(refresh);
        cv.args = std::move(args);
        cv.sub = app.add_subcommand(verb, help);
        for (const auto& arg : cv.args) {
            auto* option = arg.kind == Arg::number
                               ? cv.sub->add_option(arg.name, cv.numbers.emplace_back(0.0))
                               : cv.sub->add_option(arg.name, cv.texts.emplace_back());
            option->required();
        }
    };
    const auto text = [](std::string name) { return VerbArg{std::move(name), Arg::text}; };
    const auto number = [](std::string name) { return VerbArg{std::move(name), Arg::number}; };
    command_verb("stop", "stop", "cancel the running motion (decelerate on path)");
    command_verb("estop", "estop", "latch a software stop");
    command_verb("resume", "resume", "clear a latched stop");
    command_verb("grasp", "grasp", "close the end effector");
    command_verb("release", "release", "open the end effector");
    command_verb("pick", "pick", "pick the part vision currently sees");
    command_verb("intercept", "intercept", "pick a MOVING part (tracking + lead)");
    command_verb("place", "place", "place at a station's next slot", {text("station")});
    command_verb("deliver", "deliver", "put a fresh workpiece at the head of a line",
                 {text("station")});
    command_verb("app", "run_app", "run an application", {text("file")});
    command_verb("use-scene", "load_scene", "run this scenario on the live cell", {text("file")},
                 "model");
    command_verb("swap", "set_driver", "swap one node's driver version",
                 {text("node"), text("driver")}, "nodes");
    command_verb("family", "driver", "rebuild the cell under a driver family", {text("family")},
                 "nodes");
    command_verb("version", "set_version", "swap a capability's algorithm version",
                 {text("capability"), text("version")}, "capabilities/{capability}");
    command_verb("define", "define_module", "author a sandboxed capability version",
                 {text("capability"), text("name"), text("source")}, "capabilities/{capability}");
    command_verb("jog", "jog", "jerk-limited jog of one axis (OTG)",
                 {text("axis"), number("target")});
    command_verb("movel", "move_l", "Cartesian move: TCP to x y z (metres)",
                 {number("x"), number("y"), number("z")});
    command_verb("movep", "move_pose", "6-DoF move: TCP to x y z at quaternion qw qx qy qz",
                 {number("x"), number("y"), number("z"), number("qw"), number("qx"), number("qy"),
                  number("qz")});

    std::string motion;
    bool run_quality = false;
    auto* c_run = app.add_subcommand("run", "run a named coordinated motion");
    c_run->add_option("motion", motion, "motion id (defaults to the cell's first)");
    c_run->add_flag("--quality", run_quality,
                    "print how the run went (cycles, overruns, jitter, budget) instead of telemetry");

    std::string new_cell;
    // The CLI boots its own Platform, so a cell it adds lives for this process.
    // Against a long-running server the same verbs are POST/DELETE /cells.
    std::string new_robot;
    auto* c_addcell = app.add_subcommand("add-cell", "start another robot from the catalogue");
    c_addcell->add_option("id", new_cell, "cell id")->required();
    c_addcell->add_option("--robot", new_robot,
                          "descriptor from `robonode robots` (default: this platform's own)");

    std::string drop_cell;
    auto* c_dropcell = app.add_subcommand("drop-cell", "stop and remove a cell (this process only)");
    c_dropcell->add_option("id", drop_cell, "cell id")->required();

    std::string belt_station;
    bool belt_off = false;
    auto* c_belt = app.add_subcommand("conveyor", "run or stop a belt");
    c_belt->add_option("station", belt_station, "station id")->required();
    c_belt->add_flag("--off", belt_off, "stop the belt instead of running it");

    auto* c_contacts = app.add_subcommand("contacts", "what is touching what, right now");

    double watch_s = 10.0;
    bool watch_json = false;
    std::string watch_fields;
    auto* c_watch = app.add_subcommand("watch", "follow telemetry until it settles");
    c_watch->add_option("--for", watch_s, "seconds to follow");
    c_watch->add_flag("--json", watch_json,
                      "one compact JSON object per change and nothing else (pipe into jq)");
    c_watch->add_option("--quiet", watch_fields,
                        "print only when these telemetry fields change (e.g. state,contacts)");

    std::string cmp_cap, cmp_a, cmp_b;
    auto* c_compare = app.add_subcommand(
        "compare", "run two versions of a capability against the work that exercises it");
    c_compare->add_option("capability", cmp_cap, "vision | tracking | planner | control")->required();
    c_compare->add_option("a", cmp_a, "first version")->required();
    c_compare->add_option("b", cmp_b, "second version")->required();


    CLI11_PARSE(app, argc, argv);

    const char* env_settings = std::getenv("ROBONODE_SETTINGS");
    robonode::Platform p{ROBONODE_WORLDS, ROBONODE_CELL, ROBONODE_APPS, ROBONODE_MODULES,
                         ROBONODE_SCENES, ROBONODE_SESSIONS,
                         robonode::settings_or_defaults(env_settings != nullptr ? env_settings
                                                                                : ROBONODE_SETTINGS),
                         robonode::vendor_drivers(), robonode::vision_versions(),
                         ROBONODE_ROBOTS};

    std::unique_ptr<robonode::cli::CellClient> cell;
    if (server.empty()) {
        cell = std::make_unique<robonode::cli::LocalCell>(p, session, on_cell);
    } else {
        cell = std::make_unique<robonode::cli::RemoteCell>(server, session, on_cell);
    }
    auto& c = *cell;

    // Logs read like the rest but do not PRINT like the rest: they are lines a
    // person reads, not a document. Everything else is the generic answer.
    if (*c_logs) {
        std::string body;
        if (const auto st = c.read("logs", body); !st.ok()) return fail(st);
        const auto arr = nlohmann::json::parse(body, nullptr, false);
        if (arr.is_discarded()) return fail(robonode::Status::failure("bad log payload"));
        for (const auto& line : arr) print(line.get<std::string>());
        return 0;
    }
    for (const auto& [sub, what] : reads) {
        if (*sub) return show(c, what);
    }
    for (const auto& one : one_of) {
        if (*one.sub) return show(c, one.collection + "/" + one.id);
    }

    if (*c_contacts) {
        std::string body;
        if (const auto st = c.read("telemetry", body); !st.ok()) return fail(st);
        const auto j = nlohmann::json::parse(body, nullptr, false);
        print(j.is_discarded() ? "[]" : j.value("contacts", nlohmann::json::array()).dump());
        return 0;
    }
    if (*c_watch) {
        // The agent's debugging loop: one line per change, nothing when nothing
        // happens, so a diff in the output IS the event. At 50 Hz "anything
        // changed" is most frames, so --quiet narrows what counts as a change.
        const auto fields = split_csv(watch_fields);
        const auto until = std::chrono::steady_clock::now() +
                           std::chrono::milliseconds{static_cast<long>(watch_s * 1000)};
        std::string last;
        while (std::chrono::steady_clock::now() < until) {
            std::string now;
            if (const auto st = c.read("telemetry", now); !st.ok()) return fail(st);
            const auto frame = nlohmann::json::parse(now, nullptr, false);
            std::string key = fields.empty() ? now : project(frame, fields);
            if (key != last) {
                print(watch_json && !frame.is_discarded() ? frame.dump() : now);
                last = std::move(key);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{200});
        }
        return 0;
    }

    if (*c_run) {
        // The run's own record, in the SAME process that ran it: each CLI
        // invocation boots its own platform, so asking a second one how the
        // first went answers about a cell that never moved.
        return act(c,
                   motion.empty() ? nlohmann::json{{"cmd", "run"}}
                                  : nlohmann::json{{"cmd", "run"}, {"motion", motion}},
                   run_quality ? "last-run" : "telemetry");
    }
    for (const auto& cv : commands) {
        if (!*cv.sub) continue;
        nlohmann::json body{{"cmd", cv.cmd}};
        std::string after = cv.refresh;
        std::size_t texts = 0, numbers = 0;
        for (const auto& arg : cv.args) {
            if (arg.kind == Arg::number) {
                body[arg.name] = cv.numbers[numbers++];
                continue;
            }
            const auto& value = cv.texts[texts++];
            body[arg.name] = value;
            const std::string hole = "{" + arg.name + "}";
            if (const auto at = after.find(hole); at != std::string::npos) {
                after.replace(at, hole.size(), value);
            }
        }
        return after.empty() ? act(c, body) : act(c, body, after);
    }
    if (*c_belt) {
        return act(c, {{"cmd", "conveyor"}, {"station", belt_station},
                       {"run", belt_off ? "false" : "true"}});
    }
    if (*c_compare) {
        const auto report = c.compare(cmp_cap, {cmp_a, cmp_b});
        if (!report) return fail(report.error());
        print(*report);
        return 0;
    }

    // Store and lifecycle verbs the command bus does not carry. They go through
    // the client, so they work against a running server exactly as they work in
    // process — they used to be refused with "local-only" for no reason the
    // server agreed with.
    if (*c_fork) {
        if (const auto st = c.fork_doc(fork_kind, fork_from, fork_to, fork_name); !st.ok()) {
            return fail(st);
        }
        return show(c, fork_kind);
    }
    if (*c_addcell) {
        if (const auto st = c.add_cell(new_cell, new_robot); !st.ok()) return fail(st);
        return show(c, "cells");
    }
    if (*c_dropcell) {
        if (const auto st = c.drop_cell(drop_cell); !st.ok()) return fail(st);
        return show(c, "cells");
    }
    return 0;
}
