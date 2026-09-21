// cell_server — the live cell behind the web app. Boots the 7-DOF arm cell,
// serves its node tree + live joint telemetry over Server-Sent Events, and
// accepts run / driver-swap commands over POST. The web app (served static)
// renders the robot in 3D and drives it.
//
//   ./cell_server                 → http://localhost:8080
//
// Transport is a thin HTTP+SSE gateway (not WebSocket framing) so it is
// self-contained and robust; the roadmap Zenoh/gRPC surface swaps in behind
// the same CellGateway seam.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <string>
#include <thread>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "robonode/platform.hpp"
#include "robonode/apps/vendors.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must point at the sim-mujoco worlds dir"
#endif
#ifndef ROBONODE_WEB
#error "ROBONODE_WEB must point at the cell_server web dir"
#endif
#ifndef ROBONODE_ROBOTS
#error "ROBONODE_ROBOTS must point at the robot (cell descriptor) catalogue"
#endif
#ifndef ROBONODE_CELL
#error "ROBONODE_CELL must point at the cell descriptor JSON"
#endif
#ifndef ROBONODE_APPS
#error "ROBONODE_APPS must point at the applications dir"
#endif
#ifndef ROBONODE_MODULES
#error "ROBONODE_MODULES must point at the user-modules dir"
#endif

namespace {
// Compile-time paths are the default; env vars override so the same binary
// deploys anywhere (e.g. relocated inside a container).
std::string env_or(const char* var, const char* fallback) {
    const char* v = std::getenv(var);
    return v != nullptr ? std::string{v} : std::string{fallback};
}
int env_or_int(const char* var, int fallback) {
    const char* v = std::getenv(var);
    return v != nullptr ? std::atoi(v) : fallback;
}

std::atomic<bool> g_stop{false};
void request_stop(int) { g_stop.store(true); }
}  // namespace

int main() {
    const std::string worlds = env_or("ROBONODE_WORLDS_DIR", ROBONODE_WORLDS);
    const std::string web = env_or("ROBONODE_WEB_DIR", ROBONODE_WEB);
    const std::string cell = env_or("ROBONODE_CELL_FILE", ROBONODE_CELL);
    const std::string robots = env_or("ROBONODE_ROBOTS_DIR", ROBONODE_ROBOTS);
    const std::string apps = env_or("ROBONODE_APPS_DIR", ROBONODE_APPS);
    const std::string modules = env_or("ROBONODE_MODULES_DIR", ROBONODE_MODULES);
    const std::string settings_path = env_or("ROBONODE_SETTINGS", ROBONODE_SETTINGS);
    const std::string scenes = env_or("ROBONODE_SCENES_DIR", ROBONODE_SCENES);
    const std::string sessions = env_or("ROBONODE_SESSIONS_DIR", ROBONODE_SESSIONS);

    // Production controls (safe defaults). Bind loopback so robot control is not
    // exposed to the network unless explicitly opened; CORS is off unless an
    // origin is set (the UI is same-origin, so it needs none).
    const std::string host = env_or("ROBONODE_HOST", "127.0.0.1");
    const int port = env_or_int("ROBONODE_PORT", 8080);
    const std::string cors = env_or("ROBONODE_CORS_ORIGIN", "");
    // A public instance still lets a visitor fork a scene, author an algorithm
    // and run it — that is the point of the platform. What it refuses is the
    // destructive minority: deleting someone else's session, and spawning cells
    // (a physics cell is a CPU, so "add ten" is a denial of service).
    const bool public_mode = !env_or("ROBONODE_PUBLIC", "").empty();

    // Tuning is data (config/robonode.settings.json): no heuristic is a literal.
    std::string settings_note;
    const auto settings = robonode::settings_or_defaults(settings_path, &settings_note);

    // The one facade every surface binds to (#33). HTTP is just transport.
    robonode::Platform platform{worlds,
                                cell,
                                apps,
                                modules,
                                scenes,
                                sessions,
                                settings,
                                robonode::vendor_drivers(),
                                robonode::vision_versions(),
                                robots};
    httplib::Server svr;

    // One place for the cross-origin header (no per-route duplication); emitted
    // only when an origin is configured.
    if (!cors.empty()) {
        svr.set_post_routing_handler([cors](const httplib::Request&, httplib::Response& res) {
            res.set_header("Access-Control-Allow-Origin", cors);
        });
    }

    // Live stream: node tree once, then telemetry frames ~50 Hz; re-send the
    // node tree whenever the driver family is swapped.
    const int stream_period_ms = settings.gateway.stream_period_ms;
    const int log_every = std::max(1, settings.gateway.log_every_n_frames);
    // Which session a request works in. Everything a user authors is scoped to
    // it; the shipped library shows through underneath.
    const auto session_of = [](const httplib::Request& req) {
        if (req.has_param("session")) return req.get_param_value("session");
        return req.get_header_value("X-Robonode-Session");
    };

    // Which robot a read is about. Same shape as the session: a query parameter,
    // defaulting to the platform's first cell.
    const auto cell_of = [](const httplib::Request& req) {
        return req.has_param("cell") ? req.get_param_value("cell") : std::string{};
    };

    // A read that names a robot the platform does not have is refused. Answering
    // it with the FIRST robot's telemetry would be a plausible wrong answer —
    // the worst kind, because nothing about it looks wrong.
    const auto no_such_cell = [&platform](const httplib::Request& req, httplib::Response& res) {
        const std::string on = req.has_param("cell") ? req.get_param_value("cell") : std::string{};
        if (platform.has_cell(on)) return false;
        res.status = 404;
        res.set_content(nlohmann::json{{"ok", false}, {"error", "no cell '" + on + "'"}}.dump(),
                        "application/json");
        return true;
    };
    const auto refused = robonode::Status::failure(
        "refused: this instance runs in public mode (ROBONODE_PUBLIC)");
    const auto reply = [](httplib::Response& res, const robonode::Status& st) {
        res.status = st.ok() ? 200 : 400;
        res.set_content(st.ok() ? R"({"ok":true})"
                                : nlohmann::json{{"ok", false}, {"error", st.message()}}.dump(),
                        "application/json");
    };

    // The stream follows the robot the client asked for, so a surface watching a
    // second cell sees that cell's telemetry rather than the first one's.
    svr.Get("/events", [&](const httplib::Request& req, httplib::Response& res) {
        if (no_such_cell(req, res)) return;
        const std::string on = cell_of(req);
        res.set_chunked_content_provider(
            "text/event-stream", [&, on](std::size_t, httplib::DataSink& sink) {
                const std::string init = "event: nodes\ndata: " + platform.nodes_json(on) + "\n\n";
                if (!sink.write(init.data(), init.size())) return false;
                std::string last_nodes = platform.nodes_json(on);
                int k = 0;
                while (true) {
                    // A stream outlives the page that opened it: if the robot it
                    // follows is dropped, the stream ends rather than reporting
                    // about a different one.
                    if (!platform.has_cell(on)) break;
                    const std::string nn = platform.nodes_json(on);
                    if (nn != last_nodes) {
                        const std::string m = "event: nodes\ndata: " + nn + "\n\n";
                        if (!sink.write(m.data(), m.size())) break;
                        last_nodes = nn;
                    }
                    const std::string frame = "data: " + platform.telemetry_json(on) + "\n\n";
                    if (!sink.write(frame.data(), frame.size())) break;
                    // Logs ~every 500 ms — the followable observability surface.
                    if (++k % log_every == 0) {
                        const std::string lg = "event: logs\ndata: " + platform.logs_json(on) + "\n\n";
                        if (!sink.write(lg.data(), lg.size())) break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds{stream_period_ms});
                }
                return true;
            });
    });

    svr.Post("/command", [&platform](const httplib::Request& req, httplib::Response& res) {
        res.set_content(platform.submit_command(req.body), "application/json");
    });

    svr.Get("/sessions", [&platform](const httplib::Request&, httplib::Response& res) {
        res.set_content(platform.sessions_json(), "application/json");
    });
    svr.Delete(R"(/sessions/([\w-]+))", [&](const httplib::Request& req, httplib::Response& res) {
        if (public_mode) return reply(res, refused);
        reply(res, platform.delete_session(req.matches[1]));
    });

    // WHICH live views exist comes from the platform's table, so adding one is
    // a row there and nothing here — and none of them can forget to refuse a
    // robot that is not present, because there is one implementation of both
    // the refusal and the answer.
    for (const auto& v : robonode::Platform::kViews) {
        svr.Get("/" + std::string{v.name},
                [&, name = std::string{v.name}](const httplib::Request& req, httplib::Response& res) {
                    auto body = platform.view_json(name, cell_of(req));
                    if (!body) {
                        res.status = 404;
                        res.set_content(
                            nlohmann::json{{"ok", false}, {"error", body.error().message()}}.dump(),
                            "application/json");
                        return;
                    }
                    res.set_content(*body, "application/json");
                });
    }

    // Every document collection — scenes, applications, algorithms, run records
    // — is the same five routes over the same store, so they are written once
    // and the kind comes from the path. WHICH kinds exist comes from the
    // platform, so adding one is a row in its table and nothing here.
    const auto kinds = [] {
        std::string pattern;
        for (const auto& kind : robonode::Platform::kinds()) {
            pattern += (pattern.empty() ? "" : "|") + kind;
        }
        return pattern;
    }();
    const auto docs = [&](const httplib::Request& req) { return std::string{req.matches[1]}; };
    svr.Get("/(" + kinds + ")", [&](const httplib::Request& req, httplib::Response& res) {
        res.set_content(platform.docs_json(docs(req), session_of(req)), "application/json");
    });
    svr.Get("/(" + kinds + R"()/(.+\.json))", [&](const httplib::Request& req, httplib::Response& res) {
        if (const auto body = platform.doc(docs(req), req.matches[2], session_of(req))) {
            res.set_content(*body, "application/json");
        } else {
            res.status = 404;
        }
    });
    svr.Put("/(" + kinds + R"()/(.+\.json))", [&](const httplib::Request& req, httplib::Response& res) {
        reply(res, platform.save_doc(docs(req), req.matches[2], req.body, session_of(req)));
    });
    svr.Delete("/(" + kinds + R"()/(.+\.json))", [&](const httplib::Request& req, httplib::Response& res) {
        reply(res, platform.delete_doc(docs(req), req.matches[2], session_of(req)));
    });
    // Fork to start from something that works: the shipped document is copied
    // into the caller's session under a new name, and theirs from then on.
    svr.Post("/(" + kinds + R"()/(.+\.json)/fork)", [&](const httplib::Request& req, httplib::Response& res) {
        const auto j = nlohmann::json::parse(req.body, nullptr, false);
        const std::string to = j.is_object() ? j.value("to", "") : "";
        if (to.empty()) return reply(res, robonode::Status::failure("need a name to fork to"));
        reply(res, platform.fork_doc(docs(req), req.matches[2], to, j.value("name", ""),
                                     session_of(req)));
    });



    svr.Get(R"(/capabilities/([\w-]+))", [&](const httplib::Request& req, httplib::Response& res) {
        if (no_such_cell(req, res)) return;
        std::string view;
        if (const auto st = platform.capability_json(req.matches[1], view, cell_of(req)); !st.ok()) {
            res.status = 404;
            res.set_content(nlohmann::json{{"ok", false}, {"error", st.message()}}.dump(),
                            "application/json");
            return;
        }
        res.set_content(view, "application/json");
    });

    // What the detector sees, as an image. A user tuning a vision algorithm
    // needs the frame, not a claim about it.
    svr.Get("/camera.bmp", [&](const httplib::Request& req, httplib::Response& res) {
        if (no_such_cell(req, res)) return;
        auto bmp = platform.camera_bmp(cell_of(req));
        if (bmp.empty()) {
            res.status = 503;
            res.set_content(R"({"ok":false,"error":"no camera in this cell"})", "application/json");
            return;
        }
        res.set_header("Cache-Control", "no-store");
        res.set_content(std::move(bmp), "image/bmp");
    });

    // Compare two versions of a capability on the work that exercises it. The
    // platform runs it; a surface renders what comes back, so "A vs B" cannot
    // mean two different things in the browser and in the CLI.
    svr.Post("/compare", [&](const httplib::Request& req, httplib::Response& res) {
        const auto body = nlohmann::json::parse(req.body, nullptr, false);
        if (!body.is_object()) return reply(res, robonode::Status::failure("need a request body"));
        const auto versions = body.value("versions", std::vector<std::string>{});
        if (versions.size() < 2) return reply(res, robonode::Status::failure("need two versions"));
        std::string out;
        // A comparison runs work on a robot; which one is the caller's choice,
        // and the body may say so just as a command does.
        const auto st = platform.compare(body.value("capability", ""), versions, out,
                                         session_of(req), body.value("cell", ""));
        if (!st.ok()) return reply(res, st);
        res.set_content(out, "application/json");
    });



    // The running cells: one "main" at boot, a second robot is a POST naming a
    // descriptor from the catalogue that GET /robots serves.
    svr.Get("/cells", [&platform](const httplib::Request&, httplib::Response& res) {
        res.set_content(platform.cells_json(), "application/json");
    });
    svr.Post("/cells", [&](const httplib::Request& req, httplib::Response& res) {
        if (public_mode) return reply(res, refused);
        const auto j = nlohmann::json::parse(req.body, nullptr, false);
        if (j.is_discarded() || !j.contains("id")) {
            res.status = 400;
            res.set_content(R"({"ok":false,"error":"need id"})", "application/json");
            return;
        }
        // `robot` names a descriptor in the catalogue (GET /robots); omitted, the
        // new cell is another of whatever this platform booted with.
        reply(res, platform.add_cell(j.at("id").get<std::string>(),
                                     j.value("robot", j.value("cell", "")), session_of(req)));
    });
    svr.Delete(R"(/cells/([\w-]+))", [&](const httplib::Request& req, httplib::Response& res) {
        if (public_mode) return reply(res, refused);
        reply(res, platform.remove_cell(req.matches[1]));
    });


    svr.set_mount_point("/assets", worlds + "/assets");  // UR10e meshes for the 3D view
    svr.set_mount_point("/", web);

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    std::printf("cell_server on http://%s:%d  (web: %s%s%s)\nsettings: %s\n", host.c_str(), port,
                web.c_str(), cors.empty() ? "" : " · CORS open",
                public_mode ? " · public mode" : "", settings_note.c_str());
    // listen() blocks; run it off the main thread so a signal can stop it.
    std::atomic<bool> bound{true};
    std::thread http{[&] {
        if (!svr.listen(host, port)) {
            std::fprintf(stderr, "failed to bind %s:%d\n", host.c_str(), port);
            bound.store(false);
            g_stop.store(true);
        }
    }};
    http.detach();
    while (!g_stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds{100});
    // Stop accepting, let in-flight requests drain, then exit deterministically
    // (httplib's listen does not always unblock on stop()); the OS reclaims the
    // detached listener. No persisted state is buffered, so this is clean.
    svr.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds{150});
    std::fflush(stdout);
    std::_Exit(bound.load() ? 0 : 1);
}
