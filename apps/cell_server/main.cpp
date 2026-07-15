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

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include <httplib.h>

#include "robonode/platform.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must point at the sim-mujoco worlds dir"
#endif
#ifndef ROBONODE_WEB
#error "ROBONODE_WEB must point at the cell_server web dir"
#endif
#ifndef ROBONODE_CELL
#error "ROBONODE_CELL must point at the cell descriptor JSON"
#endif

namespace {
// Compile-time paths are the default; env vars override so the same binary
// deploys anywhere (e.g. relocated inside a container).
std::string env_or(const char* var, const char* fallback) {
    const char* v = std::getenv(var);
    return v != nullptr ? std::string{v} : std::string{fallback};
}
}  // namespace

int main() {
    const std::string worlds = env_or("ROBONODE_WORLDS_DIR", ROBONODE_WORLDS);
    const std::string web = env_or("ROBONODE_WEB_DIR", ROBONODE_WEB);
    const std::string cell = env_or("ROBONODE_CELL_FILE", ROBONODE_CELL);

    // The one facade every surface binds to (#33). HTTP is just transport.
    robonode::Platform platform{worlds + "/rail_ur10e.xml", cell};
    httplib::Server svr;

    // Live stream: node tree once, then telemetry frames ~50 Hz; re-send the
    // node tree whenever the driver family is swapped.
    svr.Get("/events", [&platform](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_chunked_content_provider(
            "text/event-stream", [&platform](std::size_t, httplib::DataSink& sink) {
                const std::string init = "event: nodes\ndata: " + platform.nodes_json() + "\n\n";
                if (!sink.write(init.data(), init.size())) return false;
                std::string last_nodes = platform.nodes_json();
                int k = 0;
                while (true) {
                    const std::string nn = platform.nodes_json();
                    if (nn != last_nodes) {
                        const std::string m = "event: nodes\ndata: " + nn + "\n\n";
                        if (!sink.write(m.data(), m.size())) break;
                        last_nodes = nn;
                    }
                    const std::string frame = "data: " + platform.telemetry_json() + "\n\n";
                    if (!sink.write(frame.data(), frame.size())) break;
                    // Logs ~every 500 ms — the followable observability surface.
                    if (++k % 25 == 0) {
                        const std::string lg = "event: logs\ndata: " + platform.logs_json() + "\n\n";
                        if (!sink.write(lg.data(), lg.size())) break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds{20});
                }
                return true;
            });
    });

    svr.Post("/command", [&platform](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_content(platform.submit_command(req.body), "application/json");
    });

    svr.set_mount_point("/", web);

    std::printf("cell_server on http://localhost:8080  (web: %s)\n", web.c_str());
    std::printf("  GET /events (SSE)  POST /command {\"cmd\":\"run\"|\"driver\",...}\n");
    if (!svr.listen("0.0.0.0", 8080)) {
        std::fprintf(stderr, "failed to bind :8080\n");
        return 1;
    }
    return 0;
}
