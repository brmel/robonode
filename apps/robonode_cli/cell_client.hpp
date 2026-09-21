#pragma once

// Where the CLI's verbs land. Two implementations, one contract (ADR-8):
//
//   local  — boots a Platform in this process. What you get with no --server:
//            a cell of your own, useful for scripted runs and CI.
//   remote — talks to a cell_server over the same HTTP surface the web app
//            uses. This is the one an agent debugging a LIVE cell needs:
//            without it `robonode telemetry` answers about a different robot
//            than the one on screen, which is worse than having no CLI at all.
//
// Reads are named by their endpoint, so adding a view means adding a name.

#include <chrono>
#include <string>
#include <thread>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "robonode/core/result.hpp"
#include "robonode/core/status.hpp"
#include "robonode/gateway/progress.hpp"
#include "robonode/platform.hpp"

namespace robonode::cli {

class CellClient {
public:
    virtual ~CellClient() = default;

    // A view, by endpoint name ("telemetry", "nodes", "capabilities/vision").
    virtual Status read(const std::string& what, std::string& out) = 0;

    // A command, in the wire shape every surface uses: {"cmd": …}.
    virtual Status command(const nlohmann::json& body) = 0;

    // Block until the cell has applied everything accepted and is idle again.
    virtual Status settled() = 0;

    // Store and lifecycle work: not commands on the bus, but operations the
    // server has always served and this client used to refuse. "local-only"
    // meant the CLI could not fork a document, judge two algorithms or start a
    // second robot against a RUNNING platform — a capability in one surface and
    // not another, which is the drift ADR-8 exists to prevent.
    virtual Status fork_doc(const std::string& kind, const std::string& from,
                            const std::string& to, const std::string& name) = 0;
    virtual Result<std::string> compare(const std::string& capability,
                                        const std::vector<std::string>& versions) = 0;
    virtual Status add_cell(const std::string& id, const std::string& robot) = 0;
    virtual Status drop_cell(const std::string& id) = 0;
};

class LocalCell final : public CellClient {
public:
    LocalCell(Platform& platform, std::string session, std::string cell = {})
        : p_{platform}, session_{std::move(session)}, cell_{std::move(cell)} {}

    Status read(const std::string& what, std::string& out) override {
        // The live views come from the platform's table, so this surface cannot
        // be missing one the server publishes — which is what a hand-written
        // chain of names had been quietly free to be (ADR-8).
        // A robot this platform does not have is an error, not the first
        // robot's answer — the same rule, from the same place, as the server.
        if (auto body = p_.view_json(what, cell_)) return ok(std::move(*body), out);
        if (!p_.has_cell(cell_)) return Status::failure("no cell '" + cell_ + "'");
        if (what == "cells") return ok(p_.cells_json(), out);
        if (what == "sessions") return ok(p_.sessions_json(), out);

        // Everything left addresses ONE thing inside a collection, and they all
        // spell it the same way — `<collection>/<name>`. Splitting once, here,
        // is what stopped this being a prefix test and an offset into the
        // string per collection (`rfind("capabilities/", 0)`, then `substr(13)`).
        const auto slash = what.find('/');
        const std::string collection = what.substr(0, slash);
        const std::string name = slash == std::string::npos ? std::string{} : what.substr(slash + 1);
        if (collection == "capabilities" && !name.empty()) return p_.capability_json(name, out);
        // A document collection reads the same way over HTTP and in-process:
        // `<kind>` lists it, `<kind>/<file>` is one document.
        if (p_.has_kind(collection)) {
            if (name.empty()) return ok(p_.docs_json(collection, session_), out);
            auto body = p_.doc(collection, name, session_);
            if (!body) return body.error();
            out = std::move(*body);
            return Status::success();
        }
        return Status::failure("no view '" + what + "'");
    }

    Status command(const nlohmann::json& body) override {
        auto with_session = body;
        if (!session_.empty()) with_session["session"] = session_;
        if (!cell_.empty()) with_session["cell"] = cell_;
        const auto ack = nlohmann::json::parse(p_.submit_command(with_session.dump()), nullptr,
                                               false);
        if (ack.is_discarded() || !ack.value("ok", false)) {
            return Status::failure(ack.is_discarded() ? "bad response"
                                                      : ack.value("error", "command rejected"));
        }
        return Status::success();
    }

    Status settled() override { return p_.await_settled({}, cell_); }

    Status fork_doc(const std::string& kind, const std::string& from, const std::string& to,
                    const std::string& name) override {
        return p_.fork_doc(kind, from, to, name, session_);
    }
    Result<std::string> compare(const std::string& capability,
                                const std::vector<std::string>& versions) override {
        std::string out;
        if (const auto st = p_.compare(capability, versions, out, session_, cell_); !st.ok()) {
            return no(st);
        }
        return out;
    }
    Status add_cell(const std::string& id, const std::string& robot) override {
        return p_.add_cell(id, robot, session_);
    }
    Status drop_cell(const std::string& id) override { return p_.remove_cell(id); }

private:
    static Status ok(std::string value, std::string& out) {
        out = std::move(value);
        return Status::success();
    }

    Platform& p_;
    std::string session_;
    std::string cell_;  // which robot this client drives; empty = the first
};

class RemoteCell final : public CellClient {
public:
    RemoteCell(const std::string& url, std::string session, std::string cell = {})
        : http_{host_of(url), port_of(url)}, session_{std::move(session)},
          cell_{std::move(cell)} {
        http_.set_read_timeout(std::chrono::seconds{120});
    }

    Status read(const std::string& what, std::string& out) override {
        const auto res = http_.Get(path("/" + what));
        if (!res) return Status::failure("no cell server at " + std::string{http_.host()});
        if (res->status != 200) return Status::failure(what + ": HTTP " + std::to_string(res->status));
        out = res->body;
        return Status::success();
    }

    Status command(const nlohmann::json& body) override {
        auto with_session = body;
        if (!session_.empty()) with_session["session"] = session_;
        if (!cell_.empty()) with_session["cell"] = cell_;
        const auto res = http_.Post("/command", with_session.dump(), "application/json");
        if (!res) return Status::failure("no cell server at " + std::string{http_.host()});
        const auto ack = nlohmann::json::parse(res->body, nullptr, false);
        if (ack.is_discarded() || !ack.value("ok", false)) {
            return Status::failure(ack.is_discarded() ? "bad response"
                                                      : ack.value("error", "command rejected"));
        }
        return Status::success();
    }

    // The facade's predicate, over the wire — literally the same one, because
    // two definitions of "done" is two chances to disagree about the only thing
    // a caller is waiting on (ADR-12).
    Status settled() override {
        std::string last_token;
        auto quiet_since = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - quiet_since < std::chrono::seconds{30}) {
            std::string body;
            if (const auto st = read("telemetry", body); !st.ok()) return st;
            const auto now = progress_of(nlohmann::json::parse(body, nullptr, false));
            if (now.settled) {
                return now.error.empty() ? Status::success() : Status::failure(now.error);
            }
            if (!now.token.empty() && now.token != last_token) {
                last_token = now.token;
                quiet_since = std::chrono::steady_clock::now();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
        }
        return Status::failure("the cell stopped reporting progress");
    }

    Status fork_doc(const std::string& kind, const std::string& from, const std::string& to,
                    const std::string& name) override {
        return posted(path("/" + kind + "/" + from + "/fork"),
                      nlohmann::json{{"to", to}, {"name", name}});
    }
    Result<std::string> compare(const std::string& capability,
                                const std::vector<std::string>& versions) override {
        const auto res = http_.Post(path("/compare"),
                                    nlohmann::json{{"capability", capability},
                                                   {"versions", versions},
                                                   {"cell", cell_}}
                                        .dump(),
                                    "application/json");
        if (!res) return no("no cell server at " + std::string{http_.host()});
        if (res->status != 200) return no("compare: HTTP " + std::to_string(res->status));
        return res->body;
    }
    Status add_cell(const std::string& id, const std::string& robot) override {
        return posted(path("/cells"), nlohmann::json{{"id", id}, {"robot", robot}});
    }
    Status drop_cell(const std::string& id) override {
        const auto res = http_.Delete(path("/cells/" + id));
        if (!res) return Status::failure("no cell server at " + std::string{http_.host()});
        return accepted(res->body, res->status);
    }

private:
    Status posted(const std::string& where, const nlohmann::json& body) {
        const auto res = http_.Post(where, body.dump(), "application/json");
        if (!res) return Status::failure("no cell server at " + std::string{http_.host()});
        return accepted(res->body, res->status);
    }

    // The server answers {"ok":…} for every one of these; a non-200 with no
    // body is still a refusal worth naming.
    static Status accepted(const std::string& body, int status) {
        const auto j = nlohmann::json::parse(body, nullptr, false);
        if (!j.is_discarded() && j.value("ok", false)) return Status::success();
        if (!j.is_discarded() && j.contains("error")) {
            return Status::failure(j.value("error", "refused"));
        }
        return Status::failure("HTTP " + std::to_string(status));
    }

    // Both scopes ride as query parameters, the way the browser sends them.
    std::string path(const std::string& endpoint) const {
        std::string query;
        if (!session_.empty()) query += (query.empty() ? "?" : "&") + std::string{"session="} + session_;
        if (!cell_.empty()) query += (query.empty() ? "?" : "&") + std::string{"cell="} + cell_;
        return endpoint + query;
    }

    static std::string strip_scheme(const std::string& url) {
        const auto sep = url.find("://");
        return sep == std::string::npos ? url : url.substr(sep + 3);
    }
    static std::string host_of(const std::string& url) {
        const auto rest = strip_scheme(url);
        const auto colon = rest.find(':');
        return colon == std::string::npos ? rest : rest.substr(0, colon);
    }
    static int port_of(const std::string& url) {
        const auto rest = strip_scheme(url);
        const auto colon = rest.find(':');
        return colon == std::string::npos ? 8080 : std::atoi(rest.c_str() + colon + 1);
    }

    httplib::Client http_;
    std::string session_;
    std::string cell_;
};

}  // namespace robonode::cli
