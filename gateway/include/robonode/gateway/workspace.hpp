#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/celld/json_doc_store.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// One user's workspace: their applications, their algorithms, their scenes.
// Sessions exist so people can work on the same platform without editing each
// other's files — everything a user authors is scoped to their id, and the
// shipped library stays read-only underneath them.
class Workspace {
public:
    // The kinds of document a workspace holds, and the suffix that identifies
    // each on disk. A fourth kind is one row — never a fourth field, a fourth
    // accessor and a fourth set of routes.
    static constexpr std::pair<const char*, const char*> kKinds[]{{"apps", ".app.json"},
                                                                  {"modules", ".module.json"},
                                                                  {"scenes", ".scene.json"},
                                                                  {"runs", ".run.json"},
                                                                  {"robots", ".cell.json"}};

    Workspace(std::string root, std::string id) : id_{std::move(id)}, root_{std::move(root)} {
        const auto dir = std::filesystem::path{root_} / id_;
        for (const auto& [kind, suffix] : kKinds) {
            stores_.emplace(kind, std::make_unique<JsonDocStore>((dir / kind).string(), suffix));
        }
    }

    [[nodiscard]] const std::string& id() const { return id_; }
    [[nodiscard]] JsonDocStore& docs(const std::string& kind) { return *stores_.at(kind); }
    [[nodiscard]] JsonDocStore& apps() { return docs("apps"); }
    [[nodiscard]] JsonDocStore& modules() { return docs("modules"); }
    [[nodiscard]] JsonDocStore& scenes() { return docs("scenes"); }

private:
    std::string id_, root_;
    std::map<std::string, std::unique_ptr<JsonDocStore>> stores_;
};

// The sessions on this platform. A session is created on first use, so a client
// simply names one and starts working.
class Sessions {
public:
    explicit Sessions(std::string root) : root_{std::move(root)} {}

    // Every HTTP request resolves its session, and httplib answers each one on
    // its own thread — so this map is written from many threads at once. It was
    // not guarded, which corrupts the tree rather than losing an entry.
    //
    // The reference handed back stays valid because the workspaces are held by
    // unique_ptr: only the MAP is shared, and only its mutation needs the lock.
    Workspace& open(const std::string& id) {
        const auto& key = id.empty() ? kShared : id;
        std::lock_guard<std::mutex> lk{mtx_};
        const auto it = open_.find(key);
        if (it != open_.end()) return *it->second;
        auto ws = std::make_unique<Workspace>(root_, key);
        auto& ref = *ws;
        open_[key] = std::move(ws);
        return ref;
    }

    [[nodiscard]] bool exists(const std::string& id) const {
        std::error_code ec;
        return std::filesystem::is_directory(std::filesystem::path{root_} / id, ec);
    }

    // Every session that has left something on disk, plus the shared one.
    [[nodiscard]] std::string list_json() const {
        auto arr = nlohmann::json::array();
        arr.push_back(kShared);
        std::error_code ec;
        if (std::filesystem::is_directory(root_, ec)) {
            for (const auto& e : std::filesystem::directory_iterator(root_, ec)) {
                if (e.is_directory(ec) && e.path().filename() != kShared) {
                    arr.push_back(e.path().filename().string());
                }
            }
        }
        return arr.dump();
    }

    Status remove(const std::string& id) {
        if (id.empty() || id == kShared) return Status::failure("the shared session stays");
        std::error_code ec;
        const auto removed = std::filesystem::remove_all(std::filesystem::path{root_} / id, ec);
        std::lock_guard<std::mutex> lk{mtx_};
        open_.erase(id);
        return removed > 0 ? Status::success() : Status::failure("no session '" + id + "'");
    }

    static constexpr const char* kShared = "shared";

private:
    std::string root_;
    mutable std::mutex mtx_;  // the open-session map is written from request threads
    std::map<std::string, std::unique_ptr<Workspace>> open_;
};

}  // namespace robonode
