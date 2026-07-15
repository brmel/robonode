#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/status.hpp"

namespace robonode {

// One saved application in the store.
struct AppEntry {
    std::string name;  // the app's display name
    std::string file;  // its filename (id within the store)
};

// Persistence seam (#59): store, list, and load applications. v0 is a
// filesystem backend (apps live as *.app.json in a directory) — enough to
// manage recipes on a dev box with zero setup. A SQLite or Postgres backend
// (the modular deployment's `postgres` service, docs/DEPLOYMENT.md) swaps in
// behind this exact interface for a fleet; the seam is the point, not the
// engine — so we never reinvent a database.
class AppStore {
public:
    explicit AppStore(std::string dir) : dir_{std::move(dir)} {}

    // Every saved app: reads each *.app.json and pulls its "name".
    [[nodiscard]] std::vector<AppEntry> list() const {
        std::vector<AppEntry> out;
        std::error_code ec;
        if (!std::filesystem::is_directory(dir_, ec)) return out;
        for (const auto& e : std::filesystem::directory_iterator(dir_, ec)) {
            const auto& p = e.path();
            if (p.extension() != ".json") continue;
            std::ifstream f{p};
            const auto j = nlohmann::json::parse(f, nullptr, false);
            out.push_back({j.is_discarded() ? p.stem().string() : j.value("name", p.stem().string()),
                           p.filename().string()});
        }
        std::sort(out.begin(), out.end(), [](auto& a, auto& b) { return a.file < b.file; });
        return out;
    }

    [[nodiscard]] std::string list_json() const {
        auto arr = nlohmann::json::array();
        for (const auto& e : list()) arr.push_back({{"name", e.name}, {"file", e.file}});
        return arr.dump();
    }

    Status load(const std::string& file, std::string& out) const {
        std::ifstream f{path_of(file)};
        if (!f) return Status::failure("no such app: " + file);
        std::ostringstream ss;
        ss << f.rdbuf();
        out = ss.str();
        return Status::success();
    }

    // Create/replace an app (the editor #58 uses this).
    Status save(const std::string& file, const std::string& json) const {
        std::error_code ec;
        std::filesystem::create_directories(dir_, ec);
        std::ofstream f{path_of(file), std::ios::trunc};
        if (!f) return Status::failure("cannot write app: " + file);
        f << json;
        return Status::success();
    }

private:
    [[nodiscard]] std::string path_of(const std::string& file) const {
        return (std::filesystem::path{dir_} / file).string();
    }
    std::string dir_;
};

}  // namespace robonode
