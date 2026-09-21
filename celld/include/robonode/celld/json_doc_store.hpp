#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/result.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// One document in the store.
struct DocEntry {
    std::string name;  // display name (the doc's "name" field, else its stem)
    std::string file;  // filename — the id within the store
};

// Persistence seam: store, list, and load JSON documents of one kind. The kind
// is the filename suffix (`.app.json`, `.module.json`), so one implementation
// serves every document store instead of a class per document type.
//
// v0 is a filesystem backend — enough to manage recipes on a dev box with zero
// setup. A SQLite or Postgres backend swaps in behind this exact interface for
// a fleet; the seam is the point, not the engine.
class JsonDocStore {
public:
    explicit JsonDocStore(std::string dir, std::string suffix = ".json")
        : dir_{std::move(dir)}, suffix_{std::move(suffix)} {
        sweep_staging();
    }

    // Every document of this kind, with its display name.
    [[nodiscard]] std::vector<DocEntry> list() const {
        std::vector<DocEntry> out;
        std::error_code ec;
        if (!std::filesystem::is_directory(dir_, ec)) return out;
        for (const auto& e : std::filesystem::directory_iterator(dir_, ec)) {
            const auto& p = e.path();
            if (!matches(p.filename().string())) continue;
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

    // The document, or why it is not there. A caller that wants the text says
    // so in an expression instead of declaring an empty string first.
    [[nodiscard]] Result<std::string> read(const std::string& file) const {
        std::string body;
        if (const auto st = load(file, body); !st.ok()) return no(st);
        return body;
    }

    Status load(const std::string& file, std::string& out) const {
        std::ifstream f{path_of(file)};
        if (!f) return Status::failure("no such document: " + file);
        std::ostringstream ss;
        ss << f.rdbuf();
        out = ss.str();
        return Status::success();
    }

    // Create/replace a document (the app editor and the module library).
    // Write beside the target, then rename onto it. Two clients saving the same
    // document at once would otherwise interleave into one truncated file that
    // neither of them wrote — and a reader between the truncate and the flush
    // sees an empty document. Rename is atomic, so a reader sees the old
    // version or the new one, never half of either.
    Status save(const std::string& file, const std::string& json) const {
        std::error_code ec;
        std::filesystem::create_directories(dir_, ec);
        const auto target = path_of(file);
        const auto staging = target + ".part" + std::to_string(next_write());
        {
            std::ofstream f{staging, std::ios::trunc};
            if (!f) return Status::failure("cannot write: " + file);
            f << json;
            if (!f) return Status::failure("cannot write: " + file);
        }
        std::filesystem::rename(staging, target, ec);
        if (ec) {
            std::filesystem::remove(staging, ec);
            return Status::failure("cannot replace: " + file);
        }
        return Status::success();
    }

    Status remove(const std::string& file) const {
        std::error_code ec;
        if (!std::filesystem::remove(path_of(file), ec) || ec) {
            return Status::failure("no such entry: " + file);
        }
        return Status::success();
    }

private:
    // A write that died between staging and rename leaves a `.part` behind.
    // Nothing reads them — `matches()` ignores them — but a directory that
    // accumulates debris forever is a leak with a tidy name. Opening the store
    // is the moment nobody is mid-write, so it is the moment to clear them.
    void sweep_staging() const {
        std::error_code ec;
        if (!std::filesystem::is_directory(dir_, ec)) return;
        for (const auto& e : std::filesystem::directory_iterator(dir_, ec)) {
            const auto name = e.path().filename().string();
            if (name.find(".part") != std::string::npos) {
                std::filesystem::remove(e.path(), ec);
            }
        }
    }

    static std::uint64_t next_write() {
        static std::atomic<std::uint64_t> seq{0};
        return seq.fetch_add(1);
    }

    [[nodiscard]] bool matches(const std::string& filename) const {
        return filename.size() > suffix_.size() &&
               filename.compare(filename.size() - suffix_.size(), suffix_.size(), suffix_) == 0;
    }

    [[nodiscard]] std::string path_of(const std::string& file) const {
        return (std::filesystem::path{dir_} / file).string();
    }

    std::string dir_, suffix_;
};

}  // namespace robonode
