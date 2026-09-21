#pragma once

#include <array>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/status.hpp"

namespace robonode {

// One thing placed in a scene: an obstacle to avoid, a fixture, another part to
// pick, a decoy for the camera. Everything a user needs to change the world is
// here as data — no XML, no rebuild.
struct SceneObject {
    std::string name;
    std::string type{"box"};      // box · sphere · cylinder
    std::string material{"part"};
    double size[3]{0.05, 0.05, 0.05};  // half-extents, MuJoCo's convention
    double pos[3]{0.0, 0.0, 0.0};
    bool collides{true};   // an obstacle collides; a decoy is only seen
    bool visible{true};
    double mass{0.5};
    // A single sliding degree of freedom, for something that travels.
    std::string slide_axis;        // e.g. "-1 0 0"; empty ⇒ fixed in place
    double slide_range[2]{-0.5, 0.5};
};

// A change to something the base world ALREADY declares (#93). Composition was
// additive only, so "start from an existing scene and make it yours" stopped at
// adding clutter: you could not move the conveyor, delete the decoy to make the
// vision problem easier, or make the part heavier. Each field is optional
// because an override says what changed and nothing else.
struct SceneOverride {
    std::string name;  // the body in the base world
    std::optional<std::array<double, 3>> pos;
    std::optional<bool> visible;  // false removes it from the model entirely
    std::optional<std::string> material;
    std::optional<double> mass;
};

// A scene: an existing world to start from, plus what you changed. Forking a
// scenario is copying this file and editing a number — the platform composes
// the physics model from it.
struct SceneDescriptor {
    std::string name;
    std::string base;  // the MJCF this scene starts from
    std::vector<SceneObject> objects;
    std::vector<SceneOverride> overrides;
};

namespace detail {

inline void read_triple(const nlohmann::json& j, const char* key, double (&out)[3]) {
    const auto it = j.find(key);
    if (it == j.end() || !it->is_array() || it->size() != 3) return;
    for (std::size_t i = 0; i < 3; ++i) out[i] = (*it)[i].get<double>();
}

inline SceneObject parse_object(const nlohmann::json& o) {
    SceneObject s;
    s.name = o.at("name").get<std::string>();
    s.type = o.value("type", s.type);
    s.material = o.value("material", s.material);
    s.collides = o.value("collides", s.collides);
    s.visible = o.value("visible", s.visible);
    s.mass = o.value("mass", s.mass);
    s.slide_axis = o.value("slide_axis", "");
    read_triple(o, "size", s.size);
    read_triple(o, "pos", s.pos);
    if (const auto r = o.find("slide_range"); r != o.end() && r->is_array() && r->size() == 2) {
        s.slide_range[0] = (*r)[0].get<double>();
        s.slide_range[1] = (*r)[1].get<double>();
    }
    return s;
}

}  // namespace detail

namespace detail {

inline SceneOverride parse_override(const nlohmann::json& o) {
    SceneOverride s;
    s.name = o.at("name").get<std::string>();
    if (const auto p = o.find("pos"); p != o.end() && p->is_array() && p->size() == 3) {
        s.pos = std::array<double, 3>{(*p)[0].get<double>(), (*p)[1].get<double>(),
                                      (*p)[2].get<double>()};
    }
    if (const auto v = o.find("visible"); v != o.end()) s.visible = v->get<bool>();
    if (const auto m = o.find("material"); m != o.end()) s.material = m->get<std::string>();
    if (const auto m = o.find("mass"); m != o.end()) s.mass = m->get<double>();
    return s;
}

}  // namespace detail

inline Status parse_scene_descriptor(const nlohmann::json& j, SceneDescriptor& out,
                                     const std::string& source = "scene") {
    if (j.is_discarded() || !j.is_object()) return Status::failure("invalid JSON: " + source);
    if (!j.contains("base")) return Status::failure(source + ": no base scene named");
    out.name = j.value("name", source);
    out.base = j.at("base").get<std::string>();
    try {
        if (const auto it = j.find("objects"); it != j.end() && it->is_array()) {
            for (const auto& o : *it) out.objects.push_back(detail::parse_object(o));
        }
        if (const auto it = j.find("overrides"); it != j.end() && it->is_array()) {
            for (const auto& o : *it) out.overrides.push_back(detail::parse_override(o));
        }
    } catch (const nlohmann::json::exception& e) {
        return Status::failure(source + ": " + e.what());
    }
    return Status::success();
}

inline Status load_scene_descriptor(const std::string& path, SceneDescriptor& out) {
    std::ifstream f{path};
    if (!f) return Status::failure("cannot open scene: " + path);
    return parse_scene_descriptor(nlohmann::json::parse(f, nullptr, false), out, path);
}

}  // namespace robonode
