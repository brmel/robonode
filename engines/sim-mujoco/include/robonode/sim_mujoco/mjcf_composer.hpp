#pragma once

#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>

#include <tinyxml2.h>

#include "robonode/celld/scene_descriptor.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// Builds a physics model from a scene descriptor: take the base world the user
// started from, apply what they changed about it, add what they placed, and
// write the result where the engine can load it.
//
// This is the only place that knows MJCF is XML, and it uses the parser MuJoCo
// itself uses — editing an attribute on a named body is not a job for string
// surgery, and a second XML implementation is a second set of bugs.
//
// A user edits JSON — a number, a position, one more obstacle — and the scene
// changes; nothing above here sees a tag.
class MjcfComposer {
public:
    MjcfComposer(std::string worlds_dir, std::string cache_dir)
        : worlds_{std::move(worlds_dir)}, cache_{std::move(cache_dir)} {}

    // Returns a path an engine can load. A scene that changes nothing is the
    // base world itself — composing nothing should cost nothing.
    Status compose(const SceneDescriptor& scene, std::string& world_path) const {
        const auto base = (std::filesystem::path{worlds_} / scene.base).string();
        if (scene.objects.empty() && scene.overrides.empty()) {
            world_path = base;
            return Status::success();
        }

        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(base.c_str()) != tinyxml2::XML_SUCCESS) {
            return Status::failure("cannot open base scene: " + base);
        }
        auto* root = doc.RootElement();
        auto* worldbody = root != nullptr ? root->FirstChildElement("worldbody") : nullptr;
        if (worldbody == nullptr) return Status::failure(base + ": no worldbody");

        if (const auto st = apply_overrides(scene, worldbody); !st.ok()) return st;
        if (const auto st = add_objects(scene, doc, worldbody); !st.ok()) return st;

        tinyxml2::XMLPrinter printer;
        doc.Print(&printer);
        const std::string xml = printer.CStr();

        std::error_code ec;
        std::filesystem::create_directories(cache_, ec);
        // Composed scenes live beside the base so relative mesh paths resolve,
        // and carry a digest of their content: engines cache worlds by path, so
        // an edited scene has to be a different file to be a different world.
        world_path = (std::filesystem::path{worlds_} /
                      (".composed-" + safe(scene.name) + "-" + digest(xml) + ".xml"))
                         .string();
        std::ofstream out{world_path, std::ios::trunc};
        if (!out) return Status::failure("cannot write composed scene: " + world_path);
        out << xml;
        return Status::success();
    }

private:
    static std::string digest(const std::string& xml) {
        std::ostringstream hex;
        hex << std::hex << std::hash<std::string>{}(xml);
        return hex.str();
    }

    static std::string safe(const std::string& name) {
        std::string out;
        for (const char c : name) out.push_back(std::isalnum(static_cast<unsigned char>(c)) ? c : '-');
        return out.empty() ? "scene" : out;
    }

    // Depth-first: a body may be nested inside another (an arm link, a tool).
    static tinyxml2::XMLElement* find_body(tinyxml2::XMLElement* parent, const std::string& name) {
        for (auto* e = parent->FirstChildElement("body"); e != nullptr;
             e = e->NextSiblingElement("body")) {
            const char* named = e->Attribute("name");
            if (named != nullptr && name == named) return e;
            if (auto* deeper = find_body(e, name)) return deeper;
        }
        return nullptr;
    }

    // An override naming a body the base does not have is an error, not a
    // silent no-op: a scene that quietly ignores half of what it says is worse
    // than one that refuses to load.
    static Status apply_overrides(const SceneDescriptor& scene, tinyxml2::XMLElement* worldbody) {
        for (const auto& o : scene.overrides) {
            auto* body = find_body(worldbody, o.name);
            if (body == nullptr) {
                return Status::failure("scene '" + scene.name + "': overrides '" + o.name +
                                       "', which the base world does not declare");
            }
            if (o.visible && !*o.visible) {
                body->Parent()->DeleteChild(body);
                continue;  // everything else about a body that is gone is moot
            }
            if (o.pos) {
                std::ostringstream pos;
                pos << (*o.pos)[0] << " " << (*o.pos)[1] << " " << (*o.pos)[2];
                body->SetAttribute("pos", pos.str().c_str());
            }
            if (auto* geom = body->FirstChildElement("geom"); geom != nullptr) {
                if (o.material) geom->SetAttribute("material", o.material->c_str());
                if (o.mass) geom->SetAttribute("mass", *o.mass);
            }
        }
        return Status::success();
    }

    static Status add_objects(const SceneDescriptor& scene, tinyxml2::XMLDocument& doc,
                              tinyxml2::XMLElement* worldbody) {
        if (scene.objects.empty()) return Status::success();
        const std::string fragment = "<worldbody>" + bodies_of(scene) + "</worldbody>";
        tinyxml2::XMLDocument added;
        if (added.Parse(fragment.c_str()) != tinyxml2::XML_SUCCESS) {
            return Status::failure("scene '" + scene.name + "': cannot compose its objects");
        }
        for (auto* e = added.RootElement()->FirstChildElement(); e != nullptr;
             e = e->NextSiblingElement()) {
            worldbody->InsertEndChild(e->DeepClone(&doc));
        }
        return Status::success();
    }

    static std::string bodies_of(const SceneDescriptor& scene) {
        std::ostringstream xml;
        for (const auto& o : scene.objects) {
            if (!o.visible) continue;
            xml << "    <body name=\"" << o.name << "\" pos=\"" << o.pos[0] << " " << o.pos[1] << " "
                << o.pos[2] << "\">\n";
            if (!o.slide_axis.empty()) {
                xml << "      <joint name=\"" << o.name << "_slide\" type=\"slide\" axis=\""
                    << o.slide_axis << "\" range=\"" << o.slide_range[0] << " " << o.slide_range[1]
                    << "\" damping=\"0.2\"/>\n";
            }
            xml << "      <geom type=\"" << o.type << "\" size=\"" << o.size[0] << " " << o.size[1]
                << " " << o.size[2] << "\" material=\"" << o.material << "\" mass=\"" << o.mass
                << "\"";
            // Contact groups match the base world: things live in group 4, so the
            // robot (group 1) is stopped by what a user places in its way.
            xml << (o.collides ? " contype=\"4\" conaffinity=\"15\""
                               : " contype=\"0\" conaffinity=\"0\"");
            xml << "/>\n";
            xml << "      <site name=\"" << o.name << "\" pos=\"0 0 0\" size=\"0.006\"/>\n";
            xml << "    </body>\n";
        }
        return xml.str();
    }

    std::string worlds_, cache_;
};

}  // namespace robonode
