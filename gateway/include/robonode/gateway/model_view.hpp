#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/sim_mujoco/mujoco_world.hpp"

namespace robonode {

// The kinematic chain, served from the model the physics actually runs. A
// viewer that rebuilds the robot from this cannot drift from the simulation —
// which is what happens the moment a link table is transcribed by hand.
// `workpiece_site` names the frame the platform reports dynamically (the part
// vision tracks). A viewer skips that body's geometry and draws the workpiece
// from telemetry instead, so a carried part is not also shown sitting still.
inline std::string model_json(const std::string& world_path, const std::string& tcp_site,
                              const std::string& workpiece_site) {
    std::shared_ptr<MujocoWorld> world;
    if (const auto st = MujocoWorld::load(world_path, world); !st.ok()) {
        return nlohmann::json{{"error", st.message()}}.dump();
    }
    auto links = nlohmann::json::array();
    for (const auto& l : world->links()) {
        auto geoms = nlohmann::json::array();
        for (const auto& g : l.geoms) {
            geoms.push_back({{"type", g.type},
                             {"group", g.group},
                             {"mesh", g.mesh},
                             {"material", g.material},
                             {"size", {g.size[0], g.size[1], g.size[2]}},
                             {"pos", {g.pos[0], g.pos[1], g.pos[2]}},
                             {"quat", {g.quat[0], g.quat[1], g.quat[2], g.quat[3]}}});
        }
        links.push_back({{"name", l.name},
                         {"parent", l.parent},
                         {"joint", l.joint},
                         {"joint_type", l.joint_type},
                         {"pos", {l.pos[0], l.pos[1], l.pos[2]}},
                         {"quat", {l.quat[0], l.quat[1], l.quat[2], l.quat[3]}},
                         {"axis", {l.axis[0], l.axis[1], l.axis[2]}},
                         {"geoms", geoms}});
    }
    auto sites = nlohmann::json::array();
    for (const auto& st : world->sites()) {
        sites.push_back({{"name", st.name},
                         {"body", st.body},
                         {"pos", {st.pos[0], st.pos[1], st.pos[2]}}});
    }
    return nlohmann::json{{"up", "z"},
                         {"tcp_site", tcp_site},
                         {"workpiece_site", workpiece_site},
                         {"links", links},
                         {"sites", sites}}
        .dump();
}

}  // namespace robonode
