#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <string>

#include <mujoco/mujoco.h>

#include "robonode/core/geometry.hpp"
#include "robonode/core/state.hpp"
#include "robonode/core/status.hpp"
#include "robonode/motion/steppable.hpp"

namespace robonode {

// Owns one MuJoCo model + data — the physics substrate a cell's axis/joint
// adapters read and drive. One world may back many adapters (an arm's joints
// share it); it is a CycleSteppable the executive ticks exactly once per
// control cycle (SyncExecutive phases write→step→read so every ctrl is set
// before physics advances). Stepping is the executive's job, not any single
// adapter's — so swapping any node leaves the world advancing (#50).
//
// mujoco.h is private to this module (boundary lint): nothing above
// sim-mujoco sees a MuJoCo type.
class MujocoWorld : public CycleSteppable {
public:
    // Loads an MJCF world from disk. Failure Status carries MuJoCo's parser
    // error verbatim.
    static Status load(const std::string& xml_path, std::shared_ptr<MujocoWorld>& out) {
        char err[1024] = {0};
        mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
        if (m == nullptr) {
            return Status::failure("mujoco load '" + xml_path + "': " + err);
        }
        out = std::shared_ptr<MujocoWorld>(new MujocoWorld(m));
        return Status::success();
    }

    ~MujocoWorld() {
        if (data_ != nullptr) mj_deleteData(data_);
        if (model_ != nullptr) mj_deleteModel(model_);
    }
    MujocoWorld(const MujocoWorld&) = delete;
    MujocoWorld& operator=(const MujocoWorld&) = delete;

    // Name lookups return -1 when absent (adapters fault-check in configure).
    [[nodiscard]] int joint_id(const std::string& name) const {
        return mj_name2id(model_, mjOBJ_JOINT, name.c_str());
    }
    [[nodiscard]] int actuator_id(const std::string& name) const {
        return mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());
    }
    [[nodiscard]] int qpos_adr(int joint_id) const { return model_->jnt_qposadr[joint_id]; }
    [[nodiscard]] int qvel_adr(int joint_id) const { return model_->jnt_dofadr[joint_id]; }

    void set_ctrl(int actuator, double value) noexcept { data_->ctrl[actuator] = value; }
    [[nodiscard]] double qpos(int adr) const noexcept { return data_->qpos[adr]; }
    [[nodiscard]] double qvel(int adr) const noexcept { return data_->qvel[adr]; }

    // Every named frame's world position, as the last step left it. Reading
    // costs nothing and writes nothing — which is what lets one thread own the
    // physics while every other thread reads a copy.
    [[nodiscard]] std::vector<std::pair<std::string, Vec3>> site_world_poses() const {
        std::vector<std::pair<std::string, Vec3>> out;
        out.reserve(static_cast<std::size_t>(model_->nsite));
        for (int i = 0; i < model_->nsite; ++i) {
            out.emplace_back(name_of(mjOBJ_SITE, i), site_xpos(i));
        }
        return out;
    }

    // Which bodies are touching, right now. Contacts are how a cell explains
    // itself: "the move stopped short" and "the arm is leaning on the belt"
    // are the same sentence without them.
    [[nodiscard]] std::vector<std::pair<std::string, std::string>> contacts() const {
        std::vector<std::pair<std::string, std::string>> out;
        for (int c = 0; c < data_->ncon; ++c) {
            const auto& con = data_->contact[c];
            auto a = name_of(mjOBJ_BODY, model_->geom_bodyid[con.geom[0]]);
            auto b = name_of(mjOBJ_BODY, model_->geom_bodyid[con.geom[1]]);
            if (a > b) std::swap(a, b);
            auto pair = std::pair{std::move(a), std::move(b)};
            if (std::find(out.begin(), out.end(), pair) == out.end()) out.push_back(std::move(pair));
        }
        return out;
    }

    // Is anything touching anything, right now? The collision query a planner
    // needs is cheaper than the pairs: it only asks whether the count is zero.
    [[nodiscard]] int contact_count() const noexcept { return data_->ncon; }

    // Put a free body at a pose, at rest. A line that delivers a fresh part
    // does this once; everything after is contacts and friction.
    Status place_body(const std::string& body, const Vec3& pos) {
        const int b = mj_name2id(model_, mjOBJ_BODY, body.c_str());
        if (b < 0) return Status::failure("no body '" + body + "'");
        const int j = model_->body_jntadr[b];
        if (model_->body_jntnum[b] != 1 || j < 0 || model_->jnt_type[j] != mjJNT_FREE) {
            return Status::failure("body '" + body + "' is not free to be placed");
        }
        const int q = model_->jnt_qposadr[j], v = model_->jnt_dofadr[j];
        const double xyz[3] = {pos.x, pos.y, pos.z};
        for (int k = 0; k < 3; ++k) data_->qpos[q + k] = xyz[k];
        data_->qpos[q + 3] = 1.0;
        for (int k = 4; k < 7; ++k) data_->qpos[q + k] = 0.0;
        for (int k = 0; k < 6; ++k) data_->qvel[v + k] = 0.0;
        mj_forward(model_, data_);
        return Status::success();
    }

    // Close a declared equality (a gripper's weld) on the pose the bodies are
    // in RIGHT NOW — the tool holds the part where it caught it, not where the
    // model was authored. Opening it drops what was held.
    Status constrain(const std::string& name, Grip grip) {
        const int e = mj_name2id(model_, mjOBJ_EQUALITY, name.c_str());
        if (e < 0) return Status::failure("no constraint '" + name + "'");
        const bool closed = grip == Grip::kClosed;
        if (closed) capture_weld(e);
        data_->eq_active[e] = closed ? 1 : 0;
        mj_forward(model_, data_);
        return Status::success();
    }

    // Advance physics by dt_s, accumulating a residual so sim time tracks the
    // control clock regardless of the mj timestep / cycle-rate ratio. This is
    // the CycleSteppable hook the executive calls once per cycle.
    void tick(double dt_s) noexcept override {
        accum_ += dt_s;
        const double ts = model_->opt.timestep;
        while (accum_ >= ts - 1e-12) {
            mj_step(model_, data_);
            accum_ -= ts;
        }
    }

    [[nodiscard]] double timestep() const noexcept { return model_->opt.timestep; }

    // One link of the model's body tree: enough for a viewer to rebuild the
    // exact chain the physics uses, instead of transcribing it by hand.
    // One drawable on a link: a mesh, or a primitive with its half-extents.
    struct Geom {
        std::string type, mesh, material;
        int group{0};  // MuJoCo's visibility group; 3+ is collision geometry
        double size[3]{};
        double pos[3]{};
        double quat[4]{1, 0, 0, 0};
    };

    struct Link {
        std::string name, joint, joint_type;
        int parent{-1};
        double pos[3]{};
        double quat[4]{};   // MuJoCo order: w x y z
        double axis[3]{};   // joint axis; zero when the body has no joint
        std::vector<Geom> geoms;
    };

    [[nodiscard]] std::vector<Link> links() const {
        std::vector<Link> out;
        out.reserve(static_cast<std::size_t>(model_->nbody));
        for (int b = 0; b < model_->nbody; ++b) {
            Link l;
            l.name = name_of(mjOBJ_BODY, b);
            l.parent = model_->body_parentid[b];
            for (int k = 0; k < 3; ++k) l.pos[k] = model_->body_pos[b * 3 + k];
            for (int k = 0; k < 4; ++k) l.quat[k] = model_->body_quat[b * 4 + k];
            if (model_->body_jntnum[b] > 0) {
                const int j = model_->body_jntadr[b];
                l.joint = name_of(mjOBJ_JOINT, j);
                l.joint_type = joint_type_of(model_->jnt_type[j]);
                for (int k = 0; k < 3; ++k) l.axis[k] = model_->jnt_axis[j * 3 + k];
            }
            collect_geoms(b, l);
            out.push_back(std::move(l));
        }
        return out;
    }

    // A named frame on a link — where a tool point actually is, rather than
    // wherever the last body in the file happens to be.
    struct Site {
        std::string name;
        int body{-1};
        double pos[3]{};
    };

    [[nodiscard]] std::vector<Site> sites() const {
        std::vector<Site> out;
        out.reserve(static_cast<std::size_t>(model_->nsite));
        for (int i = 0; i < model_->nsite; ++i) {
            Site s;
            s.name = name_of(mjOBJ_SITE, i);
            s.body = model_->site_bodyid[i];
            for (int k = 0; k < 3; ++k) s.pos[k] = model_->site_pos[i * 3 + k];
            out.push_back(std::move(s));
        }
        return out;
    }

    [[nodiscard]] int nv() const noexcept { return model_->nv; }
    [[nodiscard]] int site_id(const std::string& name) const {
        return mj_name2id(model_, mjOBJ_SITE, name.c_str());
    }
    void set_qpos(int adr, double v) noexcept { data_->qpos[adr] = v; }
    void forward() noexcept { mj_forward(model_, data_); }  // FK + Jacobians, no integration
    [[nodiscard]] Vec3 site_xpos(int site) const noexcept {
        return {data_->site_xpos[3 * site], data_->site_xpos[3 * site + 1],
                data_->site_xpos[3 * site + 2]};
    }
    // Jacobians of a site, 3×nv row-major each (caller-sized). Either may be
    // null: orientation costs nothing to skip when a caller only reaches.
    void jac_site(int site, double* jacp, double* jacr = nullptr) noexcept {
        mj_jacSite(model_, data_, jacp, jacr, site);
    }

    // The site's orientation as a unit quaternion, from its rotation matrix.
    [[nodiscard]] Quat site_xquat(int site) const noexcept {
        double q[4];
        mju_mat2Quat(q, data_->site_xmat + 9 * site);
        return Quat{q[0], q[1], q[2], q[3]}.normalized();
    }

private:
    // Weld data layout: [anchor in body2, anchor in body1, relative quat,
    // torque scale]. Filling it from the live pose is what makes the grip
    // "where it is" instead of "where the file says".
    void capture_weld(int eq) {
        if (model_->eq_type[eq] != mjEQ_WELD) return;
        const int b1 = model_->eq_obj1id[eq], b2 = model_->eq_obj2id[eq];
        mjtNum* d = model_->eq_data + mjNEQDATA * eq;
        mjtNum offset[3], world_offset[3];
        for (int k = 0; k < 3; ++k) {
            world_offset[k] = data_->xpos[3 * b2 + k] - data_->xpos[3 * b1 + k];
            d[k] = 0.0;  // anchor at body2's origin
        }
        mju_mulMatTVec3(offset, data_->xmat + 9 * b1, world_offset);
        for (int k = 0; k < 3; ++k) d[3 + k] = offset[k];
        mjtNum inv1[4];
        mju_negQuat(inv1, data_->xquat + 4 * b1);
        mju_mulQuat(d + 6, inv1, data_->xquat + 4 * b2);
        d[10] = 1.0;
    }

    // A viewer must know whether a joint rotates or slides to pose the link.
    static const char* joint_type_of(int jnt_type) {
        switch (jnt_type) {
            case mjJNT_HINGE: return "hinge";
            case mjJNT_SLIDE: return "slide";
            case mjJNT_BALL: return "ball";
            case mjJNT_FREE: return "free";
            default: break;
        }
        return "";
    }

    [[nodiscard]] std::string name_of(mjtObj type, int id) const {
        const char* n = mj_id2name(model_, type, id);
        return n == nullptr ? std::string{} : std::string{n};
    }

    static const char* geom_type_of(int type) {
        switch (type) {
            case mjGEOM_PLANE: return "plane";
            case mjGEOM_SPHERE: return "sphere";
            case mjGEOM_CAPSULE: return "capsule";
            case mjGEOM_CYLINDER: return "cylinder";
            case mjGEOM_BOX: return "box";
            case mjGEOM_MESH: return "mesh";
            default: break;
        }
        return "";
    }

    // Everything drawable on a link — meshes AND primitives, with the material
    // name so a viewer can map the model's palette onto its own theme.
    void collect_geoms(int body, Link& link) const {
        for (int g = model_->body_geomadr[body];
             g < model_->body_geomadr[body] + model_->body_geomnum[body]; ++g) {
            Geom geom;
            geom.type = geom_type_of(model_->geom_type[g]);
            if (geom.type.empty()) continue;
            geom.group = model_->geom_group[g];
            if (model_->geom_type[g] == mjGEOM_MESH) {
                geom.mesh = name_of(mjOBJ_MESH, model_->geom_dataid[g]);
            }
            if (model_->geom_matid[g] >= 0) {
                geom.material = name_of(mjOBJ_MATERIAL, model_->geom_matid[g]);
            }
            for (int k = 0; k < 3; ++k) {
                geom.size[k] = model_->geom_size[g * 3 + k];
                geom.pos[k] = model_->geom_pos[g * 3 + k];
            }
            for (int k = 0; k < 4; ++k) geom.quat[k] = model_->geom_quat[g * 4 + k];
            link.geoms.push_back(std::move(geom));
        }
    }

    // Wake in the model's "home" keyframe when it defines one — it sets qpos AND
    // ctrl to a natural ready pose, so the servos hold that posture at rest (a
    // real robot at rest, not a vertical mast) and moves start from it.
    explicit MujocoWorld(mjModel* m) : model_{m}, data_{mj_makeData(m)} {
        if (model_->nkey > 0) mj_resetDataKeyframe(model_, data_, 0);
    }
    mjModel* model_;
    mjData* data_;
    double accum_{0.0};
};

// Shares one MujocoWorld across every adapter that names the same MJCF path,
// so an arm's joints (7 nodes, one `world` path) drive a single physics body.
// Every adapter reads/drives the shared world; the executive ticks it once
// per cycle (no adapter "owns" the clock — #50). Loaded lazily, once.
class MujocoWorldPool {
public:
    Status get(const std::string& path, std::shared_ptr<MujocoWorld>& out) {
        const auto it = worlds_.find(path);
        if (it != worlds_.end()) {
            out = it->second;
            return Status::success();
        }
        std::shared_ptr<MujocoWorld> w;
        if (const auto st = MujocoWorld::load(path, w); !st.ok()) return st;
        worlds_[path] = w;
        out = w;
        return Status::success();
    }

private:
    std::map<std::string, std::shared_ptr<MujocoWorld>> worlds_;
};

}  // namespace robonode
