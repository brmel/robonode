#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "robonode/celld/cell.hpp"
#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/celld/robot_node.hpp"
#include "robonode/celld/scene_descriptor.hpp"
#include "robonode/sim_mujoco/mjcf_composer.hpp"
#include "robonode/core/status.hpp"
#include "robonode/motion/cartesian.hpp"
#include "robonode/log.hpp"

namespace robonode {

// Owns the live cell and the robots bound to it. The mutex never leaves this
// class: callers reach the cell through with_cell(), which fails closed when no
// cell is built.
class CellRuntime {
public:
    struct NodeView {
        std::string id, driver, unit, joint;
        double lo, hi;
    };

    // `worlds_dir` is where scenes live; which one this cell uses is the
    // descriptor's business, so a new scenario is a new descriptor.
    CellRuntime(std::string worlds_dir, const DriverRegistry& registry)
        : worlds_dir_{std::move(worlds_dir)}, registry_{registry} {}

    // The cell names a world: either a physics model, or a scene descriptor
    // that starts from one and adds the user's objects. Composing is what makes
    // a scenario editable without touching XML.
    Status load(const std::string& path) {
        if (const auto st = load_cell_descriptor(path, desc_); !st.ok()) return st;
        if (!is_scene(desc_.world)) {
            world_ = (std::filesystem::path{worlds_dir_} / desc_.world).string();
            return Status::success();
        }
        SceneDescriptor scene;
        const auto file = (std::filesystem::path{scenes_dir_.empty() ? worlds_dir_ : scenes_dir_} /
                           desc_.world)
                              .string();
        if (const auto st = load_scene_descriptor(file, scene); !st.ok()) return st;
        return MjcfComposer{worlds_dir_, worlds_dir_}.compose(scene, world_);
    }

    // Swap the scenario from the descriptor itself. The caller resolved which
    // library or session the scene came from, so the runtime needs only the
    // data; a bad scene leaves the running world untouched.
    Status set_scene(const std::string& json, const std::string& label) {
        SceneDescriptor scene;
        if (const auto st =
                parse_scene_descriptor(nlohmann::json::parse(json, nullptr, false), scene, label);
            !st.ok()) {
            return st;
        }
        std::string composed;
        if (const auto st = MjcfComposer{worlds_dir_, worlds_dir_}.compose(scene, composed);
            !st.ok()) {
            return st;
        }
        world_ = std::move(composed);
        desc_.world = label;
        return Status::success();
    }

    void set_scenes_dir(std::string dir) { scenes_dir_ = std::move(dir); }

    [[nodiscard]] const std::string& world() const { return world_; }

    const CellDescriptor& descriptor() const { return desc_; }

    Status build(const std::string& family) {
        // Configuring and activating adapters TOUCHES the physics (they bind
        // joints and write initial commands), so a build must exclude anyone
        // else stepping the same world — the idle ticker, most of all. Holding
        // the cell lock for the whole build is what makes "swap the driver
        // family live" safe rather than lucky.
        std::lock_guard<std::mutex> lk{mtx_};
        auto cell = std::make_unique<Cell>(registry_);
        if (const auto st = add_nodes(*cell, family); !st.ok()) return refuse(st);
        if (const auto st = cell->configure_all(); !st.ok()) return refuse(st);
        if (const auto st = cell->activate_all(); !st.ok()) return refuse(st);

        std::vector<RobotNode> robots;
        for (const auto& spec : desc_.robots) {
            RobotNode r;
            if (const auto st = RobotNode::bind(spec, *cell, desc_, carrier_weight_, r);
                !st.ok()) {
                return refuse(st);
            }
            robots.push_back(std::move(r));
        }

        cell_ = std::move(cell);
        robots_ = std::move(robots);
        family_ = family;
        RN_LOG_INFO("cell built: {} nodes ({} family)", desc_.nodes.size(), family);
        return Status::success();
    }

    using CellFn = std::function<Status(Cell&)>;
    using RobotFn = std::function<Status(Cell&, const RobotNode&)>;

    Status with_cell(const CellFn& fn) {
        std::lock_guard<std::mutex> lk{mtx_};
        if (!cell_) return Status::failure("no cell built");
        return fn(*cell_);
    }

    Status with_robot(const std::string& id, const RobotFn& fn) {
        std::lock_guard<std::mutex> lk{mtx_};
        if (!cell_) return Status::failure("no cell built");
        const auto* r = find_robot(id);
        if (r == nullptr) return Status::failure("no robot '" + (id.empty() ? "<default>" : id) + "'");
        return fn(*cell_, *r);
    }

    std::string family() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return family_;
    }

    // The families this cell declares — what a surface may offer.
    std::vector<std::string> families() const {
        std::vector<std::string> out;
        out.reserve(desc_.families.size());
        for (const auto& f : desc_.families) out.push_back(f.id);
        return out;
    }

    std::vector<AxisLimits> limits() const {
        std::vector<AxisLimits> out;
        out.reserve(desc_.nodes.size());
        for (const auto& n : desc_.nodes) out.push_back(n.limits);
        return out;
    }

    std::vector<std::string> axis_units() const {
        std::vector<std::string> out;
        out.reserve(desc_.nodes.size());
        for (const auto& n : desc_.nodes) out.push_back(n.unit);
        return out;
    }

    std::vector<std::string> axis_ids() const {
        std::vector<std::string> out;
        out.reserve(desc_.nodes.size());
        for (const auto& n : desc_.nodes) out.push_back(n.id);
        return out;
    }

    // How a robot's joints map onto cell axes, and the scale between the
    // model's units and the descriptor's.
    struct JointMap {
        std::vector<std::size_t> slots;
        std::vector<double> per_m;
        std::vector<double> weights;
        std::vector<std::string> chain_ids;
    };

    JointMap joint_map(const std::string& id = {}) const {
        std::lock_guard<std::mutex> lk{mtx_};
        const auto* r = find_robot(id);
        if (r == nullptr) return {};
        return {r->joint_slots(), r->joint_scales(), r->joint_weights(), r->chain_ids()};
    }

    void set_carrier_weight(double w) { carrier_weight_ = w; }

    // The travel of one robot's joints in MODEL units, in joint order.
    std::vector<JointBound> joint_bounds(const std::string& id = {}) const {
        const auto map = joint_map(id);
        std::vector<JointBound> out;
        for (std::size_t k = 0; k < map.slots.size(); ++k) {
            const auto slot = map.slots[k];
            if (slot >= desc_.nodes.size()) continue;
            const auto& lim = desc_.nodes[slot].limits;
            out.push_back({lim.position_min / map.per_m[k], lim.position_max / map.per_m[k]});
        }
        return out;
    }

    std::vector<NodeView> node_views() const {
        std::lock_guard<std::mutex> lk{mtx_};
        std::vector<NodeView> out;
        if (!cell_) return out;
        const auto& nodes = cell_->nodes();
        for (std::size_t i = 0; i < nodes.size() && i < desc_.nodes.size(); ++i) {
            out.push_back({nodes[i].id, nodes[i].descriptor.driver, desc_.nodes[i].unit,
                           desc_.nodes[i].joint, desc_.nodes[i].limits.position_min,
                           desc_.nodes[i].limits.position_max});
        }
        return out;
    }

    std::vector<double> positions() const {
        std::lock_guard<std::mutex> lk{mtx_};
        std::vector<double> out;
        if (!cell_) return out;
        for (const auto& n : cell_->nodes()) out.push_back(n.adapter->read().position);
        return out;
    }

    // Which cell slots hold a robot's joints. Read once after a build; the
    // telemetry path then needs no lock (it runs inside with_cell).
    std::vector<std::size_t> joint_slots(const std::string& id = {}) const {
        std::lock_guard<std::mutex> lk{mtx_};
        const auto* r = find_robot(id);
        return r == nullptr ? std::vector<std::size_t>{} : r->joint_slots();
    }

    Status replace_node(const std::string& node, const std::string& driver) {
        return with_cell([&](Cell& c) { return c.replace_node(node, driver); });
    }

private:
    static bool is_scene(const std::string& world) {
        static constexpr std::string_view kSuffix{".scene.json"};
        return world.size() > kSuffix.size() &&
               world.compare(world.size() - kSuffix.size(), kSuffix.size(), kSuffix) == 0;
    }

    static Status refuse(const Status& why) {
        RN_LOG_ERROR("cell build failed: {} — previous cell left running", why.message());
        return why;
    }

    Status add_nodes(Cell& cell, const std::string& family) const {
        if (desc_.nodes.empty()) return Status::failure("descriptor has no nodes");
        const auto* spec = desc_.family(family);
        if (spec == nullptr) return Status::failure("unknown driver family '" + family + "'");
        const std::string& driver = spec->driver;
        for (const auto& s : desc_.nodes) {
            Descriptor d;
            d.id = s.id;
            d.driver = driver;
            d.limits = s.limits;
            d.command_rate_hz = 1000;
            d.config = {{"world", world_}, {"joint", s.joint}, {"actuator", s.actuator},
                        {"units_per_m", std::to_string(s.units_per_m)}};
            if (const auto st = cell.add_node(d); !st.ok()) return st;
        }
        return Status::success();
    }

    const RobotNode* find_robot(const std::string& id) const {
        for (const auto& r : robots_) {
            if (id.empty() || r.id() == id) return &r;
        }
        return nullptr;
    }

    std::string worlds_dir_, scenes_dir_, world_;
    const DriverRegistry& registry_;
    CellDescriptor desc_;
    mutable std::mutex mtx_;
    std::unique_ptr<Cell> cell_;
    std::vector<RobotNode> robots_;
    std::string family_{"physics"};
    double carrier_weight_{1.0};
};

}  // namespace robonode
