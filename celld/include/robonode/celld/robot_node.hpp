#pragma once

#include <string>
#include <vector>

#include "robonode/celld/cell.hpp"
#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// A robot as a node: the whole arm addressed by id, joints internal. Binds axes
// by NAME, so nothing above indexes the flat node list.
class RobotNode {
public:
    // `cell` gives the axis order, `desc` the unit scale of each axis. The
    // robot speaks MODEL units (m, rad); the cell speaks descriptor units
    // (mm for a rail). This class is the only place that conversion lives.
    static Status bind(const RobotSpec& spec, const Cell& cell, const CellDescriptor& desc,
                       double carrier_weight, RobotNode& out) {
        RobotNode r;
        r.id_ = spec.id;
        r.tcp_site_ = spec.tcp_site;
        if (spec.joints.empty()) return Status::failure(spec.id + ": no joints");
        // The chain the kinematics solves over is carrier first, then arm: a
        // 7th axis is part of the reach, not a separate machine.
        for (const auto& id : spec.carrier) r.chain_ids_.push_back(id);
        for (const auto& id : spec.joints) r.chain_ids_.push_back(id);
        if (const auto st = resolve(r.chain_ids_, spec.id, cell, r.joints_); !st.ok()) return st;
        for (const auto& id : r.chain_ids_) r.per_m_.push_back(scale_of(desc, id));
        for (std::size_t k = 0; k < r.chain_ids_.size(); ++k) {
            r.weights_.push_back(k < spec.carrier.size() ? carrier_weight : 1.0);
        }
        out = std::move(r);
        return Status::success();
    }

    [[nodiscard]] const std::string& id() const { return id_; }
    [[nodiscard]] const std::string& tcp_site() const { return tcp_site_; }
    [[nodiscard]] std::size_t dof() const { return joints_.size(); }

    [[nodiscard]] std::vector<double> joint_positions(const Cell& cell) const {
        std::vector<double> q;
        q.reserve(joints_.size());
        for (std::size_t k = 0; k < joints_.size(); ++k) {
            q.push_back(cell.nodes()[joints_[k]].adapter->read().position / per_m_[k]);
        }
        return q;
    }

    [[nodiscard]] const std::vector<std::size_t>& joint_slots() const { return joints_; }
    [[nodiscard]] const std::vector<double>& joint_scales() const { return per_m_; }
    [[nodiscard]] const std::vector<double>& joint_weights() const { return weights_; }
    [[nodiscard]] const std::vector<std::string>& chain_ids() const { return chain_ids_; }

    // Joint waypoints → full-cell waypoint lists, every other axis held.
    Status compose(const Cell& cell, const std::vector<std::vector<double>>& joint_waypoints,
                   std::vector<std::vector<double>>& cell_waypoints) const {
        if (joint_waypoints.size() != joints_.size()) return Status::failure(id_ + ": dof mismatch");
        const std::size_t steps = joint_waypoints.empty() ? 0 : joint_waypoints.front().size();
        if (steps == 0) return Status::failure(id_ + ": empty trajectory");

        hold_all(cell, steps, cell_waypoints);
        for (std::size_t k = 0; k < joints_.size(); ++k) {
            if (joint_waypoints[k].size() != steps) return Status::failure(id_ + ": ragged");
            auto& out = cell_waypoints[joints_[k]];
            out.clear();
            for (const double q : joint_waypoints[k]) out.push_back(q * per_m_[k]);
        }
        return Status::success();
    }

private:
    static constexpr std::size_t kNone = static_cast<std::size_t>(-1);

    static std::size_t index_of(const Cell& cell, const std::string& id) {
        const auto& nodes = cell.nodes();
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i].id == id) return i;
        }
        return kNone;
    }

    static Status resolve(const std::vector<std::string>& ids, const std::string& owner,
                          const Cell& cell, std::vector<std::size_t>& out) {
        for (const auto& id : ids) {
            const auto i = index_of(cell, id);
            if (i == kNone) return Status::failure(owner + ": no axis '" + id + "'");
            out.push_back(i);
        }
        return Status::success();
    }

    static double scale_of(const CellDescriptor& desc, const std::string& id) {
        for (const auto& n : desc.nodes) {
            if (n.id == id) return n.units_per_m == 0.0 ? 1.0 : n.units_per_m;
        }
        return 1.0;
    }

    static void hold_all(const Cell& cell, std::size_t steps,
                         std::vector<std::vector<double>>& out) {
        const auto& nodes = cell.nodes();
        out.assign(nodes.size(), {});
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            out[i].assign(steps, nodes[i].adapter->read().position);
        }
    }

    std::string id_, tcp_site_{"tcp"};
    std::vector<std::size_t> joints_;
    std::vector<double> per_m_, weights_;
    std::vector<std::string> chain_ids_;
};

}  // namespace robonode
