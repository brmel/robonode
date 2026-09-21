#pragma once

#include "robonode/core/status.hpp"

namespace robonode {

// The end-effector seam: what a robot node carries. A vacuum cup, a two-finger
// gripper, and a sim stand-in are interchangeable versions of this.
class ToolNode {
public:
    virtual ~ToolNode() = default;
    virtual Status grasp() = 0;
    virtual Status release() = 0;
    [[nodiscard]] virtual bool holding() const = 0;
};

}  // namespace robonode
