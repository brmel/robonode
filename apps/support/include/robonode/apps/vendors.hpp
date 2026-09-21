#pragma once

#include <string>

#include "robonode/gateway/cell_gateway.hpp"

#ifdef ROBONODE_WITH_UR
#include "robonode/adapter_ur/ur_wrist_adapter.hpp"
#endif
#ifdef ROBONODE_WITH_OPENCV
#include "robonode/opencv/cv_vision.hpp"
#endif

namespace robonode {

// The vendor drivers this build supports. Composition roots call this; no
// library does, which is why it lives with the apps and not in the gateway.
// A build without the hardware simply offers fewer versions — the seam, the
// descriptor and every surface are unchanged.
inline CellGateway::DriverHook vendor_drivers() {
#ifdef ROBONODE_WITH_UR
    return [](DriverRegistry& registry) { register_ur_wrist(registry, ROBONODE_UR_RESOURCES); };
#else
    return {};
#endif
}

// The perception engines this build links. A build without OpenCV simply offers
// one fewer vision version.
inline CellGateway::VisionHook vision_versions() {
#ifdef ROBONODE_WITH_OPENCV
    return [](VisionRegistry& registry) { register_cv_detector(registry); };
#else
    return {};
#endif
}

}  // namespace robonode
