#pragma once

#include <algorithm>
#include <cstdlib>

#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/core/geometry.hpp"

namespace robonode {

// Station behaviour as domain logic (not data). A pallet fills a grid: `cols`
// columns spaced `dx`, advancing a row (spaced `dz`) each time a row fills, from
// origin (ox,oy,oz). Slot `index` (0-based) is placed row-major. Config keys
// carry the shop values; sensible defaults keep a bare pallet usable.
inline Vec3 pallet_slot(const StationSpec& s, int index) {
    const auto num = [&](const char* k, double def) {
        const auto it = s.config.find(k);
        return it == s.config.end() ? def : std::strtod(it->second.c_str(), nullptr);
    };
    const int cols = std::max(1, static_cast<int>(num("cols", 2)));
    const double ox = num("ox", 0.45), oy = num("oy", -0.3), oz = num("oz", 0.45);
    const double dx = num("dx", 0.15), dz = num("dz", 0.15);
    const int col = index % cols;
    const int row = index / cols;
    return {ox + col * dx, oy, oz + row * dz};
}

}  // namespace robonode
