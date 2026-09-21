#pragma once

#include <cstdint>
#include <string>

#include "robonode/vision/camera.hpp"

namespace robonode {

// A frame a browser can render, with no image library and no encoder: BMP is
// the one format that is literally the pixels a camera already gives us (24-bit
// BGR, bottom-up rows) behind a 54-byte header. Serving a *frame* matters —
// a user tuning a detector has to see what the detector saw, not a number that
// claims something was found.
//
// Uncompressed on purpose: this is a dev-loop feed on a local socket, and a PNG
// encoder would be a dependency bought for nothing.
inline std::string bmp_of(const Frame& f) {
    if (f.empty()) return {};
    const int stride = (f.width * 3 + 3) & ~3;  // BMP rows are 4-byte aligned
    const std::uint32_t pixels = static_cast<std::uint32_t>(stride * f.height);
    const std::uint32_t size = 54 + pixels;

    std::string out;
    out.reserve(size);
    const auto u16 = [&out](std::uint16_t v) {
        out.push_back(static_cast<char>(v & 0xFF));
        out.push_back(static_cast<char>(v >> 8));
    };
    const auto u32 = [&out](std::uint32_t v) {
        for (int i = 0; i < 4; ++i) out.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
    };

    out += "BM";
    u32(size);
    u32(0);
    u32(54);
    u32(40);  // BITMAPINFOHEADER
    u32(static_cast<std::uint32_t>(f.width));
    u32(static_cast<std::uint32_t>(f.height));
    u16(1);
    u16(24);
    u32(0);  // BI_RGB
    u32(pixels);
    u32(2835);  // ~72 dpi
    u32(2835);
    u32(0);
    u32(0);

    // BMP rows run bottom-up; the frame's run top-down.
    for (int y = f.height - 1; y >= 0; --y) {
        const auto row = static_cast<std::size_t>(y) * static_cast<std::size_t>(f.width) * 3;
        out.append(reinterpret_cast<const char*>(f.bgr.data() + row),
                   static_cast<std::size_t>(f.width) * 3);
        out.append(static_cast<std::size_t>(stride - f.width * 3), '\0');
    }
    return out;
}

// Draw where the platform believes the part is, in the frame the detector read.
// A crosshair the operator can compare against the blob is the whole point: a
// detector that is confidently wrong looks exactly like one that is right until
// you can see both.
inline void mark(Frame& f, double u, double v, std::uint8_t b = 0, std::uint8_t g = 255,
                 std::uint8_t r = 255, int arm = 8) {
    const auto plot = [&f, b, g, r](int x, int y) {
        if (x < 0 || y < 0 || x >= f.width || y >= f.height) return;
        const auto at = (static_cast<std::size_t>(y) * static_cast<std::size_t>(f.width) +
                         static_cast<std::size_t>(x)) * 3;
        f.bgr[at] = b;
        f.bgr[at + 1] = g;
        f.bgr[at + 2] = r;
    };
    const int cx = static_cast<int>(u), cy = static_cast<int>(v);
    for (int d = -arm; d <= arm; ++d) {
        plot(cx + d, cy);
        plot(cx, cy + d);
    }
}

}  // namespace robonode
