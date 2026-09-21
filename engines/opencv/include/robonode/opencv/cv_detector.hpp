#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// OpenCV 5 moved contour geometry out of imgproc; include both so this builds
// against 4.x and 5.x alike.
#include <opencv2/imgproc.hpp>
#if __has_include(<opencv2/geometry/2d.hpp>)
#include <opencv2/geometry/2d.hpp>
#endif

#include "robonode/vision/camera.hpp"
#include "robonode/vision/detector.hpp"

namespace robonode {

// A real vision node: it finds the workpiece in PIXELS. Colour threshold in
// HSV, largest contour, centroid, then back-projection through the camera model
// onto the work plane — the same pipeline a shop-floor cell runs, with OpenCV
// doing the image work rather than us reinventing it.
//
// Everything it needs is a Frame and a CameraModel, so it neither knows nor
// cares whether those come from a simulated camera or a real one.
class CvDetector final : public Detector {
public:
    struct Tuning {
        int hue_low{0}, hue_high{10};       // red wraps the hue circle; low half
        int hue_low2{170}, hue_high2{180};  // …and the high half
        int sat_min{120}, val_min{80};
        double min_area_px{25.0};
        double plane_z{0.37};     // the belt surface the part rides on
        double part_size_m{0.06}; // what we are looking for…
        double size_tolerance{0.45};  // …and how far off an area may be
    };

    CvDetector(std::unique_ptr<Camera> camera, Tuning tuning, std::string label)
        : camera_{std::move(camera)}, tuning_{tuning}, label_{std::move(label)} {}

    std::vector<Detection> detect() override {
        const Frame frame = camera_->grab();
        if (frame.empty()) return {};  // nothing has been captured long enough ago yet

        const cv::Mat bgr{frame.height, frame.width, CV_8UC3,
                          const_cast<std::uint8_t*>(frame.bgr.data())};
        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

        cv::Mat low, high, mask;
        cv::inRange(hsv, cv::Scalar(tuning_.hue_low, tuning_.sat_min, tuning_.val_min),
                    cv::Scalar(tuning_.hue_high, 255, 255), low);
        cv::inRange(hsv, cv::Scalar(tuning_.hue_low2, tuning_.sat_min, tuning_.val_min),
                    cv::Scalar(tuning_.hue_high2, 255, 255), high);
        cv::bitwise_or(low, high, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // "The biggest red blob" is the naive answer, and clutter defeats it.
        // The workpiece has a known size, and the camera model says how many
        // pixels that is at this distance — so an implausible blob is rejected
        // however red it is.
        const double expected = expected_area_px();
        const std::vector<cv::Point>* best = nullptr;
        double best_error = tuning_.size_tolerance;
        for (const auto& c : contours) {
            const double area = cv::contourArea(c);
            if (area < tuning_.min_area_px) continue;
            const double error = std::abs(area - expected) / expected;
            if (error < best_error) {
                best_error = error;
                best = &c;
            }
        }
        if (best == nullptr) return {};

        const cv::Moments m = cv::moments(*best);
        if (m.m00 <= 0.0) return {};
        const Vec3 world = camera_->model().unproject(m.m10 / m.m00, m.m01 / m.m00, tuning_.plane_z);
        // The sighting is stamped with the CAPTURE time, not now: everything
        // downstream has to know how old this is.
        return {{label_, world, frame.captured_s, 1.0 - best_error}};
    }

private:
    // How large the part should appear, from the camera model and the plane it
    // sits on — the projection maths run backwards.
    [[nodiscard]] double expected_area_px() const {
        const double dz = std::max(1e-3, camera_->model().position.z - tuning_.plane_z);
        const double side_px = tuning_.part_size_m * camera_->model().fx / dz;
        return side_px * side_px;
    }

    std::unique_ptr<Camera> camera_;
    Tuning tuning_;
    std::string label_;
};

}  // namespace robonode
