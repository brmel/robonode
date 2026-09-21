// The OpenCV vision node: find the workpiece in PIXELS and report where it is
// in the world. Proves the camera model and the detector agree, which is the
// step that makes vision a real node instead of a scene oracle.

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "check.hpp"
#include "robonode/opencv/cv_detector.hpp"
#include "robonode/opencv/cv_vision.hpp"
#include "robonode/vision/scene_camera.hpp"

namespace {

// A camera already past its latency window, so `grab()` returns a frame.
std::unique_ptr<robonode::Camera> camera_watching(const robonode::Vec3& part,
                                                  std::vector<robonode::Vec3> clutter = {}) {
    robonode::CameraModel model;
    std::vector<robonode::SceneCamera::Target> targets{{"part"}};
    for (std::size_t i = 0; i < clutter.size(); ++i) {
        targets.push_back({"clutter" + std::to_string(i), 60, 60, 225, 0.10});  // same colour, wrong size
    }
    auto poses = [part, clutter](const std::string& site) {
        if (site == "part") return part;
        const auto index = static_cast<std::size_t>(std::stoul(site.substr(7)));
        return clutter[index];
    };
    // A clock that has already advanced past the shutter + latency.
    auto clock = [t = std::make_shared<double>(0.0)]() mutable { return (*t += 1.0); };
    return std::make_unique<robonode::SceneCamera>(model, poses, clock, std::move(targets));
}

// A camera has latency: the first look returns nothing, because nothing has
// been captured long enough ago to have been processed. Vision runs
// continuously, so a test must let a frame or two flow — that is the sensor
// behaving, not the test working around it.
std::vector<robonode::Detection> settled(robonode::Detector& detector) {
    std::vector<robonode::Detection> found;
    for (int i = 0; i < 3; ++i) found = detector.detect();
    return found;
}

// Latency is part of the contract: the first look sees nothing.
void test_first_look_sees_nothing_because_frames_take_time() {
    robonode::CvDetector detector{camera_watching({0.9, 0.25, 0.37}), {}, "part"};
    CHECK(detector.detect().empty());
}

void test_cv_detector_recovers_the_world_pose_from_pixels() {
    const robonode::Vec3 truth{0.9, 0.25, 0.37};
    robonode::CvDetector detector{camera_watching(truth), {}, "part"};

    const auto found = settled(detector);
    CHECK(found.size() == 1);
    CHECK(found.front().label == "part");
    // Back-projection is exact to within a pixel of quantisation.
    CHECK(std::abs(found.front().position.x - truth.x) < 0.01);
    CHECK(std::abs(found.front().position.y - truth.y) < 0.01);
}

// The detector must track the part along the belt, not just find it once.
void test_cv_detector_follows_the_part_along_the_belt() {
    double last_x = 0.0;
    for (const double x : {1.30, 1.10, 0.90, 0.70, 0.50}) {
        robonode::CvDetector detector{camera_watching({x, 0.25, 0.37}), {}, "part"};
        const auto found = settled(detector);
        CHECK(found.size() == 1);
        CHECK(std::abs(found.front().position.x - x) < 0.02);
        CHECK(last_x == 0.0 || found.front().position.x < last_x);  // moving one way
        last_x = found.front().position.x;
    }
}

// "Find the biggest red blob" is the naive answer. A larger object of the same
// colour defeats it; the part's known size does not lie.
void test_cv_detector_rejects_clutter_of_the_same_colour() {
    const robonode::Vec3 truth{0.9, 0.25, 0.37};
    robonode::CvDetector detector{camera_watching(truth, {{0.65, 0.25, 0.37}}), {}, "part"};

    const auto found = settled(detector);
    CHECK(found.size() == 1);
    CHECK(std::abs(found.front().position.x - truth.x) < 0.02);  // the part, not the decoy
}

// A sighting is stamped with the time the frame was CAPTURED. Without it,
// nothing downstream can tell how stale the observation is.
void test_detections_carry_the_capture_time() {
    robonode::CvDetector detector{camera_watching({0.9, 0.25, 0.37}), {}, "part"};
    const auto found = settled(detector);
    CHECK(found.size() == 1);
    CHECK(found.front().captured_s > 0.0);
    CHECK(found.front().confidence > 0.5);
}

// Nothing in frame is reported as nothing found — never as a pose at the origin.
void test_cv_detector_reports_nothing_when_the_belt_is_empty() {
    robonode::CvDetector detector{camera_watching({9.0, 9.0, 0.37}), {}, "part"};
    CHECK(settled(detector).empty());
}

// The sensor is its own node: a detector is handed one, so "change the camera"
// and "change the vision algorithm" are independent choices.
void test_camera_versions_are_selectable_and_differ() {
    robonode::CameraRegistry registry;
    robonode::register_scene_cameras(registry);
    CHECK(registry.names().size() >= 2);

    robonode::CameraContext ctx;
    ctx.site_pose = [](const std::string&) { return robonode::Vec3{0.9, 0.25, 0.37}; };
    ctx.clock = [t = std::make_shared<double>(0.0)]() mutable { return (*t += 0.05); };
    ctx.config = {{"watch", "part"}};

    std::unique_ptr<robonode::Camera> fast, slow;
    CHECK(registry.make("robonode.overhead", ctx, fast).ok());
    CHECK(registry.make("robonode.overhead-slow", ctx, slow).ok());

    // The same scene through two sensors: the slower one delivers later.
    int fast_frames = 0, slow_frames = 0;
    for (int i = 0; i < 20; ++i) {
        if (!fast->grab().empty()) ++fast_frames;
        if (!slow->grab().empty()) ++slow_frames;
    }
    CHECK(fast_frames > 0);
    CHECK(fast_frames > slow_frames);  // higher rate, lower latency
}

// A detector with no sensor selected reports that, rather than inventing one.
void test_detector_without_a_camera_is_refused() {
    robonode::VisionRegistry vision;
    robonode::register_cv_detector(vision);
    robonode::VisionContext ctx;  // no camera source
    std::unique_ptr<robonode::Detector> detector;
    CHECK(!vision.make("robonode.opencv", ctx, detector).ok());
}

}  // namespace

int main() {
    test_first_look_sees_nothing_because_frames_take_time();
    test_cv_detector_recovers_the_world_pose_from_pixels();
    test_cv_detector_follows_the_part_along_the_belt();
    test_cv_detector_rejects_clutter_of_the_same_colour();
    test_detections_carry_the_capture_time();
    test_cv_detector_reports_nothing_when_the_belt_is_empty();
    test_camera_versions_are_selectable_and_differ();
    test_detector_without_a_camera_is_refused();
    std::puts("robonode vision_cv: all tests passed");
    return 0;
}
