// M2 — stream a jerk-limited wrist move to URSim (skeleton).
//
//   ./stream_joint [robot_ip=127.0.0.1]
//
// Uses UrDriver's reverse interface: the External Control URCap runs a
// program on the robot that polls us for setpoints; we answer every 2 ms
// with the next sample of our own S-curve profile (SERVOJ control mode).
// External trajectory math, vendor servo loop — the standard industrial
// streaming split.
//
// Skeleton status: control-flow complete, but UrDriver's constructor wants
// script/recipe resource files shipped with ur_client_library — wire the
// paths below to the FetchContent build dir before first run (see the
// library's examples/full_driver.cpp, which this file follows).

#include <ur_client_library/ur/ur_driver.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <thread>

#include "trajlib/scurve.hpp"

int main(int argc, char** argv) {
    const std::string robot_ip = argc > 1 ? argv[1] : "127.0.0.1";

    // TODO(M2): point these at the files under build/_deps/ur_client_library-src/resources/
    const std::string script_file = "resources/external_control.urscript";
    const std::string output_recipe = "examples/resources/rtde_output_recipe.txt";
    const std::string input_recipe = "examples/resources/rtde_input_recipe.txt";

    std::unique_ptr<urcl::UrDriver> driver;
    try {
        driver = std::make_unique<urcl::UrDriver>(
            robot_ip, script_file, output_recipe, input_recipe,
            [](bool program_running) {
                std::printf("external-control program %s\n", program_running ? "running" : "stopped");
            },
            /*headless=*/true);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "driver init failed: %s\n", e.what());
        return 1;
    }

    // Pre-allocated data package, reused every cycle (allocating overload is
    // deprecated and has no place at 500 Hz anyway).
    urcl::rtde_interface::DataPackage pkg{driver->getRTDEOutputRecipe()};

    // Read current joints, plan a +0.3 rad jerk-limited move on the wrist.
    if (!driver->getDataPackage(pkg)) {
        std::fprintf(stderr, "no RTDE data — is the robot powered on?\n");
        return 1;
    }
    urcl::vector6d_t q{};
    if (!pkg.getData("actual_q", q)) {
        std::fprintf(stderr, "actual_q missing from RTDE recipe\n");
        return 1;
    }

    const trajlib::SCurveProfile wrist{
        q[5], q[5] + 0.3, {.max_velocity = 0.5, .max_acceleration = 2.0, .max_jerk = 20.0}};

    // Stream @ 500 Hz with absolute deadlines (see trajectory-lab stream_demo).
    const auto t0 = std::chrono::steady_clock::now();
    auto deadline = t0;
    constexpr auto kTick = std::chrono::milliseconds{2};

    for (double t = 0.0; t <= wrist.duration(); t += 2e-3) {
        deadline += kTick;
        std::this_thread::sleep_until(deadline);

        // Safety gate every cycle: never stream setpoints into a robot that
        // is not in NORMAL/REDUCED safety mode (UR: 1 = NORMAL, 2 = REDUCED;
        // 3+ = protective stop, recovery, e-stop, fault...). The recipe file
        // must include safety_mode for this to work — fail closed if absent.
        if (driver->getDataPackage(pkg)) {
            std::int32_t safety_mode = 0;
            if (!pkg.getData("safety_mode", safety_mode) || safety_mode > 2) {
                std::fprintf(stderr, "aborting stream: safety_mode=%d\n", safety_mode);
                driver->stopControl();
                return 1;
            }
        }

        urcl::vector6d_t target = q;
        target[5] = wrist.sample(t).position;
        if (!driver->writeJointCommand(target, urcl::comm::ControlMode::MODE_SERVOJ,
                                       urcl::RobotReceiveTimeout::millisec(20))) {
            std::fprintf(stderr, "aborting stream: writeJointCommand failed at t=%.3f\n", t);
            driver->stopControl();
            return 1;
        }
    }
    driver->stopControl();
    std::printf("move complete: wrist %.4f -> %.4f rad over %.2f s\n", q[5], q[5] + 0.3,
                wrist.duration());
    return 0;
}
