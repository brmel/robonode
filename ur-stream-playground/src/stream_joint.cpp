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

    // Read current joints, plan a +0.3 rad jerk-limited move on the wrist.
    auto pkg = driver->getDataPackage();
    if (!pkg) {
        std::fprintf(stderr, "no RTDE data — is the robot powered on?\n");
        return 1;
    }
    urcl::vector6d_t q{};
    pkg->getData("actual_q", q);

    const trajlib::SCurveProfile wrist{
        q[5], q[5] + 0.3, {.max_velocity = 0.5, .max_acceleration = 2.0, .max_jerk = 20.0}};

    // Stream @ 500 Hz with absolute deadlines (see trajectory-lab stream_demo).
    const auto t0 = std::chrono::steady_clock::now();
    auto deadline = t0;
    constexpr auto kTick = std::chrono::milliseconds{2};

    for (double t = 0.0; t <= wrist.duration(); t += 2e-3) {
        deadline += kTick;
        std::this_thread::sleep_until(deadline);

        urcl::vector6d_t target = q;
        target[5] = wrist.sample(t).position;
        driver->writeJointCommand(target, urcl::comm::ControlMode::MODE_SERVOJ,
                                  urcl::RobotReceiveTimeout::millisec(20));
    }
    driver->stopControl();
    std::printf("move complete: wrist %.4f -> %.4f rad over %.2f s\n", q[5], q[5] + 0.3,
                wrist.duration());
    return 0;
}
