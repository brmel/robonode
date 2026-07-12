// M1 — connect to URSim's RTDE interface and print joint state @ 500 Hz.
//
//   ./read_state [robot_ip=127.0.0.1]
//
// RTDE (Real-Time Data Exchange) is UR's 500 Hz state/IO channel on port
// 30004: you negotiate a recipe of output fields, then receive a packet
// every 2 ms. This is the "read" half of every UR integration.

#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/log.h>

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

int main(int argc, char** argv) {
    const std::string robot_ip = argc > 1 ? argv[1] : "127.0.0.1";

    // Output recipe: which fields the robot streams to us each cycle.
    const std::vector<std::string> outputs = {
        "timestamp", "actual_q", "actual_qd", "robot_mode", "safety_mode"};
    const std::vector<std::string> inputs = {};  // none needed for reading

    urcl::comm::INotifier notifier;
    urcl::rtde_interface::RTDEClient client{robot_ip, notifier, outputs, inputs};

    if (!client.init()) {
        std::fprintf(stderr, "RTDE init failed — is URSim up and powered on?\n");
        return 1;
    }
    client.start();
    std::printf("connected to %s, RTDE @ %.0f Hz\n", robot_ip.c_str(), client.getMaxFrequency());

    // Pre-allocated from the negotiated recipe; reused every cycle (the
    // unique_ptr-returning getDataPackage allocates per call and is
    // deprecated).
    urcl::rtde_interface::DataPackage pkg{client.getOutputRecipe()};

    std::int32_t last_safety = -1;
    for (int i = 0; i < 2500; ++i) {  // ~5 s at 500 Hz
        if (!client.getDataPackage(pkg, std::chrono::milliseconds{100})) continue;

        urcl::vector6d_t q{};
        if (!pkg.getData("actual_q", q)) {
            std::fprintf(stderr, "actual_q missing from negotiated recipe\n");
            return 1;
        }
        // Surface safety transitions — the reason the field is in the recipe.
        std::int32_t safety_mode = 0;
        if (pkg.getData("safety_mode", safety_mode) && safety_mode != last_safety) {
            std::printf("safety_mode -> %d (1=NORMAL 2=REDUCED 3+=stopped/fault)\n", safety_mode);
            last_safety = safety_mode;
        }
        if (i % 250 == 0) {  // print twice a second
            std::printf("q = [%7.4f %7.4f %7.4f %7.4f %7.4f %7.4f]\n",
                        q[0], q[1], q[2], q[3], q[4], q[5]);
        }
    }
    return 0;
}
