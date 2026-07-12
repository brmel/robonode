// M1 — connect to URSim's RTDE interface and print joint state @ 500 Hz.
//
//   ./read_state [robot_ip=127.0.0.1]
//
// RTDE (Real-Time Data Exchange) is UR's 500 Hz state/IO channel on port
// 30004: you negotiate a recipe of output fields, then receive a packet
// every 2 ms. This is the "read" half of every UR integration.

#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/log.h>

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

    for (int i = 0; i < 2500; ++i) {  // ~5 s at 500 Hz
        const auto pkg = client.getDataPackage(std::chrono::milliseconds{100});
        if (!pkg) continue;

        urcl::vector6d_t q{};
        pkg->getData("actual_q", q);
        if (i % 250 == 0) {  // print twice a second
            std::printf("q = [%7.4f %7.4f %7.4f %7.4f %7.4f %7.4f]\n",
                        q[0], q[1], q[2], q[3], q[4], q[5]);
        }
    }
    return 0;
}
