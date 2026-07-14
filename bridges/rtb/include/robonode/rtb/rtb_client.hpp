#pragma once

#include <stdexcept>
#include <string>

#include <httplib.h>
#include <nlohmann/json.hpp>

namespace robonode::rtb {

// Thin JSON-RPC client to the Python rtb-kinematics service (real robot
// models via Robotics Toolbox). Non-RT: planning/kinematics queries only.
// A fresh httplib::Client per call keeps the Kinematics/Planner impls
// const-correct (their query methods are const).
class RtbClient {
public:
    RtbClient(std::string host, int port) : host_{std::move(host)}, port_{port} {}

    [[nodiscard]] nlohmann::json post(const std::string& path, const nlohmann::json& body) const {
        httplib::Client cli{host_, port_};
        cli.set_connection_timeout(2, 0);
        cli.set_read_timeout(10, 0);
        auto res = cli.Post(path, body.dump(), "application/json");
        if (!res) {
            throw std::runtime_error("rtb service unreachable at " + host_ + ":" +
                                     std::to_string(port_) + path);
        }
        if (res->status != 200) {
            throw std::runtime_error("rtb service " + path + " HTTP " +
                                     std::to_string(res->status) + ": " + res->body);
        }
        return nlohmann::json::parse(res->body, nullptr, false);
    }

private:
    std::string host_;
    int port_;
};

}  // namespace robonode::rtb
