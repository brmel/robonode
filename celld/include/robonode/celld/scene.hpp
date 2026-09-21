#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/core/state.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// The live scene a cell runs in — the physical world, addressed by the names
// the model declares. This is what lets a station move a belt and a sensor read
// a frame WITHOUT anything above knowing which engine is underneath.
//
// It is deliberately narrow: read a named frame, drive a named actuator, place
// a named joint. Anything richer belongs behind its own seam.
class Scene {
public:
    virtual ~Scene() = default;

    // World position of a named site, or nothing when the model has no such name.
    [[nodiscard]] virtual std::optional<Vec3> site(const std::string& name) const = 0;

    // Command a named actuator (a belt drive, a fixture clamp).
    virtual Status drive(const std::string& actuator, double value) = 0;

    // Put a named joint at a value — how a station recycles a workpiece.
    virtual Status place(const std::string& joint, double value) = 0;

    // Put a whole body somewhere, at rest — how a line delivers a fresh part.
    virtual Status place_body(const std::string& body, const Vec3& pos) = 0;

    // Close or open a constraint the model declares. A tool that picks
    // something up holds it because the physics says so, not because the
    // platform remembers that it should.
    virtual Status constrain(const std::string& name, Grip grip) = 0;

    // Every named frame at once. A reader outside the thread that owns the
    // physics must take a copy, not a pointer into it.
    [[nodiscard]] virtual std::vector<std::pair<std::string, Vec3>> site_poses() const = 0;

    // The bodies touching right now, as name pairs. A cell that cannot say
    // what it is leaning on cannot explain why a move stopped short.
    [[nodiscard]] virtual std::vector<std::pair<std::string, std::string>> contacts() const = 0;

    // Advance the world by dt. A cell whose physics only runs while the robot
    // moves cannot have a conveyor: the belt would freeze between commands.
    virtual void advance(double dt_s) = 0;
};

}  // namespace robonode
