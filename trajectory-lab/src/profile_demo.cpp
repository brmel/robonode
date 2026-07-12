// Prints trapezoid vs S-curve side by side as CSV:
//   t,trap_pos,trap_vel,trap_acc,s_pos,s_vel,s_acc
// Plot: ./profile_demo > out.csv && python3 ../scripts/plot_profiles.py out.csv

#include <algorithm>
#include <cstdio>

#include "trajlib/scurve.hpp"
#include "trajlib/trapezoidal.hpp"

int main() {
    using namespace trajlib;

    const TrapezoidalProfile trap{0.0, 0.5, {.max_velocity = 0.25, .max_acceleration = 1.0}};
    const SCurveProfile scurve{
        0.0, 0.5, {.max_velocity = 0.25, .max_acceleration = 1.0, .max_jerk = 10.0}};

    std::printf("t,trap_pos,trap_vel,trap_acc,s_pos,s_vel,s_acc\n");
    const double dt = 0.002;  // 500 Hz — UR RTDE rate
    const double T = std::max(trap.duration(), scurve.duration());
    for (double t = 0.0; t <= T; t += dt) {
        const State a = trap.sample(t);
        const State b = scurve.sample(t);
        std::printf("%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", t, a.position, a.velocity,
                    a.acceleration, b.position, b.velocity, b.acceleration);
    }
    return 0;
}
