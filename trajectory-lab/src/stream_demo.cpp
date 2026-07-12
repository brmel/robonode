// Exercise 4 — fixed-rate setpoint streamer: the UR servoj / Fanuc Stream
// Motion shape in miniature.
//
//   [producer thread, elevated priority if the OS allows]
//       every 2 ms (absolute deadlines): sample S-curve -> push setpoint
//   [consumer thread]
//       drains ring buffer (the "network sender"), records timing jitter
//
// Things this demonstrates that interviews probe:
//   - absolute deadlines (sleep_until) so loop period does not drift
//   - no allocation, no locks, no IO on the producer path
//   - jitter measured as max |actual - nominal| tick time, not average
//
// On PREEMPT_RT Linux you'd add: SCHED_FIFO via pthread_setschedparam,
// mlockall(MCL_CURRENT|MCL_FUTURE), pinned core. Attempted below where
// supported; failure is reported, not fatal (needs privileges).

#include <pthread.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

#include "trajlib/scurve.hpp"
#include "trajlib/spsc_ring.hpp"

using namespace trajlib;
using Clock = std::chrono::steady_clock;

namespace {

struct Setpoint {
    double t;
    State state;
};

constexpr auto kTick = std::chrono::milliseconds{2};  // 500 Hz, UR RTDE rate

bool try_elevate_priority() {
    sched_param sp{};
    sp.sched_priority = 47;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0;
}

}  // namespace

int main() {
    const SCurveProfile profile{
        0.0, 0.5, {.max_velocity = 0.25, .max_acceleration = 1.0, .max_jerk = 10.0}};

    SpscRing<Setpoint, 256> ring;
    std::atomic<bool> done{false};
    std::atomic<int> overruns{0};
    std::atomic<long> max_jitter_us{0};

    std::thread producer{[&] {
        const bool rt = try_elevate_priority();
        std::printf("producer: SCHED_FIFO %s\n", rt ? "acquired" : "unavailable (expected without privileges)");

        const auto t0 = Clock::now();
        auto deadline = t0;
        const double T = profile.duration();

        for (double t = 0.0; t <= T; t += 2e-3) {
            deadline += kTick;                  // absolute: no drift
            std::this_thread::sleep_until(deadline);

            const auto now = Clock::now();
            const auto jitter =
                std::chrono::duration_cast<std::chrono::microseconds>(now - deadline).count();
            if (jitter > max_jitter_us.load(std::memory_order_relaxed)) {
                max_jitter_us.store(jitter, std::memory_order_relaxed);
            }
            if (jitter > 500) overruns.fetch_add(1, std::memory_order_relaxed);

            if (!ring.push({t, profile.sample(t)})) {
                overruns.fetch_add(1, std::memory_order_relaxed);  // consumer too slow
            }
        }
        done.store(true, std::memory_order_release);
    }};

    std::vector<Setpoint> received;
    received.reserve(4096);
    while (!done.load(std::memory_order_acquire) || !ring.empty()) {
        if (auto sp = ring.pop()) {
            received.push_back(*sp);
        } else {
            std::this_thread::yield();
        }
    }
    producer.join();

    std::printf("streamed %zu setpoints @ 500 Hz over %.3f s\n", received.size(),
                profile.duration());
    std::printf("max tick jitter: %ld us, overruns(>500us or full ring): %d\n",
                max_jitter_us.load(), overruns.load());
    std::printf("final setpoint: p=%.6f v=%.6f (target 0.5, 0)\n",
                received.back().state.position, received.back().state.velocity);
    return 0;
}
