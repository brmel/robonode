#pragma once

#include <cstddef>
#include <map>
#include <string>

namespace robonode {

// Every tunable the platform has, as data. Nothing here may be re-declared as a
// literal in the code that uses it: a heuristic you cannot change without a
// rebuild is a heuristic nobody will improve.
//
// This is the plain vocabulary (core depends on nothing); the JSON loader lives
// in celld, which owns that dependency. Missing keys keep the default below, so
// a partial settings file is a valid override rather than a trap.
struct Settings {
    struct Ik {
        double lambda{0.05};          // DLS damping
        double tol_m{1e-4};           // convergence: TCP error norm
        int max_iters{200};
        double max_step_rad{0.1};     // per-iteration joint step clamp
        double carrier_weight{0.08};  // mobility of a carrier axis vs an arm joint
        int line_steps{30};           // moveL interpolation steps
        int sandbox_line_steps{15};
        double lead_in_s{0.8};  // how far upstream a rendezvous meets a moving part
        double tol_rad{5e-3};            // convergence: orientation error (rad)
        double orientation_weight{0.3};  // how much a radian counts against a metre in pose IK
        double clearance_m{0.18};        // how high an avoiding planner lifts over an obstacle
    } ik;

    struct Motion {
        double rate_hz{1000.0};
        double settle_s{1.5};          // dwell after a Cartesian move
        double stop_time_s{0.35};      // on-path deceleration window for a cancel
        double max_duration_s{120.0};  // refuse trajectories longer than this
        double limit_tolerance{1e-3};  // fraction of travel tolerated at a soft limit
    } motion;

    struct App {
        double approach_m{0.06};      // clearance above a pick/place target
        double grasp_reach_m{0.05};   // how close the part must be to be grasped
        int intercept_attempts{4};    // tries before an interception is given up
        double intercept_lead_s{1.6}; // first guess at how long the reach takes
        double sight_timeout_s{4.0};  // how long to wait for a part to come into view
        std::string grip_constraint{"grip"};  // the model constraint a closed tool holds with
    } app;

    struct Tracking {
        std::size_t window{12};       // sightings a tracker may keep
        double gate_m{0.25};          // a sighting further than this is a different part
        double min_span_s{0.15};      // sightings must span this long before a speed is claimed
        double smoothing{0.4};        // exponential estimator: fraction of each disagreement taken
    } tracking;

    struct Gateway {
        std::size_t queue_capacity{64};
        int telemetry_decimation{20};  // publish every Nth RT cycle
        int stream_period_ms{20};      // SSE frame period
        int log_every_n_frames{25};
        int stall_timeout_ms{15000};  // give up only when the cell stops changing
        // What "the loop kept up" means, as numbers. Reported on every run and
        // asserted by the deploy smoke: a regression in RT quality is otherwise
        // a line in a log that nobody reads until a machine misbehaves.
        double max_overrun_fraction{0.05};  // cycles later than one full period
        double max_jitter_p99_us{1500.0};
    } gateway;

    struct Workspace {  // bound on what a sandboxed planner may emit (metres)
        double lo[3]{-3.0, -3.0, -1.0};
        double hi[3]{3.0, 3.0, 3.0};
    } workspace;

    struct Recorder {  // every run to MCAP, with the versions that produced it
        bool enabled{false};
        std::string dir{"runs"};
    } recorder;

    // Comparing two versions of a capability only means something if the work
    // exercises it: a reach says nothing about a detector. So each capability
    // names the application that puts IT under load — data, because which app
    // proves a tracker is exactly the kind of judgement that should change
    // without a rebuild.
    struct Compare {
        std::map<std::string, std::string> trial;
        std::string fallback;  // named by the settings file, never by this code
    } compare;
};

}  // namespace robonode
