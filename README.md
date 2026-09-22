# RoboNode

[![CI](https://github.com/brmel/robonode/actions/workflows/ci.yml/badge.svg)](https://github.com/brmel/robonode/actions/workflows/ci.yml)
[![Licence](https://img.shields.io/badge/licence-Apache--2.0-blue.svg)](LICENSE)
[![Release](https://img.shields.io/badge/release-v0.9.0-informational)](CHANGELOG.md)

## The goal

RoboNode sets out to give roboticists somewhere to test a robot and an algorithm
without setting anything up first. The goal is a hosted platform where the cell is
already waiting for you — robot arms, vision systems, grippers, conveyors, trajectory
planners and controllers — each one a module you can configure, swap for another
version, or replace outright with your own code. You bring the algorithm; the robot,
the physics and the tooling around them are already there, and nothing about trying an
idea should require a lab, a rig, or a week of setup.

More on how it came about: [ibraverse.ca/projects/robonode](https://ibraverse.ca/projects/robonode/).

RoboNode is also an experiment in what generative AI and agents can build. I wanted to
see how much of a real-time C++ platform an agent could carry, and the answer shaped the
repo: the executable gates described in [AGENTS.md](AGENTS.md) exist because autonomy
only works when the checks are machine-verifiable rather than trusted.

**An open-source platform for testing robotics algorithms in real physics.**

Swap the vision, the tracking, the trajectory or the control algorithm of a
running robot cell — from a browser, a CLI, or your own code — and watch what it
does to a UR10e on a rail in MuJoCo. The physics does not pretend: the arm is
stopped by what is in its way, a grasp is a constraint the model closes on the
part it actually caught, and a conveyor carries the workpiece by friction. When
an algorithm is wrong, it visibly fails, and the log says what it hit.

```sh
docker compose up --build      # → http://localhost:8080
```

Building from source instead: `scripts/setup.sh --deps` (or
`cmake --preset dev` for the fast loop — see
[CONTRIBUTING.md](CONTRIBUTING.md)).

![The RoboNode dashboard mid-place: the arm carrying a part to the pallet, the Contacts card naming every body that is touching, and the capability modules on the right with their selectable versions](docs/images/dashboard.png)

*Mid-place. The **Contacts** card is reading the physics — `pallet ↔ wrist_2_link · part ↔ wrist_2_link` — and every card on the right is an algorithm you can swap while it runs.*

## What it looks like

![A grasp closing on the part in the physics engine: the gripper has the workpiece and the constraint is on the body it actually caught](docs/images/grasp-in-physics.png)

*A grasp is a constraint the model closes on the part it actually caught — not an animation of a successful pick.*

![The same contract driving a real UR10e arm on a bench](docs/images/real-ur10e.png)

*The same wire contract driving a real UR10e. The simulator is the safe half of the loop, not the whole of it.*

## Try it in five minutes

```sh
docker compose up --build      # → http://localhost:8080
```

No host toolchain needed. To build from source instead, see [CONTRIBUTING.md](CONTRIBUTING.md).

| | |
|---|---|
| **Run an application** | Click **Bin picking**. Vision locates the part, the arm picks it, the pallet receives it. |
| **Break it on purpose** | Dock → **Cell**, switch Tracking to `snapshot`, run **Moving bin picking**. The arm aims where the part *was* and misses. The log says why. |
| **Write your own algorithm** | Dock → **Editor**. `x`, `y`, `z + 0.05` is a valid grasp offset. It compiles into a sandbox and becomes a version you can select. |
| **Change the world** | Dock → **Scenes**. Fork `cluttered-line`, move the crate, hit **Run scene**. The physics rebuilds around your JSON. |
| **Drive it from a terminal** | `robonode --server http://localhost:8080 contacts` — what is touching what, right now. Every button in the UI has a CLI verb, because both talk to the same HTTP API. |

## Why it exists

Robotics algorithms are usually judged in a paper, a notebook, or a simulation
that flatters them. Here an algorithm has to work in a scenario that is honestly
hard: the target is moving, the sensor is late, and there is a decoy in the bin
the same colour as the part.

Everything you can change is data — the cell, the scene, the application, the
tuning. Everything you can replace sits behind an interface we own, so swapping
the planner touches neither the UI, nor the CLI, nor the real-time loop.

## How the repository is laid out

```sh
cmake -B build && cmake --build build -j && ctest --test-dir build
bash scripts/verify.sh            # the definition of done: gates + build + every suite + smoke
bash scripts/verify.sh --e2e      # …plus the browser journeys
```

| Folder | What lives there |
|---|---|
| [core/](core/) | The vocabulary every other module speaks: state, limits, one error type, lifecycle verbs. Depends on nothing. |
| [motion/](motion/) | Turning a goal into movement — trajectory planning, blending, and the per-cycle safety gate every command passes through. |
| [celld/](celld/) | The cell as an object model: robots, tools, conveyors and pallets, all built from JSON descriptors rather than code. |
| [vision/](vision/) | What sees — detectors, the camera model, and the tracker that predicts where a moving part will be. |
| [sandbox/](sandbox/) | Where a stranger's algorithm runs: a compiler and a fuel-bounded VM with no access to the host. |
| [gateway/](gateway/) | The control plane. Owns the live cell, routes commands, publishes telemetry. Nothing depends on it. |
| [engines/](engines/) | The borrowed engines, each behind one of our interfaces: MuJoCo, OpenCV, Robotics Toolbox, Wasmtime. |
| [adapters/](adapters/) | Real hardware behind that same interface — today a UR wrist driven at 500 Hz. |
| [services/](services/) | Helpers that run out of process and are spoken to over HTTP. |
| [recorder/](recorder/) | Telemetry written to MCAP, openable in Foxglove. |
| [apps/](apps/) | The two things you actually run: the cell server and the `robonode` CLI. |
| [contracts/](contracts/) | The wire contract as JSON Schemas, validated against a live server. |
| [robonode-idl/](robonode-idl/) | A Protobuf sketch of a future gRPC surface. Nothing generates from it yet. |
| [config/](config/) | Every tunable as data, so changing behaviour does not mean changing code. |
| [examples/](examples/) | One runnable demo per interface. Not product surface. |
| [tests/](tests/) · [e2e/](e2e/) | Unit and integration tests; browser journeys and the contract tests. |
| [scripts/](scripts/) | The gates CI runs — `verify.sh` is the definition of done. |
| [docs/](docs/) | How it is designed and why. Start with [ARCHITECTURE.md](docs/ARCHITECTURE.md). |

The boundaries are enforced, not just described: [scripts/check-boundaries.sh](scripts/check-boundaries.sh) fails CI on any cross-module include — `core` depends on nothing, `recorder` never sees `motion`, and `trajlib` never escapes `motion/`.

## Documents

| | |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | The system and the stack in one file. Start here. |
| [docs/DECISIONS.md](docs/DECISIONS.md) | Why each choice was made, one record per decision. |
| [docs/MODULE-MAP.md](docs/MODULE-MAP.md) | Where a change belongs, and which test proves it. |
| [docs/CHALLENGES.md](docs/CHALLENGES.md) | The scenarios where a naive algorithm genuinely fails. |
| [docs/TEST-STRATEGY.md](docs/TEST-STRATEGY.md) | The test layers and the scenario matrix. |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | Self-hosting, and the cloud options with their trade-offs. |
| [docs/REAL-ROBOTS.md](docs/REAL-ROBOTS.md) | Driving real robot kinematics through the same interfaces. |
| [AGENTS.md](AGENTS.md) | How an unsupervised agent works this repository, and the gates it must pass. |

## Contributing

Start with [CONTRIBUTING.md](CONTRIBUTING.md). The bar is one command:

```sh
bash scripts/verify.sh         # design gates + build + every C++ suite
bash scripts/verify.sh --e2e   # … plus the browser journeys (needs docker)
```

If it exits 0, the change is reviewable. Issues labelled **good first issue** are
scoped to one file and one test; new algorithms and new scenarios are the most
useful things you can send. Read [SECURITY.md](SECURITY.md) before hosting an
instance publicly — the platform runs code its users write. See also
[CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).

## Status

A cell you can drive from a browser, a CLI or a script: real physics, swappable
algorithms at every capability, user code in a sandbox, and a control plane that
can stop the robot. Everything claimed here is covered by `scripts/verify.sh` and
the browser suite.

Next, in order: 6-DoF orientation (`Goal::kCartesianPose` has no planner yet, the
largest gap), Pinocchio behind the kinematics interface, a planner that avoids
the contacts the cell already reports, and per-session cells.
