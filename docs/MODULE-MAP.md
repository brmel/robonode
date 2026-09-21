# Module map — how to work on one system without touching the others

The point of the seams is that you can change one thing and nothing else has to
know. This file says, per area: what it owns, what it may depend on, which tests
cover it, and the shortest loop that proves your change.

The structural rule is enforced, not advisory: `scripts/check-boundaries.sh`
fails the build on a cross-module include, and it runs in CI.

## The dependency rule

```
core  ←  motion  ←  celld  ←  gateway  ←  apps
  ↑        ↑         ↑          ↑
  └──── sandbox, vision, recorder, engines/* ────┘
```

Arrows point at *depends on*. Nothing depends on `gateway`; nothing at all
depends on `apps`. A module may only include headers from a module to its left.
`engines/*` are adapters: each owns exactly one third-party dependency and is
included by nobody except the composition root that links it.

## Areas

| Area | Owns | Depends on | Tests | Fast loop |
|---|---|---|---|---|
| `core/` | the vocabulary: `Status`, `AxisState`, `AxisLimits`, `TelemetryRow`, `CancelToken`, `Settings` | nothing | every suite | `ctest --preset fast` |
| `motion/` | the 1 kHz spine and the seams it consumes: `SyncExecutive`, `SetpointSource`, `Governor`, `AxisAdapter`, `Kinematics`, `Planner`, `Controller`, `ModuleRegistry` | core | `motion_tests`, `profile_tests`, `kinematics_tests` | `ctest --preset fast` |
| `celld/` | the cell as an object model: `Cell`, `RobotNode`, `ToolNode`, descriptors, `Scene`, `Conveyor`, `JsonDocStore` | core, motion | `celld_tests`, `scene_tests` | `ctest --preset fast` |
| `sandbox/` | untrusted user code: `Compiler`, `Program`, output validation | core | `sandbox_tests` | `ctest --preset fast` |
| `vision/` | perception seams: `Detector`, `Camera`, `CameraModel`, `Tracker` | core, sandbox | `vision_tests`, `tracking_tests` | `ctest --preset fast` |
| `recorder/` | telemetry → MCAP | core | `recorder_tests` | `ctest --preset fast` |
| `gateway/` | composition: `Platform`, `CellGateway`, `CellRuntime`, `SceneView` (the live world + the snapshot readers get), capabilities, command bus, HTTP contract | everything above | `capability_tests`, the four `gateway_*_tests` | `ctest --preset sim -R capability` |
| `engines/sim-mujoco/` | the physics twin; the only module that knows MuJoCo exists | core, motion, celld | `mujoco_tests`, `arm_tests` | `ctest --preset sim -R mujoco` |
| `engines/opencv/` | the real detector behind `Detector` | vision | `vision_cv_tests` | `ctest --preset dev -R vision_cv` |
| `engines/rtb/`, `services/rtb-kinematics/` | real-robot kinematics via Robotics Toolbox (a Python service) | motion seams / none | `make test` in the service | `cd services/rtb-kinematics && make test` |
| `apps/cell_server/`, `apps/robonode_cli/` | composition roots: they choose which vendor drivers to link | gateway | `robonode_cli_*` tests, the browser journeys | `npx playwright test` |
| `apps/cell_server/web/` | the browser client — a thin client of the same facade | the HTTP contract | `e2e/cell.spec.ts` | `npx playwright test -g "…"` |

## Recipes

### "I want to write a better algorithm"

You need no C++ and no build.

```sh
robonode --server http://localhost:8080 define vision my-grasp "x; y; z + 0.05"
robonode --server http://localhost:8080 version vision user.my-grasp
```

or the browser's **Editor** tab. It compiles into the sandbox, appears as a
version on the capability card, and is selectable everywhere. Nothing in the
platform changed.

### "I want to add a built-in version in C++"

One registry, one file. `vision/include/robonode/vision/tracker.hpp` is the
model: implement the seam, register the version in that module's
`register_*` function, add a test next to the existing ones. No other module
compiles differently.

### "I want to put a mature engine behind a seam" (Pinocchio, OMPL, cuRobo, ONNX)

Create `engines/<name>/` with its own `CMakeLists.txt` owning that dependency
and an `option(ROBONODE_BUILD_<NAME>)` defaulting OFF until it is proven. Your
module implements an existing seam from `motion/` or `vision/`; nothing above
it changes. If you find yourself editing `gateway/` to make it work, the seam is
wrong — say so in the PR.

### "I want to change the scenario"

`apps/cell_server/scenes/*.scene.json` — a base world plus objects, as data.
Fork one in the UI, move a number, run it. The physics recomposes. No build.

### "I want to add an application"

`apps/cell_server/apps/*.app.json` — a program of task steps (`deliver`,
`pick`, `place`, `intercept`, `conveyor`, `wait`, `version`, `family`). Every
step is also a CLI verb, so you can try the sequence by hand before writing the
file.

### "I want to work on the UI"

```sh
docker compose up -d          # the dev override mounts web/ from source
# edit apps/cell_server/web/*.js, reload the browser
npx playwright test -g "the 3D view"
```

No C++ build at all. The UI is a thin client: if you need data the contract does
not carry, that is a gateway change and a schema change in `contracts/`.

### "I want to work on the physics"

```sh
cmake --preset sim && cmake --build --preset sim -j
ctest --preset sim -R "mujoco|arm"
```

MuJoCo builds from source the first time. Contact groups, the weld, the world
layout: `engines/sim-mujoco/worlds/rail_ur10e.xml` and ADR-16.

### "I want to work on the Python service"

```sh
cd services/rtb-kinematics && make setup && make test
```

A venv with pinned deps. It never builds C++, and the C++ never imports it —
they meet over a JSON HTTP RPC behind the `Kinematics` seam.

## What crosses areas (and therefore needs care)

- **The wire contract** (`contracts/*.schema.json`) is shared by the browser,
  the CLI and every test. Changing a payload shape is a change to all three;
  `e2e/contract.spec.ts` validates it against a live server.
- **`config/robonode.settings.json`** is read by everything. Adding a key is
  additive and safe; changing a default changes behaviour everywhere.
- **The seams themselves.** Adding a method to `Scene` or `AxisAdapter` obliges
  every implementation. That is the intended friction: it is a design decision,
  not a refactor.
- **The RT path.** Anything reachable from the executive's cycle must not
  allocate, lock, or block (ADR-5/6/7).
