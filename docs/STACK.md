# Stack — exact pins, licenses, and modularity seams

> Verified 2026-07-12. Survey/rationale lives in [TECH-LANDSCAPE.md](TECH-LANDSCAPE.md); this file is the buildable truth: exact repo, exact tag, how it integrates, whether it has been proven on a machine, and which platform seam hides it. **Rule: no dependency without a seam** — every third-party sits behind an interface we own, so any row below can be swapped without touching product logic.

## Modularity seams (the contract layer)

| Seam (we own) | Defined in | What plugs in behind it | Swap proof |
|---|---|---|---|
| `AxisAdapter` / arm adapters | motion-core `axis_adapter.hpp`, SPEC §3.1 | EtherCAT drives, UR, Fanuc, ABB, **sim** | SimAxis + FaultTimedAxis already swap in tests |
| OTG slot (Tier C `TrajectoryGenerator`) | SPEC §3.2 | Ruckig, in-house OTG, customer plugins | trajlib `MotionPlan` is the current occupant; Ruckig verified as replacement |
| `MotionPlan` / blend+retime layer | motion-core | in-house profiles (trajlib) — **deliberately not outsourced** (differentiator D2) | — |
| Topic transport | `robonode-idl` topics + capability protos | Zenoh today; DDS possible; payloads defined by our IDL either way | IDL has no Zenoh types in it |
| Recorder / live-viz | telemetry service boundary; wire format = MCAP | foxglove-sdk, or raw mcap writer + any WS server | both verified independently |
| Tier B sandbox runtime | `CellClient` gRPC surface (cell_client.proto) | wasmtime (WASM) and OCI containers, same manifest | contract identical by design (ADR-3) |
| Program engine | flow UI compiles to engine API | BehaviorTree.CPP; could be replaced by an in-house sequencer | — |
| Sim | sim adapter implements `AxisAdapter` + twin exposes same node API (I5) | Gazebo now, Isaac later | motion core has zero sim includes |
| North APIs | gRPC/REST from IDL | any client (web app is just a client) | — |

## Pinned dependencies

| Role | Pick | Repo · pin | License | Integration | Verified |
|---|---|---|---|---|---|
| OTG (default occupant of Tier C slot) | Ruckig **community** | [pantor/ruckig](https://github.com/pantor/ruckig) · `v0.17.3` | MIT | FetchContent, `ruckig::ruckig` | ✅ built + ran on macOS: 0.15 µs/cycle 1-DOF (1 kHz budget = 1000 µs); 0.633 s trajectory **matches trajlib S-curve exactly** — independent cross-check of our math |
| Profiles / blending / retiming | in-house (`trajectory-lab` → motion-core) | this repo | — | source | ✅ Catch2 + CHECK suites |
| EtherCAT master | IgH EtherLab | [gitlab.com/etherlab.org/ethercat](https://gitlab.com/etherlab.org/ethercat) · branch `stable-1.6` (1.6.9, Apr 2026, actively maintained) | GPLv2 (kernel module) + LGPL (userspace lib we link) | kernel module on the Linux rig; userspace `libethercat` from RT process | ⬜ needs Linux bench (M1) — cannot run on macOS by nature |
| UR robot protocol | ur_client_library | [UniversalRobots/Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) · `2.13.0` | Apache-2.0 | FetchContent, alias `ur_client_library::urcl` in-tree | ✅ builds clean; `adapters/ur` UrWristAdapter built on it (compile-verified) |
| UR test rig | URSim | Docker Hub [universalrobots/ursim_e-series](https://hub.docker.com/r/universalrobots/ursim_e-series) (classic PolyScope; ports 5900/6080) | UR EULA | `scripts/ursim.sh up` | ✅ pulled + booted; ⚠️ Apple-silicon: amd64-only image → PolyScope boots under qemu in 15-40 min, and **URControl withholds RTDE data until PolyScope confirms the safety setup** (verified by raw-protocol probe: `"SafetySetup has not been confirmed yet"`). Instant on x86 Linux |
| Data plane | Zenoh (C lib + C++ bindings) | [eclipse-zenoh/zenoh-c](https://github.com/eclipse-zenoh/zenoh-c) · `1.9.0`, [zenoh-cpp](https://github.com/eclipse-zenoh/zenoh-cpp) · `1.9.0` (header-only over zenoh-c, C++17) | EPL-2.0/Apache-2.0 | prebuilt release artifacts (no Rust toolchain needed on dev machines) | ⬜ M1; maturity caveat: bindings self-described “active development”, but [rmw_zenoh is now C++ on these same bindings](https://discourse.openrobotics.org/t/flexibility-in-rmw-zenoh-switching-between-zenoh-cpp-and-zenoh-pico-backends/41324) |
| ROS 2 interop | zenoh-plugin-ros2dds | [eclipse-zenoh/zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) · track zenoh 1.9.x | EPL-2.0/Apache-2.0 | router plugin, off the RT path | ⬜ v1.x scope |
| Telemetry: record + live viz | **Foxglove SDK** (new 2025/26 — supersedes hand-rolling mcap+ws) | [foxglove/foxglove-sdk](https://github.com/foxglove/foxglove-sdk) · `sdk/v0.25.3` (C++/Python over Rust core; one API for MCAP files **and** live WebSocket) | MIT | vendored lib | ⬜ M1 spike; pre-1.0 caveat |
| Telemetry fallback: raw MCAP writer | mcap C++ | [foxglove/mcap](https://github.com/foxglove/mcap) · `releases/cpp/v2.1.3` | MIT | **fetch as tarball** — repo carries git-lfs assets, plain FetchContent git clone fails without git-lfs | ✅ in production use: `McapRecorder` (motion-core) records demo + UR runs, Foxglove-openable; recorder test in suite |
| Tier B sandbox: WASM | wasmtime C API | [bytecodealliance/wasmtime](https://github.com/bytecodealliance/wasmtime) · `v46.0.1` | Apache-2.0 | `libwasmtime` C API from celld | ⬜ M2 spike **must verify component-model support via C API** ([historically partial](https://github.com/bytecodealliance/wasmtime/issues/6987)); fallback: WASI-p1 modules first, components later |
| Tier B sandbox: containers | OCI runtime (containerd class) | distro packages | Apache-2.0 | celld supervises | ⬜ M2 |
| Program engine | BehaviorTree.CPP | [BehaviorTree/BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) · `4.9.1` | MIT | FetchContent | ⬜ M2 |
| Sim twin | Gazebo **Jetty** (gz-sim 11) | [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) | Apache-2.0 | distro packages on Linux; headless `gz sim -s` in CI ([releases](https://gazebosim.org/docs/latest/releases/): Jetty supported → 2030; Harmonic LTS fallback → 2029; **Ionic EOLs 2026-09 — do not pin**) | ⬜ M1/M2 |
| OPC UA server | open62541 | [open62541/open62541](https://github.com/open62541/open62541) · `v1.5.5` | MPL-2.0 | vendored, north gateway only | ⬜ v1.x scope (FR-9) |
| OTA | Mender | [mendersoftware/mender](https://github.com/mendersoftware/mender) · `5.1.0` | Apache-2.0 | self-hosted server + A/B client | ⬜ M3 |
| North RPC | gRPC C++ + buf (IDL lint/breaking) | distro/package pins at M2 | Apache-2.0 | generated from `robonode-idl` | ⬜ M2 |
| Web 3D twin | three.js + urdf-loaders (+ React/R3F) | [gkjohnson/urdf-loaders](https://github.com/gkjohnson/urdf-loaders) · `v0.13.1` | MIT/Apache-2.0 | web app only — pure client of north APIs | ⬜ M2/M3 |
| Retiming reference | toppra | [hungpham2511/toppra](https://github.com/hungpham2511/toppra) · `v0.6.4` | MIT | **prototyping reference only** (Python) — production retiming stays in-house C++ | — |

## Decisions this pass changed or sharpened

1. **Foxglove SDK replaces the planned mcap-writer + ws-protocol handroll** for the flight recorder + live streams — one C++ API, same MCAP wire format, so the seam (recorder service) is unchanged and raw-mcap remains the verified fallback. ([announcement](https://foxglove.dev/blog/announcing-the-foxglove-sdk))
2. **Ruckig community is single-target-state only**: intermediate waypoints are Pro (paid, local) or a non-RT cloud API ([docs](https://docs.ruckig.com/md_pages_2__intermediate__waypoints.html)). No impact on the architecture — waypoint blending/retiming was always the in-house layer (differentiator D2) — but it kills any temptation to outsource it.
3. **Gazebo pin corrected to Jetty (gz-sim 11)**: Ionic (gz-sim 10, what a naive "latest tag" pick would grab) EOLs 2026-09.
4. **wasmtime component-model-from-C++ flagged as the one unproven bet** in the Tier B design → M2 opens with that spike; module-level WASI is the fallback and the `CellClient` contract is identical either way.
5. **Dev-machine toolchain gaps** found while verifying: `git-lfs`, `cargo`, `protoc`, `buf` absent (docker present). None block M1 (tarball fetch, prebuilt zenoh artifacts); install `buf` + `protoc` at M2 when IDL codegen starts.

## Version review cadence

Re-run the tag sweep (`git ls-remote --tags`) at each milestone boundary; record changes here. Breaking-change watchlist: zenoh 1.x → 2.x, foxglove-sdk pre-1.0 API churn, wasmtime major-per-month cadence (pin, don't track latest).
