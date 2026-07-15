# CLAUDE.md — RoboNode project context (read this first)

Loaded every session. The vision, the non-negotiables, and where the truth lives. For the autonomous work loop see **[AGENTS.md](AGENTS.md)**; for the full map + stack see **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**.

## Vision

An **open-source, modular platform for testing robotics algorithms in real physics.** In a web app (and headless via CLI), users **and agents** see + manipulate robots **and stations** in a real physics engine, and swap or bring their own **module** — path/trajectory, robot control, vision, learning — behind **one clean interface that hides the complexity** (Matrox-Imaging-Library style). Every node is a typed capability with interchangeable versions and a bring-your-own slot, run safely in a sandbox.

## The four principles (do not violate; improve within them)

1. **Reuse mature engines — never reinvent.** Physics = MuJoCo. Kinematics = Pinocchio (RT) / Robotics Toolbox (offline). OTG = Ruckig. Planning = cuRobo/OMPL. Vision = OpenCV/ONNX. Telemetry = MCAP/Foxglove. Logging = spdlog/fmt. CLI = CLI11. The full catalogue with exact pins + the seam each hides behind is **ARCHITECTURE.md Part 2**. Adding a dependency without a seam we own is drift.
2. **Modular, MIL-style — one clean interface, everything behind a seam.** The seams we own: `AxisAdapter`, `Kinematics`, `Planner`, `DriverRegistry`, `SetpointSource`, `CellGateway`, the Platform facade (#33). Any engine above is swappable without touching product logic. **Extend a seam, never bypass it.** Enforced by `scripts/check-boundaries.sh`.
3. **Real-time correct + descriptor-driven.** The 1 kHz path has no Python, no heap alloc, no locks, no blocking I/O (ADR-5/6/7). Tree/limits/ports/robot/programs are **data, not code** — no hardcoding. One error model (`std::expected` at seams). One logging stack (spdlog, structured/async; no printf/iostream in product code).
4. **One contract, many surfaces.** Web UI, CLI (`--json`, agent-complete), and SDK are thin clients of the **same facade** — a capability in one but not another is a bug (ADR-8).

## Follow the architecture — AND improve it

The architecture is **living, not frozen.** When you find a better modular approach, or a more mature engine, or a way to harden the RT path: **propose it as an ADR + a tracer-slice issue, then let the gates prove it** — do not drift silently, and do not reinvent what a mature library already does. The external-review fold-in (ADR-5…9, issues #34–#44) is the model: findings → decisions → issues, all behind the existing seams.

## Where the truth lives (single sources)

| Question | File |
|---|---|
| What are we building + the stack (with pins + seams) | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| Why we chose it (ADR-1…10) | [docs/DECISIONS.md](docs/DECISIONS.md) |
| The ordered roadmap + phases | tracker **issue #19** (`gh issue view 19`) |
| Progress (scenario matrix, ▶→✅) | [docs/TEST-STRATEGY.md](docs/TEST-STRATEGY.md) |
| How to work it autonomously | [AGENTS.md](AGENTS.md) |
| Requirements / spec | [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md), [docs/SPEC.md](docs/SPEC.md) |

## Roadmap snapshot (keep current as work lands)

Branch: `UsingRealRobot`. Order (tracker #19): **Phase R** RT correctness (#34 Pinocchio, #35 SPSC, #36 expected — the P0 external-review fixes) · **Phase 0** cleanup + surfaces (#28 descriptors, #30 ontology, #33 facade, #43 CLI) · **Phase 1** real robot (#20 model) · **Phase 2** UX (#46 unified view, #47 version manager, #48 theme) · **Phase 3** capabilities (#6 vision, #26 Tier-B, #39 cuRobo) · **Phase 4** platform surface · **Phase 5** release.

Done: motion spine · MuJoCo twin · 7-DOF arm · Cartesian FK/IK · RTB kinematics behind seams · per-node driver swap · web app + live e2e · integration test · autonomy harness. **Next P0: #34 (Pinocchio, kill Python-in-loop).**

## Hard rules (every change)

- **Definition of Done = `scripts/verify.sh` exits 0** (design invariants + build + all suites; `--e2e` if you touched UI/gateway). Never commit red.
- **`scripts/check-design.sh`** is a CI gate — the ADRs as greppable rules. Don't fight it; it encodes the design.
- Every capability **adds a test** (right layer, `docs/TEST-STRATEGY.md`) and **flips its scenario-matrix row**.
- **Update the roadmap as you go** — tracker #19 + the matrix are the record, not an afterthought. No orphans, no drift.
- One issue = one commit on `UsingRealRobot`; end messages with `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`.
