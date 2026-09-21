# AGENTS.md — autonomous operating manual

You are an agent working this repo **unsupervised, start to finish**. This file is the constitution: how to pick work, prove it done, avoid design drift, and record progress — all headless, with no human in the loop. When in doubt, the rule here wins.

## Vision (why — read [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) first)

Build a **modular, open-source platform to test robotics algorithms in real physics** — users and agents swap or bring their own module (path/control/vision/learning) behind **one clean MIL-style interface**. The four principles you serve every slice: **(1) reuse mature engines, never reinvent** (MuJoCo · Pinocchio · Ruckig · OpenCV/ONNX · MCAP · spdlog — behind a seam); **(2) modular — extend a seam, never bypass**; **(3) RT-correct + descriptor-driven** (no Python/alloc/locks in the loop, no hardcoding); **(4) one contract, many surfaces** (UI = CLI = SDK). **Follow the architecture and improve it via ADR + issue — never drift silently, never reinvent.**

## The loop (repeat until the queue is empty or you hit a stop condition)

```
1. PICK   next ready, unblocked issue (see "Pick next")
2. PLAN   read its acceptance criteria + the seam it extends; restate the slice
3. BUILD  implement the thin vertical slice — extend a seam, never bypass it
4. TEST   add/extend a test in the right layer (unit · integration · e2e)
5. VERIFY scripts/verify.sh   (and --e2e if you touched the UI/gateway)  ← MUST pass
6. RECORD flip the scenario-matrix row; close the issue with evidence; commit
7. NEXT   go to 1
```

One issue = one commit = one matrix row moving →. Small slices over big ones.

## Pick next

Order comes from the tracker (**issue **, phases R → 0 → 1 → 5). Within that, take the lowest-numbered issue that is `agent-ready` and whose `blocked by` are all closed.

```sh
gh issue list --state open --label agent-ready --json number,title,labels \
  --jq 'sort_by(.number)[] | "#\(.number) \(.title)"'
gh issue view <n> --json body --jq .body     # read acceptance criteria + blocked-by
```

If the top pick is blocked, skip to the next unblocked one. If nothing is `agent-ready`, stop and report.

## Before you start a slice

The issue's **acceptance criteria are the contract** — they say what "done"
means and the seam it touches. [docs/MODULE-MAP.md](docs/MODULE-MAP.md) says
where the change belongs, what that area may depend on, which tests cover it,
and the shortest loop that proves it. Read both; between them you should never
start blind.

If an issue's criteria are vague enough that two people would build different
things, say so in the issue and sharpen them first — that is cheaper than a
rejected slice.

## Definition of Done — non-negotiable, executable

A slice is done only when **`scripts/verify.sh` exits 0** (design invariants + build + every C++ suite + the deploy smoke) — add `--e2e` when you touched the UI, the gateway or the wire contract (browser journeys + contract validation). Then:

- [ ] a test exists in the correct layer and is green (see `docs/TEST-STRATEGY.md`)
- [ ] the issue's acceptance criteria are all met
- [ ] the scenario-matrix row is flipped → (or a new row added) in `docs/TEST-STRATEGY.md`
- [ ] the issue is closed with a one-line evidence note (what proves it)
- [ ] committed, one issue per commit, `Co-Authored-By: Claude Opus 4.8`

"Works on my machine" is not done. `verify.sh` green is done.

## Design invariants — never drift (enforced by `scripts/check-design.sh`, a CI gate)

These are the ADRs as hard rules. `check-design.sh` fails the build on violation, so you *cannot* merge drift — but know them so you don't fight the gate:

- **Seams are contracts** — extend `AxisAdapter` / `SetpointSource` / `Kinematics` / `Planner` / `Controller` / `Detector` / `ToolNode` / `ModuleRegistry` / `CapabilityBase` / `JsonDocStore` / the Platform facade; never reach around them. Deleting a seam is a regression. A seam belongs in the layer that *consumes* it. (`check-boundaries.sh`)
- **RT-loop purity (ADR-5/6/7)** — the 1 kHz path carries no Python, no heap alloc, no locks, no blocking I/O. Kinematics in-process (Pinocchio is planned; today it is a hand-rolled DLS solve on MuJoCo's Jacobian), non-RT↔RT via lock-free SPSC, and seams returning `std::expected` instead of throwing are all *open* — write new code as if they were true, and do not add a violation the gate has not caught yet.
- **One owner for the physics (ADR-16)** — `mjData` is single-owner. The thread that steps the world samples frames and contacts; every other surface reads that snapshot. Reading the live world from an HTTP thread corrupts the solver, and it looks like a crash inside `mj_collideTree`.
- **Data-driven (no hardcoding)** — tree, limits, robots, motions, stations and programs are descriptor data; **every tunable is in `config/robonode.settings.json`** (ADR-14). Pure modules (`core`/`motion`/`celld`) carry no file/path literals. A number you cannot change without a rebuild is drift.
- **Address by name, never by position (ADR-13)** — a robot names its carrier and joints; `gateway/` may not index the flat node list. The gate greps for it.
- **Commands are identified (ADR-12)** — a reply means *accepted*; completion is `applied_id`. Never add a per-verb "am I done yet" predicate to a surface; use `Platform::await_settled` / `awaitApplied`.
- **Failures are loud** — a fallback is legitimate only when the degraded behaviour is *correct*. No silent `catch {}`, no default-constructed answer standing in for a missing one.
- **The wire contract is declared** — `contracts/*.schema.json`, validated against a live server by `e2e/contract.spec.ts`. Capabilities describe themselves (`GET /capabilities`); no surface hardcodes a capability id.
- **Don't reinvent** — reuse the engines in `docs/ARCHITECTURE.md` Part 2 behind a seam; adding a dep without a seam is drift.
- **One contract, many surfaces (ADR-8)** — a capability reachable in the UI but not the CLI (or vice-versa) is a bug; both are thin clients of the facade.
- **Logging is spdlog/fmt structured+async (ADR-9)** — no `printf`/`iostream` in product code.
- **Self-documenting, minimal-comment, modern code** — code speaks; comment only the non-obvious *why*, never narrate the *what*, no section-header comments. Small single-purpose functions/classes, intention-revealing names, no god-classes. Prefer modern C++20/JS idioms over boilerplate. Fewer, better lines.

The full architecture + rationale: `docs/ARCHITECTURE.md`. Decisions: `docs/DECISIONS.md`.

## Tools you have (use them, headless)

| Need | Tool |
|---|---|
| Read/close/label/comment issues, read the tracker | `gh` CLI |
| Fast loop while iterating | `cmake --preset dev && ctest --preset fast` (~2 s, no physics build) |
| Unit + integration truth | `ctest --preset sim` (via `scripts/verify.sh`) |
| Browser e2e (regression) | `npx playwright test` (via `verify.sh --e2e`) |
| Browser e2e (exploratory/live) | Playwright **MCP server** — navigate/snapshot/click/screenshot the running app |
| Drive the cell **you are looking at** | `robonode --server http://localhost:8080 <verb>` — without `--server` the CLI boots a private cell and answers about a different robot |
| Verbs | motion `run` `movel` `movep <x y z qw qx qy qz>` `jog` `stop`/`estop`/`resume` `grasp`/`release` · task steps `pick` `place` `intercept` `deliver` `conveyor` · algorithms `swap` `family` `version` `define` `verbs` `compare <cap> <a> <b>` `runs` `run-record <file>` `last-run` (`run --quality` prints it) · apps + scenes `app` `apps` `scenes` `scene` `use-scene` `fork <kind> <from> <to>` `sessions` · robots `robots` `add-cell <id> --robot <file>` `drop-cell <id>` `--cell <id>` (drive that robot) · views `nodes` `telemetry` `model` `capabilities` `capability <id>` `modules` `stations` `cells` `logs` `contacts` `watch --json --quiet <fields>` |
| Why a move stopped short | `robonode --server … contacts` — the cell names the bodies that are touching |
| Contract truth | `npx playwright test e2e/contract.spec.ts` against a running server |
| Logs / traces / telemetry | `robonode logs`, the UI Logs panel, and MCAP per run (`recorder.enabled` in settings) |
| Run the live app | `docker compose up -d` → http://localhost:8080 |

## Roadmap = single source of truth, zero drift

- **Tracker ** is the ordered backlog. It records what shipped per release and what is open, in order. Where a closed issue and the code disagree, the code wins: the P0–P6 duplicates (–) were closed in bulk and several were never implemented. **`docs/TEST-STRATEGY.md`** is the progress ledger (scenario matrix). Update both as part of Done — the roadmap is not a separate artifact you sync later, it *is* the record.
- Every open issue maps to a phase in; every user-facing capability maps to a matrix row. No orphans. If you create work, add it to both.
- Do not invent scope. If the slice needs something not in an issue, open an issue for it (tracer-slice shape: what, acceptance criteria, blocked-by), don't silently expand.

## Where things go

- `apps/` is **product surface only** — the server and the CLI. They are the composition roots: the only place that decides which vendor drivers a build registers.
- `examples/` is for runnable demos of a single seam. Nothing in `apps/` or the libraries may depend on them.
- `engines/` holds vendor adapters behind a seam; `contracts/` holds the wire schemas; `config/` holds tuning.
- A new capability is one `caps_.add(...)` plus its descriptor — if you find yourself editing a surface to teach it a capability id, stop: that is the duplication the registry exists to prevent.

## Commit / branch

Work. Commit per issue. Never commit red (`verify.sh` first). Push when the slice is done. End commit messages with `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`.

## Stop and ask the human (do NOT proceed autonomously) when:

- The acceptance criteria are ambiguous or self-contradictory.
- The slice conflicts with a design invariant / ADR (resolving it changes the architecture).
- The action is destructive or outward-facing beyond the repo (deleting data, publishing, secrets).
- An issue is blocked and no unblocked issue remains.
- `verify.sh` fails in a way that implies a design problem, not a local bug.

Everything else: proceed. The gates catch mistakes; that's what they're for.

## Launch prompt (what the human types to start you)

> "Work the RoboNode backlog autonomously per AGENTS.md: pick the next agent-ready unblocked issue from tracker, implement the slice, make `scripts/verify.sh` pass, flip its scenario-matrix row, close it with evidence, commit, and continue until the queue is empty or you hit a stop condition. Report each closed issue."
