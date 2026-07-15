# AGENTS.md — autonomous operating manual

You are an agent working this repo **unsupervised, start to finish**. This file is the constitution: how to pick work, prove it done, avoid design drift, and record progress — all headless, with no human in the loop. When in doubt, the rule here wins.

## Vision (why — read [CLAUDE.md](CLAUDE.md) + [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) first)

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

One issue = one commit = one matrix row moving ▶→✅. Small slices over big ones.

## Pick next

Order comes from the tracker (**issue #19**, phases R → 0 → 1 → 5). Within that, take the lowest-numbered issue that is `agent-ready` and whose `blocked by` are all closed.

```sh
gh issue list --state open --label agent-ready --json number,title,labels \
  --jq 'sort_by(.number)[] | "#\(.number) \(.title)"'
gh issue view <n> --json body --jq .body     # read acceptance criteria + blocked-by
```

If the top pick is blocked, skip to the next unblocked one. If nothing is `agent-ready`, stop and report.

## Definition of Done — non-negotiable, executable

A slice is done only when **`scripts/verify.sh` exits 0** (design invariants + build + every C++ suite) — add `--e2e` when you touched the UI or gateway (browser journeys). Then:

- [ ] a test exists in the correct layer and is green (see `docs/TEST-STRATEGY.md`)
- [ ] the issue's acceptance criteria are all met
- [ ] the scenario-matrix row is flipped ▶→✅ (or a new row added) in `docs/TEST-STRATEGY.md`
- [ ] the issue is closed with a one-line evidence note (what proves it)
- [ ] committed on `UsingRealRobot`, one issue per commit, `Co-Authored-By: Claude Opus 4.8`

"Works on my machine" is not done. `verify.sh` green is done.

## Design invariants — never drift (enforced by `scripts/check-design.sh`, a CI gate)

These are the ADRs as hard rules. `check-design.sh` fails the build on violation, so you *cannot* merge drift — but know them so you don't fight the gate:

- **Seams are contracts** — extend `AxisAdapter` / `Kinematics` / `Planner` / `DriverRegistry` / `CellGateway`; never reach around them. Deleting a seam is a regression. (`check-boundaries.sh`)
- **RT-loop purity (ADR-5/6/7)** — the 1 kHz path carries no Python, no heap alloc, no locks, no blocking I/O; kinematics in-process (Pinocchio); non-RT↔RT via lock-free SPSC; seams return `std::expected`, never throw.
- **Descriptor-driven (no hardcoding)** — tree, limits, ports, robot, programs are data; pure modules (`core`/`motion`/`celld`) carry no file/path/magic literals.
- **Don't reinvent** — reuse the engines in `docs/ARCHITECTURE.md` Part 2 behind a seam; adding a dep without a seam is drift.
- **One contract, many surfaces (ADR-8)** — a capability reachable in the UI but not the CLI (or vice-versa) is a bug; both are thin clients of the facade.
- **Logging is spdlog/fmt structured+async (ADR-9)** — no `printf`/`iostream` in product code.
- **Self-documenting code** — small, single-purpose functions and classes with intention-revealing names; comments carry the *why*, not the *what*. If a block needs a comment to explain what it does, extract and name it. Favour many small units over long methods.

The full architecture + rationale: `docs/ARCHITECTURE.md`. Decisions: `docs/DECISIONS.md`.

## Tools you have (use them, headless)

| Need | Tool |
|---|---|
| Read/close/label/comment issues, read the tracker | `gh` CLI |
| Unit + integration truth | `ctest --test-dir build` (via `scripts/verify.sh`) |
| Browser e2e (regression) | `npx playwright test` (via `verify.sh --e2e`) |
| Browser e2e (exploratory/live) | Playwright **MCP server** — navigate/snapshot/click/screenshot the running app |
| Drive the whole system headless | `robonode` CLI with `--json` (once #43 lands — agent-complete) |
| Logs / traces / telemetry | one followable stream (once #44 lands); MCAP flight recorder now |
| Run the live app | `docker compose up -d` → http://localhost:8080 |

## Roadmap = single source of truth, zero drift

- **Tracker #19** is the ordered backlog + phase map. **`docs/TEST-STRATEGY.md`** is the progress ledger (scenario matrix). Update both as part of Done — the roadmap is not a separate artifact you sync later, it *is* the record.
- Every open issue maps to a phase in #19; every user-facing capability maps to a matrix row. No orphans. If you create work, add it to both.
- Do not invent scope. If the slice needs something not in an issue, open an issue for it (tracer-slice shape: what, acceptance criteria, blocked-by), don't silently expand.

## Commit / branch

Work on `UsingRealRobot`. Commit per issue. Never commit red (`verify.sh` first). Push when the slice is done. End commit messages with `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`.

## Stop and ask the human (do NOT proceed autonomously) when:

- The acceptance criteria are ambiguous or self-contradictory.
- The slice conflicts with a design invariant / ADR (resolving it changes the architecture).
- The action is destructive or outward-facing beyond the repo (deleting data, publishing, secrets).
- An issue is blocked and no unblocked issue remains.
- `verify.sh` fails in a way that implies a design problem, not a local bug.

Everything else: proceed. The gates catch mistakes; that's what they're for.

## Launch prompt (what the human types to start you)

> "Work the RoboNode backlog autonomously per AGENTS.md: pick the next agent-ready unblocked issue from tracker #19, implement the slice, make `scripts/verify.sh` pass, flip its scenario-matrix row, close it with evidence, commit, and continue until the queue is empty or you hit a stop condition. Report each closed issue."
