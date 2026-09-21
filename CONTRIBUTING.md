# Contributing to RoboNode

Thanks for looking. This project is a platform for **testing robotics
algorithms in real physics** — the interesting contributions are usually a
better algorithm, a harder scenario, or a seam that hides an engine more
cleanly.

## The 60-second version

```sh
git clone https://github.com/brmel/robonode && cd robonode
docker compose up --build          # → http://localhost:8080
```

Change something, then:

```sh
bash scripts/verify.sh             # this is the Definition of Done
bash scripts/verify.sh --e2e       # … plus docker + the browser journeys
```

**A pull request is ready when `scripts/verify.sh` exits 0.** It runs the design
gates, the build, and every suite. Nothing else is required of you, and nothing
less is accepted — CI runs the same script.

## Building without Docker

```sh
scripts/setup.sh --deps        # installs cmake + OpenCV, then configures and builds
scripts/setup.sh sim           # …or skip --deps if you already have them
```

Or drive CMake yourself through the presets:

| Preset | What you get | When |
|---|---|---|
| `dev` | no MuJoCo, no vendor SDK — configures in seconds | algorithms, seams, tests |
| `sim` | the physics twin and the whole app | anything you can watch move |
| `full` | every optional engine (UR, RTB, Wasm) | before a release |
| `asan` | address + UB sanitizers on our code only | after touching threading or lifetimes |

```sh
cmake --preset dev && cmake --build --preset dev -j && ctest --preset fast
```

Needs a C++23 compiler and CMake ≥ 3.20. Everything else is fetched and pinned
by the build, **except OpenCV** — and if it is missing the build says so once
and carries on without the `robonode.opencv` detector rather than failing.

**Where does my change belong, and what can I break?**
[docs/MODULE-MAP.md](docs/MODULE-MAP.md) answers both, per area, with the
shortest loop that proves the change.

## Where to start

| You want to… | Look at | Add |
|---|---|---|
| write a better picking / tracking algorithm | `robonode define` or the browser's **Editor** tab | a capability version — no C++ needed |
| add a physics scenario | `apps/cell_server/scenes/*.scene.json` | a scene: base world + objects, as JSON |
| add an application | `apps/cell_server/apps/*.app.json` | a program of task steps |
| swap in a mature engine (Pinocchio, OMPL, cuRobo, ONNX) | `docs/ARCHITECTURE.md` Part 2 | an adapter behind an existing seam |
| harden the real-time path | `motion/` | keep ADR-5/6/7: no allocation, no locks, no blocking I/O at 1 kHz |

Issues labelled **good first issue** are scoped to one file and one test.

## The rules that are not negotiable

They exist because this repo is worked by agents as well as people, and gates
are cheaper than review:

1. **Reuse mature engines.** Physics is MuJoCo, kinematics Pinocchio/RTB, online
   trajectories Ruckig, vision OpenCV/ONNX, logging spdlog. A new dependency
   needs a seam we own — see `docs/ARCHITECTURE.md`.
2. **Extend a seam, never bypass it.** `scripts/check-boundaries.sh` fails the
   build on a cross-module include.
3. **The 1 kHz path stays real-time.** No Python, no heap allocation, no locks,
   no blocking I/O (ADR-5/6/7).
4. **Every tunable is data.** `config/robonode.settings.json`, not a literal in
   code (ADR-14). `scripts/check-design.sh` greps for the common violations.
5. **One contract, many surfaces.** Web UI, CLI and SDK are thin clients of the
   same facade (ADR-8). A capability in one surface but not another is a bug.
6. **Failures are loud.** A fallback is legitimate only when the degraded
   behaviour is *correct*; prefer a reported failure to a plausible wrong answer.
7. **Comments explain why, not what.** Small functions, intention-revealing
   names, modern C++/JS idioms.

Design decisions live in `docs/DECISIONS.md` as ADRs. If you disagree with one,
propose a new ADR in the PR — the architecture is meant to improve, not to be
frozen.

## Tests

Every capability adds a test at the right layer (`docs/TEST-STRATEGY.md`) and
flips its row in the scenario matrix.

```sh
ctest --preset fast    # the sub-2-second loop (dev preset)
ctest --preset sim     # everything, incl. the physics suites
npx playwright test    # browser journeys (needs a server running)
```

A test that cannot fail is not a test. If you fix a race or a crash, make the
test reproduce it first — `test_readers_do_not_race_the_physics` is the model.

## Pull requests

- One logical change per PR; a green `verify.sh` in the description.
- Explain the *why* in the body. Diffs show the what.
- Update `CHANGELOG.md` and, if you changed behaviour, the doc that documents it.
- By contributing you agree your work is licensed under Apache-2.0 (`LICENSE`).

## Reporting a bug

Include what you ran, what happened, and the log. `robonode --server
http://localhost:8080 logs` and `robonode --server … contacts` usually contain
the answer — a cell that stopped short will say what it is touching.
