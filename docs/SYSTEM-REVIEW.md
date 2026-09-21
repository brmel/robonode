# System design review — 2026-08-01

A pass over the whole platform asking one question of each part: *is this
separated because the design says so, or only because nobody has needed to
separate it yet?* Written down so the answer can be checked next time instead of
re-derived.

## The shape

```
core ← motion ← celld ← gateway ← apps
                          ↑
        engines (sim-mujoco · opencv · rtb · wasm)
```

Measured, not assumed — `git ls-files <dir> | xargs grep -oh '#include "robonode/…'`:

| layer | code | depends on |
|---|---|---|
| `core/` | 353 | itself |
| `motion/` | 2 190 | core |
| `celld/` | 1 218 | core, motion |
| `gateway/` | 4 414 | core, celld, motion, vision, sandbox |
| `apps/` | 4 389 | gateway (+ the vendor engines it chooses to link) |

Nothing points back up the stack. `mujoco.h` appears in exactly one file
(`engines/sim-mujoco/mujoco_world.hpp`) and `check-boundaries.sh` keeps it there.
The arrows are real.

## What is genuinely modular

- **The RT path.** `AxisAdapter`, `SetpointSource`, `Controller`, `Governor` are
  interfaces the executive drives; a driver is registered by name and selected
  from a descriptor. Swapping the physics driver for a soft-sim one is a string
  in a JSON file.
- **Capabilities.** Vision, tracking, trajectory and control are versions behind
  `ModuleRegistry`, described by `GET /capabilities`, swappable live, and
  authorable by a user in a sandbox. No surface hardcodes a capability id —
  `check-design.sh` fails the build if one does.
- **Documents.** Apps, scenes, modules, runs and robots are one store with one
  set of routes; a new kind is a row in `Workspace::kKinds`.
- **Surfaces.** Web, CLI and HTTP are thin clients of one facade. The live views
  are one table (`Platform::kViews`) that generates the routes AND the CLI verbs,
  so a view cannot exist on one surface and not the other.

## What leaks, and how much

**The physics engine was swappable in principle and not in practice.** The seams
(`Scene`, `Kinematics`) have always been abstract, but the gateway named
`MujocoScene`, `MujocoWorld` and `MujocoKinematics` directly and *stored the
concrete kinematics type*. Fixed as part of this review: `kin_` holds
`Kinematics`, and the engine choice is now two factory functions in one file.

What still names MuJoCo above the engine layer, and why it is harder:

| site | what it does | why it is not just a rename |
|---|---|---|
| `gateway/model_view.hpp` | loads a world to publish `GET /model` | the model JSON *is* a description of an MJCF world — links, geoms, sites |
| `gateway/cell_runtime.hpp` | includes `mjcf_composer.hpp` | scene composition writes MJCF XML; a second engine needs its own composer |

Both are honest engine-specific work, not accidental coupling. A second physics
engine is a real project (a second composer, a second model reader) — but it no
longer requires touching the parts of the gateway that are about *robots*.

**The log is one process-wide sink** with per-cell tagging (ADR-17). Correct for
one process; a multi-process deployment would need a real sink per cell.

## What a coherent v1 still needs

1. **No half-features.** Every control on every surface must do what it looks
   like it does. Three were found by audit and closed (`--server` was never
   exercised; the algorithm-editor's "Test against" was disabled with no reason
   given; and `fork`, `compare`, `add-cell` and `drop-cell` refused themselves as
   "local-only" while talking to a server that serves all four). The gates catch
   a markup control nothing reaches and a CLI verb that parses and dispatches
   nothing; the smoke run now walks the whole journey against a live server.
2. ~~**An e-stop that empties the queue.**~~ Closed. E-stop drops what has not
   started, reports those ids applied so nobody waits on them for ever, and says
   in the log how many went. A motion cancelled *because someone asked* is the
   stop working, not the motion failing; and a latched cell stays `held`, because
   the operator's decision outranks whatever the interrupted command has to say
   about itself.
3. ~~**`robonode-idl/proto/**` is a build input nothing builds.**~~ It is not a
   build input: its README has always said "design document, not a live
   contract". One sentence under that banner said the SDKs "is generated" from
   it in the present tense, which is what made it read as a broken build step.
   Fixed. `contracts/` remains the authoritative wire contract, validated
   against a live server.

## How to check this review is still true

```sh
scripts/check-boundaries.sh                  # the arrows
grep -rl "sim_mujoco" gateway/ | wc -l       # the engine leak: 4 files, falling
grep -c "^| J" docs/TEST-STRATEGY.md         # every row names its evidence
```
