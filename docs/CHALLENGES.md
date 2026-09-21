# Challenges — hard scenarios, honest failure, one seam to fix

The platform exists to pose problems that are genuinely hard and let people
solve them by replacing **one algorithm**, in a sandbox, against real physics.
A challenge is not a demo: the naive answer must actually fail, visibly, for a
reason the operator can read in the log.

Every challenge is the same shape:

| | |
|---|---|
| **The cell** | descriptor data — nodes, robots, stations, motions |
| **The scenario** | an application (`apps/*.app.json`) that runs it |
| **The seam** | the capability whose version decides success |
| **The baseline** | a shipped version that *fails*, so the problem is real |
| **The fix** | another version — or your own, authored in the sandbox |

---

## C1 — Pick a moving target off a conveyor

**The problem.** A workpiece rides a belt at 0.08 m/s. Reaching for where the
part *is* takes the robot a few seconds, by which time the part has moved a
quarter of a metre. The gripper closes on empty belt.

**The cell.** `conveyor-1` drives a real belt in MuJoCo and the workpiece is a
**free body carried by friction** — nothing writes its pose, so it can be
intercepted, nudged, knocked off the line or dropped. The grasp is a weld the
model closes on the pose the tool actually caught it in. The physics runs
whether or not the robot is moving.

**What makes it hard** — the same three things that make it hard on a real line:

- **The sensor is late.** The camera runs at 20 fps with ~120 ms of exposure,
  transfer and processing. What a detector sees is always the past, so every
  sighting is stamped with its **capture time** and a tracker must extrapolate
  from *then*, not from now.
- **The sensor is noisy, and the part is not alone.** A decoy sits on the line:
  the same colour as the workpiece and larger. "Find the biggest red blob" locks
  onto it every time. The part's **known size**, projected through the camera
  model, is what tells them apart.
- **Aiming ahead changes the problem.** Lead further and the arm travels
  further, which takes longer, which needs more lead. The interception is a
  fixed point, not a formula.

**The pipeline, all swappable:**

```
camera → vision (OpenCV) → tracking (predict) → trajectory (plan) → control → tool
   late, noisy, cluttered     extrapolate        meet it          hold it
```

| Capability | Baseline that fails | Version that works | Author your own |
|---|---|---|---|
| `vision` | — | `robonode.opencv` — HSV threshold, largest contour, centroid, back-projected through the camera model | `x y z` → pose |
| `tracking` | **`robonode.snapshot`** — reports the last sighting, no motion model | `robonode.constant-velocity` — least-squares velocity over a window, with track gating | `x y z vx vy vz dt` → predicted `x y z` |
| `planner` | `robonode.moveL` (straight lines hit singularities on long transfers) | `robonode.moveJ` | `x y z` → via + end |
| `planner` (moving goals) | — | `robonode.rendezvous` — the goal carries the target's velocity, so the approach runs *with* the part rather than across it | — |
| `control` | — | `robonode.direct` · `robonode.smooth` | not authorable (ADR-5) |

**Run it.**

```sh
robonode app moving-bin-picking.app.json     # constant-velocity: catches it
robonode version tracking robonode.snapshot  # …now watch it fail
```

**What the failure looks like** — the log is the evidence:

```
intercept try 1/3: aiming 0.000m ahead at (1.100, 0.250, 0.370)
intercept try 1/3 missed — re-aiming with lead 4.14s
intercept try 2/3: aiming 0.000m ahead at (0.771, 0.250, 0.370)
intercept try 2/3 missed — re-aiming with lead 3.62s
app step 5/5 'intercept' failed: intercept: missed after 3 attempts
```

Against the same cell, the same app, with prediction:

```
intercept try 1/4: aiming 0.000m ahead at (1.097, 0.250, 0.370)
intercept try 1/4 missed — re-aiming with lead 3.89s
intercept try 2/4: aiming 0.318m ahead at (0.471, 0.250, 0.370)
tool: grasped
```

The first attempt always misses: a tracker has no velocity from one sighting.
That is the honest shape of the problem — **an interception has to learn how
long its own reach takes**, and `intercept` feeds each attempt's measured
duration back as the next attempt's lead time.

**Where the difficulty actually lives.** The lead time is a fixed point: aim
further ahead and the arm travels further, which takes longer, which needs more
lead. A better solution than the one shipped would solve for it rather than
iterate, and would use the belt geometry to pick the *interception point* rather
than a time.

**An open trade-off, measured.** `robonode.rendezvous` arrives travelling with
the part — the right idea for a moving grasp — but plans two segments, and on
this cell that **doubles the reach time** (7.7 s against 3.9 s for `moveJ`). The
extra lead then overshoots the belt entirely. The shipped app therefore uses
`moveJ`, and beating that is a real, open problem: a rendezvous that costs no
extra time would be strictly better than anything here.

**Write your own tracker** — web editor, or:

```sh
robonode define tracking my-lead "x + vx * dt * 1.2
y + vy * dt
z"
```

The host measures the sightings and the elapsed time; your program answers only
the interesting question. Output is validated (finite, and within a sane travel
of the last sighting) before any motion is planned against it, so a bad
prediction is refused rather than driven into the cell.

---

## Watching it happen

The dashboard shows the two numbers the interception turns on: the **detected
part pose** (vision) and the **estimated target speed** (tracking). Swap the
tracker to `robonode.snapshot` and the speed reads `0.000 m/s` — the reason the
arm keeps arriving late, visible before the first attempt even fails.

```sh
robonode capability tracking        # versions, provenance, live speed estimate
robonode capability vision          # what the camera sees, and when it saw it
robonode logs                       # every attempt, its lead, and why it missed
```

## Clutter: the shortest path is the one that hits things

`cluttered-line.scene.json` puts a crate between the robot and the far side of
its travel. A straight line from one side to the other passes through it — and
because the physics is real, the arm does not stop politely: it shoves the crate
into the carriage and stalls short of the goal.

```sh
robonode use-scene cluttered-line.scene.json
robonode version planner robonode.moveL
robonode movel 0.35 -0.05 0.25 && robonode contacts   # crate|wrist_2_link
robonode version planner robonode.avoid
robonode movel 0.35 -0.05 0.25                        # round it, or a refusal that says why
```

`robonode.avoid` asks the kinematics whether each configuration on the path
would be touching anything, and lifts over the obstacle when the direct line is
blocked, escalating the clearance until the whole arm is clear. It is not a
roadmap planner and it does not pretend to be: it guarantees only the half that
matters — **it never emits a path it knows is blocked** — and when the lift is
not enough it refuses, naming the clearance it tried, so the answer is
something you can act on.

That is the seam a real planner (cuRobo, OMPL — ) drops into: one line in the
registry, every caller unchanged.

## Stacking: the contact decides, not the plan

`stacking-line.scene.json` puts a block on the pallet's first slot;
`stacking.app.json` delivers a part, picks it, and places it in **slot 2** —
directly on top of that block.

What makes it hard is that nothing in the plan knows whether it worked. The
place succeeds the moment the tool opens; whether the part is *still there* a
second later is contact physics. A release from the approach height drops the
part the last few centimetres, and it lands off-centre, slides, and can topple
off — the log says `touching: base_block|part` when the stack held, and the
workpiece pose says where it actually ended up:

```sh
robonode use-scene stacking-line.scene.json
robonode app stacking.app.json
robonode contacts                    # base_block|part — or nothing, if it fell
```

The baseline that fails for the right reason is the default approach height: a
part released 6 cm above a 6 cm block has to survive the drop. Lower it in
settings (`app.approach_m`), or place with a slower control version, and the
stack survives more often. That is the experiment — and it is the one scenario
where the answer comes from the physics rather than from anything the platform
computed.

## A trade-off, not a winner: direct vs smooth control

The clearest thing this platform has measured is that "which algorithm is
better" is usually the wrong question.

`robonode.smooth` beats `robonode.direct` on an ideal axis — the plant answers
the setpoint instantly, so all that is left is the command, and smoothing it
strictly reduces the following error. `test_control_smooth_tracks_gentler_than_
direct_on_an_ideal_axis` measures exactly that.

On the cell that ships, it loses. Under inertia a smoothing controller lags
further through the acceleration phase, so its **peak** following error is
worse — 0.293 rad against 0.288 on the worst axis, repeatably.
`robonode compare control robonode.direct robonode.smooth` reports it, and
`test_smoothing_costs_peak_error_on_a_real_plant` pins it.

Neither version is broken. What changed is the plant, and a controller tuned for
one is not tuned for the other. The docs claimed "gentler follow" without
qualification for months while the platform's own comparison said otherwise —
which is the argument for a compare feature that reports how well a version RAN
rather than only whether it finished.

## Making the scene harder

The world is JSON, so a harder scenario is an edit, not a fork of the physics.
Open a scene, add what gets in the way, run it — on the running platform:

```sh
robonode scenes                                     # what you can start from
robonode fork scenes cluttered-line.scene.json mine.scene.json --session me
robonode scene mine.scene.json --session me         # edit: move the crate, add an obstacle
robonode use-scene mine.scene.json --session me     # live, no restart
```

In the browser it is the **Scenes** tab: pick one, change a number, Run scene.
Writes land in your session, so the scenario everyone else started from stays
intact — and a scene the composer cannot build is refused with the running
world left alone.

## Adding a challenge

1. Put the scenario in the **scene** (objects, obstacles, what moves) and the
   **cell descriptor** (a station, a motion) — not in code.
2. Write the **application** that runs it.
3. Ship a **baseline version that fails** for the right reason, and one that works.
4. Make the failure legible in the log; assert both outcomes in a test
   (`test_moving_target_needs_prediction_to_be_picked` is the model).

A challenge whose baseline quietly succeeds is a demo, and teaches nothing.
