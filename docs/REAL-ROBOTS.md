# Real robots — using mature libraries, not reinventing

> Two aspects of "real", both behind existing seams so the node design is untouched.
>
> 1. **Real physics model** — **done**. The cell runs a real UR10e from MuJoCo Menagerie (menagerie geometry verbatim, local renames only), on a 7th-axis rail, with contacts and a weld-based grasp (ADR-16).
> 2. **Real kinematics** — via Robotics Toolbox *offline*, **open for the RT path**.
>
> **Why the RT path is still open.** Querying the Python RTB service from the 1 kHz loop is exactly the flaw ADR-5 forbids: IPC plus the GIL cannot meet the deadline. So the RT implementation is to be **Pinocchio (C++, in-process)** behind the *same* `Kinematics` seam, and RTB stays the offline model/URDF source and the Tier-C planner. Today the loop uses a hand-rolled damped-least-squares solve on MuJoCo's own Jacobian. It answers full SE(3) — `move_pose` reaches a point *and* an angle (, closed) — and what Pinocchio buys is a maintained implementation of it rather than ours. When Pinocchio lands, nothing above the seam changes. That is the seam paying rent.

## Implemented — real kinematics via Robotics Toolbox

Decision (chosen): host [petercorke/robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python) — real robot models (UR3/5/10, Panda, …) + validated FK/IK/Jacobian/trajectories — in a small Python service, and reach it through our seams:

```
C++ celld / motion (RT, unchanged)
   Planner seam    ──► RtbPlanner    ─┐  JSON/HTTP    ┌─► Python rtb-kinematics
   Kinematics seam ──► RtbKinematics ─┘  (non-RT)     └─►   Robotics Toolbox
DriverRegistry / AxisAdapter / SyncBlendPlan — all unchanged        real models + IK
```

- **No hand-rolled kinematics.** `engines/rtb` implements the motion `Kinematics`/`Planner` interfaces against the service; the RT streaming loop stays pure C++.
- **Verified:** `examples/rtb_dev` plans a real UR10 Cartesian moveL via the service — 6 joints × N real IK waypoints, FK lands on target to 1e-6 m. `services/rtb-kinematics/test_service.py` self-checks the library; `scripts/rtb-verify.sh` runs the full C++↔service loop.
- **FANUC / other industrial arms** load into RTB from their URDF (`rtb.ERobot.URDF`) — the same seam, still no reimplementation.

## Next — real physics model on the rail

The physics twin (MuJoCo) still uses the primitive arm. To make the *simulation* a real robot too, load a real model so MuJoCo simulates the same robot RTB plans for. Survey + plan below.

---

## Real robot-model repos (for the physics twin)

> Goal: replace the hand-built primitive UR10e MJCF with **real, open-source robot models** so the simulator's inertias and dynamics match hardware. The `AxisAdapter` / `DriverRegistry` / descriptor seams already make this a drop-in.

## Survey of open-source robot-model repos

| Repo | What it is | Format | Fit for us |
|---|---|---|---|
| **[google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)** | DeepMind-curated, high-quality MuJoCo models — **17 robot arms**, real inertias, meshes, calibrated actuators. Apache-2.0. | **MJCF** (native) | ⭐ **Best fit** — our sim IS MuJoCo; zero conversion. Arms: UR5e/UR10e (6-DOF), Franka FR3 · Kuka iiwa14 · Kinova Gen3 · Sawyer (7-DOF), and more. |
| [robot_descriptions.py](https://github.com/robot-descriptions/robot_descriptions.py) | Python loader that fetches 100+ robot descriptions (incl. menagerie) on demand. | URDF + MJCF | Useful as an index; menagerie is the underlying MJCF source. |
| [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) | The canonical UR **URDF** + kinematics. | URDF | Reference for DH/limits; needs URDF→MJCF (menagerie already did this). |
| [frankaemika/franka_ros](https://github.com/frankaemika/franka_ros), Kuka/Kinova ROS pkgs | Vendor URDFs. | URDF | Same — menagerie already provides clean MJCF versions. |

### Industrial arms (the real target: FANUC, Kawasaki RS, ABB, Doosan)

The brands Vention actually integrates are **not** in menagerie (menagerie is mostly collaborative/research arms). They ship as **URDF** — and **MuJoCo loads URDF directly** (`mj_loadXML` on the URDF, or the `compile` utility), so they're reachable with mesh-path + actuator fixups.

| Brand / model | Repo | Notes |
|---|---|---|
| **FANUC** LR Mate 200iD, M-10iA, M-20iA (6-DOF) | **official** [FANUC-CORPORATION/fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description) · community [ros-industrial/fanuc](https://github.com/ros-industrial/fanuc) | URDF + STL meshes. FANUC publishes these themselves. |
| **Kawasaki RS** (RS005L, RS007…) — likely the "RS5" | [ros-industrial/kawasaki_experimental](https://github.com/ros-industrial/kawasaki_experimental) | RS-series 6-DOF industrial. |
| **ABB** IRB series | [ros-industrial/abb_experimental](https://github.com/ros-industrial/abb) | URDF. |
| **Doosan** M/H/A series | [doosan-robotics/doosan-robot](https://github.com/doosan-robotics/doosan-robot) | URDF. |

**URDF → MuJoCo caveats (the real work):** MuJoCo doesn't resolve `package://` URIs (rewrite mesh paths to a `meshdir`); DAE meshes unsupported (use the STL/OBJ variants, which ros-industrial ships); URDFs carry **no actuators** — we inject one position servo per joint (our descriptor already defines each joint's limits, so this is mechanical). Inertias come from the vendor URDF (real).

**Decision (two tracks):**
1. **Fast-start / prove the pipeline:** mujoco_menagerie **UR10e** (MJCF, zero conversion) — real dynamics, and it's a robot Vention uses. Lands the compose-on-rail machinery.
2. **The real target:** **FANUC** (official `fanuc_description`) via URDF import, then Kawasaki RS / ABB / Doosan by the same recipe. This is what makes the twin a real *industrial* robot.

Both tracks are 6+ DOF and reuse every node seam unchanged.

## Why this fits our node design unchanged

- Each robot joint is already a **MotionAxis node** driven by `MujocoAxisAdapter` (joint + actuator name from descriptor `config`). A real menagerie model just supplies different — accurate — joint/actuator names, limits, and inertias. The descriptor's `limits` become the *real* robot's limits (data, FR-1.3).
- `MujocoKinematics` computes FK/Jacobian from whatever model is loaded → real kinematics for free ( cross-checks still hold).
- The rail (7th axis) stays ours; the real arm mounts on the carriage.

## The one technical problem: mounting a real arm on our rail

Menagerie arms are standalone models (`ur10e.xml` + `assets/`, `meshdir="assets"`). We need the arm's base to be a **child of the rail carriage** so the 7th axis carries it. Options:

1. **`mjSpec` composition (chosen).** MuJoCo 3.2+ exposes a model-spec C API: parse the rail-base spec and the arm spec, `mjs_attach` the arm's base body under the carriage body (with a name prefix), `mj_compile`. Programmatic, robust, version-clean, and keeps the menagerie file pristine (we never edit vendored XML). Implemented in `MujocoWorld::load_composed(...)`.
2. XML `<attach>` — works but merges at parse time and is fiddlier to keep the vendored file untouched.
3. Hand-merging bodies — brittle, rejected.

Meshes/LFS: menagerie meshes are plain files (no git-lfs), fetchable by sparse checkout — verified earlier. FetchContent (sparse) pulls only the chosen arm.

## Plan (this branch)

**Track 1 — menagerie UR10e (prove the pipeline):**
- [ ] Probe `mjSpec` attach in MuJoCo 3.10: compose a rail base + real UR10e, load, list real joints/actuators. *(de-risk)*
- [ ] `sim-mujoco`: FetchContent menagerie (sparse: `universal_robots_ur10e`); `MujocoWorld::load_composed(base_xml, arm_xml, mount_body, prefix)`.
- [ ] Rail-base MJCF (7th axis + mount site) the real arm attaches to.
- [ ] Descriptors reference the real joint/actuator names; executive drives them on one clock; `cell_server` shows the **real** UR10e moving.

**Track 2 — FANUC (the real industrial target):**
- [ ] Probe: fetch official `fanuc_description` (LR Mate 200iD), rewrite `package://` mesh paths → `meshdir`, `mj_loadXML` the URDF, confirm it loads with real inertias/meshes. *(de-risk — the URDF-import risk)*
- [ ] Actuator injection: add a position servo per joint (limits from our descriptor) — a small URDF→MJCF post-step or an `<mujoco>` block.
- [ ] Same compose-on-rail + descriptor mapping as Track 1, driven by `robonode.mujoco-axis` unchanged.
- [ ] Repeat the recipe for Kawasaki RS / ABB / Doosan.

**Both:** `MujocoKinematics` FK matches the real model (kinematic test, servo-independent); web 3D viewer either renders the real meshes or keeps the schematic (physics underneath is real either way).

Sources: [menagerie](https://github.com/google-deepmind/mujoco_menagerie), [menagerie README](https://github.com/google-deepmind/mujoco_menagerie/blob/main/README.md), [robot_descriptions.py](https://github.com/robot-descriptions/robot_descriptions.py), [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot), [MuJoCo model gallery](https://mujoco.readthedocs.io/en/stable/models.html).

## Running against URSim

`scripts/ursim.sh up | down | status` brings a URSim container up, waits for its
dashboard and powers the robot on — the rig the UR adapter was written against.
On Apple silicon the image is amd64-only and PolyScope boots under emulation, so
the first `up` can take 15–40 minutes.

Nothing in CI calls it: it needs a container and a lot of patience. It is here
because the alternative to a documented manual rig is an undocumented one.
