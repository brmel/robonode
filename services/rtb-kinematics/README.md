# rtb-kinematics service

Real robot kinematics/planning for RoboNode, from a mature library — **not reimplemented**. Hosts [petercorke/robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python): real robot models (UR3/5/10, Panda, …) with validated FK, Levenberg–Marquardt IK, Jacobians, and straight-line Cartesian planning. The C++ `robonode::rtb` bridge ([bridges/rtb/](../../bridges/rtb/)) calls it over a small JSON HTTP RPC and exposes it through the motion **`Kinematics`** and **`Planner`** seams — so celld, the executive, and the adapters are unchanged. Non-RT (planning only); the 1 kHz streaming stays in C++.

```sh
pip install -r requirements.txt
python3 service.py --port 8091
python3 test_service.py            # self-contained FK/IK/moveL checks

# end-to-end (service + C++ bridge):
scripts/rtb-verify.sh
```

## Endpoints (POST JSON)

| path | in | out |
|---|---|---|
| `/fk` | `{robot,q}` | `{ok,pos:[x,y,z]}` |
| `/jacobian` | `{robot,q}` | `{ok,jac:[3·n row-major],dof}` |
| `/plan_move_l` | `{robot,q_start,target:[x,y,z],steps}` | `{ok,waypoints:[[per-joint]·n]}` — real IK along a straight TCP line, orientation held |
| `/health` | — | `{ok,robots:[…]}` |

## Adding robots

Extend `_FACTORY` in `service.py`. RTB models are one line each (`rtb.models.DH.UR10`, …); URDF-based robots (FANUC and other industrial arms) load via `rtb.ERobot.URDF(path)` — the **same seam**, still no reimplementation. See [docs/REAL-ROBOTS.md](../../docs/REAL-ROBOTS.md).
