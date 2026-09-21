#!/usr/bin/env python3
"""RoboNode kinematics/planning service — real robot models via Robotics Toolbox.

Mature-library brains behind our node seams: petercorke/robotics-toolbox-python
supplies validated FK/IK/Jacobian and real robot models (UR3/5/10, Panda, ...).
The C++ Kinematics/Planner seams call this over a small JSON HTTP RPC (non-RT,
planning only). We do not reimplement kinematics — we host the real thing.

Endpoints (POST JSON):
  /fk           {robot, q}                         -> {ok, pos:[x,y,z]}
  /jacobian     {robot, q}                          -> {ok, jac:[3*n row-major]}
  /plan_move_l  {robot, q_start, target:[x,y,z], steps}
                                                    -> {ok, waypoints:[[per-joint]*n]}
  /health       {}                                  -> {ok, robots:[...]}

Run:  python3 service.py [--port 8091]
"""
import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# Real robot models by name → constructor. DH models (light, no meshes); URDF
# variants (with meshes) can be added the same way. FANUC/others load from
# their URDF via rtb.ERobot.URDF — same seam, no reimplementation.
_FACTORY = {
    "ur10": rtb.models.DH.UR10,
    "ur5": rtb.models.DH.UR5,
    "ur3": rtb.models.DH.UR3,
    "panda": rtb.models.DH.Panda,
}
_cache = {}
_lock = threading.Lock()


def get_robot(name):
    key = (name or "ur10").lower()
    if key not in _FACTORY:
        raise KeyError(f"unknown robot '{name}' (have {sorted(_FACTORY)})")
    with _lock:
        if key not in _cache:
            _cache[key] = _FACTORY[key]()
        return _cache[key]


def fk(robot, q):
    return robot.fkine(np.asarray(q, dtype=float))


def op_fk(req):
    r = get_robot(req["robot"])
    return {"ok": True, "pos": fk(r, req["q"]).t.tolist()}


def op_jacobian(req):
    r = get_robot(req["robot"])
    J = r.jacob0(np.asarray(req["q"], dtype=float))[:3, :]  # position rows
    return {"ok": True, "jac": J.flatten().tolist(), "dof": r.n}


def op_plan_move_l(req):
    """Straight TCP line from fk(q_start) to target, orientation held constant.
    Real IK per step (RTB Levenberg–Marquardt). Returns joint waypoints
    [joint][step] for our SyncBlendPlan. Fails closed if any step is unreachable."""
    r = get_robot(req["robot"])
    q = np.asarray(req["q_start"], dtype=float)
    T0 = fk(r, q)
    p0 = T0.t
    target = np.asarray(req["target"], dtype=float)
    steps = int(req.get("steps", 25))
    R = T0.R  # keep starting orientation (a real moveL)

    wp = [[float(v)] for v in q]
    for s in range(1, steps + 1):
        p = p0 + (target - p0) * (s / steps)
        Tep = SE3.Rt(R, p)
        sol = r.ikine_LM(Tep, q0=q)
        if not sol.success:
            return {"ok": False, "error": f"unreachable at step {s}"}
        q = sol.q
        for j in range(r.n):
            wp[j].append(float(q[j]))
    return {"ok": True, "waypoints": wp, "dof": r.n}


_OPS = {"/fk": op_fk, "/jacobian": op_jacobian, "/plan_move_l": op_plan_move_l}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # quiet
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == "/health":
            return self._send(200, {"ok": True, "robots": sorted(_FACTORY)})
        op = _OPS.get(self.path)
        if op is None:
            return self._send(404, {"ok": False, "error": "unknown endpoint"})
        try:
            n = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(n) or b"{}")
            self._send(200, op(req))
        except Exception as e:  # noqa: BLE001
            self._send(400, {"ok": False, "error": str(e)})

    def do_GET(self):
        if self.path == "/health":
            return self._send(200, {"ok": True, "robots": sorted(_FACTORY)})
        self._send(404, {"ok": False, "error": "POST only"})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8091)
    args = ap.parse_args()
    get_robot("ur10")  # warm the default
    print(f"rtb-kinematics on http://localhost:{args.port}  robots={sorted(_FACTORY)}")
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
