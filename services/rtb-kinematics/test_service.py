"""Self-contained checks that the real-robot kinematics behind the service is
sound (run directly, no server needed): pip install -r requirements.txt && python3 test_service.py"""
import numpy as np

import service


def test_fk_ik_roundtrip():
    r = service.get_robot("ur10")
    q = [0.2, -0.5, 0.6, 0.1, -0.3, 0.2]
    out = service.op_fk({"robot": "ur10", "q": q})
    assert out["ok"] and len(out["pos"]) == 3
    # plan a short reachable line and confirm the final waypoint's FK hits it
    target = (np.asarray(out["pos"]) + np.array([0.05, -0.03, 0.04])).tolist()
    plan = service.op_plan_move_l({"robot": "ur10", "q_start": q, "target": target, "steps": 20})
    assert plan["ok"], plan
    q_end = [plan["waypoints"][j][-1] for j in range(6)]
    p_end = service.fk(r, q_end).t
    assert np.linalg.norm(p_end - np.asarray(target)) < 1e-3


def test_jacobian_shape_and_unreachable():
    out = service.op_jacobian({"robot": "ur10", "q": [0] * 6})
    assert out["ok"] and out["dof"] == 6 and len(out["jac"]) == 18
    far = service.op_plan_move_l(
        {"robot": "ur10", "q_start": [0] * 6, "target": [10, 10, 10], "steps": 10})
    assert not far["ok"]


if __name__ == "__main__":
    test_fk_ik_roundtrip()
    test_jacobian_shape_and_unreachable()
    print("rtb-kinematics: all checks passed")
