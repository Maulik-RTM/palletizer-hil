"""eval2_branch_hold (P4): zero configuration-branch flips along a pick->place path.

EVAL_PROVENANCE['eval2_branch_hold'] = (P4, "desk: zero configuration flips along
any pick->place path"). A UR 6R IK has up to 8 branches; a carry must hold ONE
of them end-to-end (no elbow/wrist flip mid-move). Seed-chained IK along a dense
Cartesian path must never jump: an elbow/wrist flip shows up as a large joint
discontinuity between two neighbouring, nearly-coincident poses.

RED until `ur20_kinematics.ik` lands (plan.md phase 2).
"""
import numpy as np

from palhil.constitution import EVAL_PROVENANCE
from palhil.plant.ur20_kinematics import fk, ik

EVAL = "eval2_branch_hold"
MAX_JOINT_JUMP = np.deg2rad(30.0)   # a real branch flip is far larger than this


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


def _pick_to_place_path(steps=60):
    """approach-pick -> lift -> place, as a dense, fully continuous Cartesian
    path. Built by interpolating through reachable joint-space waypoints and
    taking FK of each sample: position AND orientation vary smoothly, so any
    large joint jump the IK produces is a genuine branch flip, not a discontinuity
    in the commanded path. The waypoint joints are the source of the path only --
    ik() re-solves each pose independently and must seed-chain one branch."""
    q_pick = np.array([0.4, -1.2, 1.2, -1.5, -1.5, 0.0])
    q_lift = np.array([0.4, -1.0, 0.8, -1.3, -1.5, 0.0])
    q_place = np.array([-0.6, -1.1, 1.0, -1.4, -1.5, 0.0])
    path = []
    for A, B in ((q_pick, q_lift), (q_lift, q_place)):
        for t in np.linspace(0.0, 1.0, steps, endpoint=False):
            path.append(fk((1.0 - t) * A + t * B))
    path.append(fk(q_place))
    return path


def test_no_branch_flip_along_pick_to_place():
    """eval2: seed-chained IK holds one branch -- neighbouring poses stay close."""
    seed = None
    prev = None
    max_jump = 0.0
    for T in _pick_to_place_path():
        q = np.asarray(ik(T, q_seed=seed))                  # NotImplementedError -> RED
        seed = q
        if prev is not None:
            max_jump = max(max_jump, float(np.max(np.abs(q - prev))))
        prev = q
    assert max_jump < MAX_JOINT_JUMP, (
        f"eval2 FAIL: branch flip -- {np.rad2deg(max_jump):.1f} deg joint jump")
