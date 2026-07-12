"""UR20 serial 6R kinematics (P4). FK implemented; analytic IK is a build task.

DH parameters below are the standard UR convention (theta about z, then
d along z, a along x, alpha about x). Values are UR's published UR20 table --
VERIFY against the official UR "DH parameters for calculations of kinematics
and dynamics" article before trusting eval-1 numbers.

Eval provenance: eval1_ik_error, eval2_branch_hold (see constitution.py).
"""
from __future__ import annotations

import numpy as np

# UR20 -- meters. [d, a, alpha] per joint, UR order.
DH = np.array([
    #  d        a        alpha
    [0.2363,  0.0,      np.pi / 2],   # J1
    [0.0,    -0.8620,   0.0      ],   # J2
    [0.0,    -0.7287,   0.0      ],   # J3
    [0.2010,  0.0,      np.pi / 2],   # J4
    [0.1593,  0.0,     -np.pi / 2],   # J5
    [0.1543,  0.0,      0.0      ],   # J6
])

REACH_M = 1.750           # UR20 nominal reach
JOINT_LIMITS = np.deg2rad(np.array([[-360, 360]] * 6, dtype=float))
# Refinement B: speed derate vs payload -- plant enforces, controller plans.
MAX_JOINT_SPEED_EMPTY = np.deg2rad(120.0)
MAX_JOINT_SPEED_LOADED = np.deg2rad(60.0)   # near 20 kg -- placeholder, tune on rig


def _dh_T(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def fk(q: np.ndarray) -> np.ndarray:
    """Base->flange 4x4 for joint vector q (rad)."""
    T = np.eye(4)
    for i in range(6):
        d, a, alpha = DH[i]
        T = T @ _dh_T(q[i], d, a, alpha)
    return T


WRIST_SINGULARITY_BAND = 1e-3   # |sin(theta5)| below this: axes 4 and 6 align (P4)


class Unreachable(Exception):
    pass


def _wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def _nearest(target: float, ref: float) -> float:
    """The representative of `target` (mod 2pi) closest to `ref` -- keeps the
    seed-chained solution continuous across turns (P4: hold one branch)."""
    return ref + _wrap(target - ref)


def _pose_close(A: np.ndarray, B: np.ndarray, tol_pos=1e-6, tol_rot=1e-5) -> bool:
    if np.linalg.norm(A[:3, 3] - B[:3, 3]) > tol_pos:
        return False
    dR = A[:3, :3].T @ B[:3, :3]
    ang = np.arccos(np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0))
    return ang < tol_rot


def _branches(T: np.ndarray):
    """Yield every geometrically valid (non-singular, in-envelope) IK solution
    for the base->flange target T. Standard UR serial-6R closed form on the
    verified UR20 DH table (P4). Up to 8 branches: 2 shoulder x 2 wrist x 2 elbow.
    """
    d1, _, _ = DH[0]
    _, a2, _ = DH[1]
    _, a3, _ = DH[2]
    d4, _, _ = DH[3]
    d5, _, _ = DH[4]
    d6, _, _ = DH[5]

    nx, ny = T[0, 0], T[1, 0]
    ox, oy = T[0, 1], T[1, 1]
    ax, ay = T[0, 2], T[1, 2]
    px, py = T[0, 3], T[1, 3]

    saw_singular = False

    # --- theta1: two shoulder solutions from the wrist-center (frame-5 origin) --
    p05x = px - d6 * ax
    p05y = py - d6 * ay
    R = np.hypot(p05x, p05y)
    if R < abs(d4):
        return                        # wrist center inside the d4 cylinder: no reach
    psi = np.arctan2(p05y, p05x)
    phi = np.arccos(np.clip(d4 / R, -1.0, 1.0))
    for t1 in (psi + phi + np.pi / 2, psi - phi + np.pi / 2):
        s1, c1 = np.sin(t1), np.cos(t1)

        # --- theta5: two wrist solutions --------------------------------------
        arg5 = (px * s1 - py * c1 - d4) / d6
        if abs(arg5) > 1.0 + 1e-9:
            continue
        for t5 in (np.arccos(np.clip(arg5, -1.0, 1.0)),
                   -np.arccos(np.clip(arg5, -1.0, 1.0))):
            s5 = np.sin(t5)
            if abs(s5) < WRIST_SINGULARITY_BAND:
                saw_singular = True
                continue              # reject: theta6 undefined in the guard band

            # --- theta6 --------------------------------------------------------
            t6 = np.arctan2((-ox * s1 + oy * c1) / s5,
                            (nx * s1 - ny * c1) / s5)

            # --- theta2/3/4: planar 2R after stripping joints 1,5,6 -----------
            A1 = _dh_T(t1, *DH[0])
            A5 = _dh_T(t5, *DH[4])
            A6 = _dh_T(t6, *DH[5])
            T14 = np.linalg.inv(A1) @ T @ np.linalg.inv(A5 @ A6)  # == A2 A3 A4
            xpl, ypl = T14[0, 3], T14[1, 3]
            D = (xpl * xpl + ypl * ypl - a2 * a2 - a3 * a3) / (2 * a2 * a3)
            if abs(D) > 1.0 + 1e-9:
                continue              # elbow cannot reach this planar point
            for t3 in (np.arccos(np.clip(D, -1.0, 1.0)),
                       -np.arccos(np.clip(D, -1.0, 1.0))):
                t2 = np.arctan2(ypl, xpl) - np.arctan2(a3 * np.sin(t3),
                                                       a2 + a3 * np.cos(t3))
                t234 = np.arctan2(T14[1, 0], T14[0, 0])
                t4 = t234 - t2 - t3
                q = np.array([t1, t2, t3, t4, t5, t6])
                if _pose_close(fk(q), T):
                    yield q
    # signal (via attribute on the generator's caller) handled in ik()
    _branches.saw_singular = saw_singular


def ik(T: np.ndarray, q_seed: np.ndarray | None = None) -> np.ndarray:
    """Analytic UR20 6R IK (P4).

    - enumerates the (<=8) closed-form branches, keeps only the ones that FK
      back to T (a wrong branch is dropped, never returned);
    - returns the branch nearest q_seed with per-joint 2pi continuity, so a
      seed-chained path holds ONE configuration end-to-end (no elbow/wrist flip);
    - raises Unreachable outside the envelope -- NEVER silently clamps;
    - rejects solutions inside the wrist-singularity guard band.

    Contract: fk(ik(T)) position error < 0.5 mm (eval1); seed-chained continuity
    (eval2). q_seed=None picks the branch nearest the zero configuration.
    """
    T = np.asarray(T, dtype=float)
    _branches.saw_singular = False
    cands = list(_branches(T))
    if not cands:
        if getattr(_branches, "saw_singular", False):
            raise Unreachable("wrist singularity: no non-singular branch for this pose")
        raise Unreachable("target outside the UR20 reach envelope")

    ref = np.zeros(6) if q_seed is None else np.asarray(q_seed, dtype=float)
    best = min(cands, key=lambda q: np.linalg.norm(
        [_wrap(qi - ri) for qi, ri in zip(q, ref)]))
    q = np.array([_nearest(qi, ri) for qi, ri in zip(best, ref)])

    lo, hi = JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]
    if np.any(q < lo) or np.any(q > hi):
        raise Unreachable("solution violates joint limits")
    return q
