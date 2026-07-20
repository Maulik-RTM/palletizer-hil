"""The placeholder cell layout is REACH-VERIFIED (P4): every pose the controller
can command -- lane picks/approaches/carries, home, and all 270 pallet slots plus
their approaches -- lies inside the UR20 envelope AND actually solves IK with the
tool-down orientation. This is the guard behind geometry.py's "reach-verified
placeholder" claim: if someone nudges the layout out of reach, this fails loudly
instead of surfacing as a mysterious plant reach-violation in the phase-3 loop.

Not a physics/collision check -- see the ENGINEERING PLACEHOLDER banner in
geometry.py. Cites P4 (reach envelope) and the fk/ik contract (P4)."""
import numpy as np

from palhil import geometry as geo
from palhil.plant.ur20_kinematics import Unreachable, fk, ik


def _all_commanded_poses():
    poses = {"home": geo.home_pose()}
    for i in range(geo.N_LANES):
        poses[f"lane{i}_pick"] = geo.lane_pick_pose(i)
        poses[f"lane{i}_approach"] = geo.lane_approach_pose(i)
        poses[f"lane{i}_carry"] = geo.carry_pose_over(geo.LANE_STOP_XYZ[i])
    for s in geo.PATTERN:
        tag = f"p{s.pallet}_l{s.layer}_i{s.index}"
        poses[f"slot_{tag}"] = geo.slot_world_pose(s)
        poses[f"approach_{tag}"] = geo.slot_approach_pose(s)
        poses[f"clear_{tag}"] = geo.slot_clear_pose(s)   # pallet-clearing via-point
    return poses


def _rotvec_to_T(pose):
    x, y, z, rx, ry, rz = pose
    v = np.array([rx, ry, rz], float)
    th = np.linalg.norm(v)
    if th < 1e-12:
        R = np.eye(3)
    else:
        k = v / th
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (x, y, z)
    return T


def test_every_commanded_pose_is_within_the_reach_limit():
    poses = _all_commanded_poses()
    over = {k: float(np.linalg.norm(p[:3]))
            for k, p in poses.items()
            if np.linalg.norm(p[:3]) > geo.REACH_LIMIT_M}
    assert not over, f"{len(over)} target(s) beyond REACH_LIMIT_M: {list(over)[:5]}"


def test_ik_solves_for_lane_picks_and_every_slot():
    worst = 0.0
    for i in range(geo.N_LANES):
        _check_ik(geo.lane_pick_pose(i), worst)
    for s in geo.PATTERN:
        worst = _check_ik(geo.slot_world_pose(s), worst)
    assert worst < 0.5e-3, f"IK round-trip {worst * 1e3:.3f} mm on a commanded slot"


def _check_ik(pose, worst):
    T = _rotvec_to_T(pose)
    try:
        q = ik(T)
    except Unreachable as e:  # pragma: no cover - fails the suite if it ever fires
        raise AssertionError(f"commanded pose is Unreachable: {pose} ({e})")
    return max(worst, float(np.linalg.norm(fk(q)[:3, 3] - T[:3, 3])))
