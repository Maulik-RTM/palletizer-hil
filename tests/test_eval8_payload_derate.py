"""eval8_payload_derate (B, P4): a loaded carry respects the derated limits.

EVAL_PROVENANCE['eval8_payload_derate'] = (B, P4, "desk: loaded-vs-empty carry
respects the derated limits"). Refinement B: joint speed scales with carried
mass -- a UR20 near 20 kg does not move like an empty one, and the PLANT enforces
the derate (the controller only plans within it). Under an identical large TCP
command, one cycle of loaded motion must cover less ground than empty, bounded by
the loaded/empty joint-speed ratio.

RED until `KinematicPalletCell` lands (plan.md phase 3).

`set_lane_box` / `lane_pick_pose` / `tcp_pose` (via sense) are ground-truth
accessors; the derate itself is the plant's to enforce, never the test's.
"""
import numpy as np

from palhil.constitution import EVAL_PROVENANCE
from palhil.interfaces import Commands
from palhil.plant.cell_plant import KinematicPalletCell
from palhil.plant.ur20_kinematics import (
    MAX_JOINT_SPEED_EMPTY,
    MAX_JOINT_SPEED_LOADED,
)

EVAL = "eval8_payload_derate"
CYCLE_S = 0.002


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


def test_derate_ratio_is_a_real_slowdown():
    """Sanity (green): the loaded cap is genuinely below the empty cap (B)."""
    assert MAX_JOINT_SPEED_LOADED < MAX_JOINT_SPEED_EMPTY


def _one_cycle_tcp_displacement(part_held: bool) -> float:
    plant = KinematicPalletCell(seed=0)                 # NotImplementedError -> RED
    if part_held:
        plant.set_lane_box(0, present=True)
        pick = plant.lane_pick_pose(0)
        for _ in range(400):                            # grab a box first
            plant.actuate(Commands(tcp_target=pick, vacuum_cmd=True))
            plant.step(CYCLE_S)
        assert plant.sense().part_held
    start = np.array(plant.sense().tcp_pose[:3])
    # move straight up 0.3 m -- far enough not to snap, and in reach from both the
    # home and the pick start (so the plant integrates rather than rejecting, P4).
    far = tuple(start + np.array([0.0, 0.0, 0.3])) + tuple(plant.sense().tcp_pose[3:])
    plant.actuate(Commands(tcp_target=far, vacuum_cmd=part_held, speed_scale=1.0))
    plant.step(CYCLE_S)
    return float(np.linalg.norm(np.array(plant.sense().tcp_pose[:3]) - start))


def test_loaded_carry_slower_than_empty():
    """eval8: identical command, loaded step covers less ground, per the derate."""
    d_empty = _one_cycle_tcp_displacement(part_held=False)
    d_loaded = _one_cycle_tcp_displacement(part_held=True)
    ratio = MAX_JOINT_SPEED_LOADED / MAX_JOINT_SPEED_EMPTY
    assert d_loaded < d_empty, "loaded carry was not derated (B)"
    assert d_loaded <= d_empty * ratio * 1.05, (
        f"loaded step {d_loaded:.4f} m exceeds derated bound {d_empty * ratio:.4f} m")
