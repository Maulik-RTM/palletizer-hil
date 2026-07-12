"""eval3_pick_coincidence (P3, P2): a pick latches ONLY on the joint condition.

EVAL_PROVENANCE['eval3_pick_coincidence'] = (P3, P2, "desk: pick latches only on
pose AND vacuum-window AND presence"). The plant -- never the controller (P2) --
adjudicates the grasp: a PICK succeeds iff |tcp - box| < tol AND vacuum is
commanded within the dwell window AND a box is present at the lane stop. The
teeth of this eval are the three negative cases: EACH condition alone must FAIL
to latch.

RED until `KinematicPalletCell` lands (plan.md phase 3) -- its __init__ raises
NotImplementedError today.

Test hooks used (`set_lane_box`, `lane_pick_pose`) are ground-truth accessors the
phase-3 plant provides for evals; they are never routed to the controller (P2).
"""
import pytest

from palhil.constitution import EVAL_PROVENANCE
from palhil.interfaces import Commands
from palhil.plant.cell_plant import KinematicPalletCell

EVAL = "eval3_pick_coincidence"
CYCLE_S = 0.002


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


def _fresh(box_present=True):
    plant = KinematicPalletCell(seed=0)              # NotImplementedError -> RED
    plant.set_lane_box(0, present=box_present)       # ground truth, never to controller
    return plant


def _settle(plant, cmd, steps=400):
    for _ in range(steps):
        plant.actuate(cmd)
        plant.step(CYCLE_S)
    return plant.sense()


def test_pick_latches_on_pose_and_vacuum_and_presence():
    """All three conditions met -> the box latches to the gripper."""
    plant = _fresh(box_present=True)
    pick = plant.lane_pick_pose(0)
    s = _settle(plant, Commands(tcp_target=pick, vacuum_cmd=True))
    assert s.part_held and plant.ledger().on_gripper == 1


def test_pose_alone_without_vacuum_does_not_latch():
    plant = _fresh(box_present=True)
    pick = plant.lane_pick_pose(0)
    s = _settle(plant, Commands(tcp_target=pick, vacuum_cmd=False))
    assert not s.part_held and plant.ledger().on_gripper == 0


def test_vacuum_alone_off_pose_does_not_latch():
    plant = _fresh(box_present=True)
    pick = plant.lane_pick_pose(0)
    far = (pick[0] + 0.20,) + tuple(pick[1:])        # well outside POS_TOL
    s = _settle(plant, Commands(tcp_target=far, vacuum_cmd=True))
    assert not s.part_held and plant.ledger().on_gripper == 0


def test_presence_alone_missing_box_does_not_latch():
    plant = _fresh(box_present=False)                # empty lane stop
    pick = plant.lane_pick_pose(0)
    s = _settle(plant, Commands(tcp_target=pick, vacuum_cmd=True))
    assert not s.part_held and plant.ledger().on_gripper == 0
