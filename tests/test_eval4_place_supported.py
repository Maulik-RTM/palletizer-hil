"""eval4_place_supported (P3, P8): a place counts only if the box is SUPPORTED.

EVAL_PROVENANCE['eval4_place_supported'] = (P3, P8, "desk: box settles in its
slot, supported, z-err < tol"). A PLACE succeeds iff release happens with the box
within slot tolerance AND the box is supported (pallet deck or the layer below).
The teeth: a release over a MISSING support (an upper-layer slot with nothing
beneath it) must NOT count as placed -- the box does not float, and the ledger
does not credit the pallet.

RED until `KinematicPalletCell` + `column_pattern` land (plan.md phase 3).

Test hooks (`lane_pick_pose`, `set_lane_box`, `slot_world_pose`) are ground-truth
accessors the phase-3 plant provides for evals; never routed to the controller.
"""
import numpy as np

from palhil.constitution import EVAL_PROVENANCE
from palhil.interfaces import Commands
from palhil.plant.cell_plant import KinematicPalletCell, SLOT_TOL_M
from palhil.plant.pallet_pattern import column_pattern

from conftest import BOX_LWH_M, LAYERS, PALLET_LW_M

EVAL = "eval4_place_supported"
CYCLE_S = 0.002


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


def _picked_plant():
    """A plant holding one box on the gripper, ready to place on pallet 0."""
    plant = KinematicPalletCell(seed=0)              # NotImplementedError -> RED
    plant.set_lane_box(0, present=True)
    pick = plant.lane_pick_pose(0)
    for _ in range(400):
        plant.actuate(Commands(tcp_target=pick, vacuum_cmd=True))
        plant.step(CYCLE_S)
    assert plant.sense().part_held, "precondition: box must be on the gripper"
    return plant


def _place_at(plant, pose, max_steps=3000):
    """Carry to the slot holding vacuum, arrive, THEN release -- the real place
    protocol. (Releasing mid-air would correctly drop the box: vacuum off while
    not over a slot = a dropped box, not a placement.)"""
    for _ in range(max_steps):
        plant.actuate(Commands(tcp_target=pose, vacuum_cmd=True))
        plant.step(CYCLE_S)
        if np.linalg.norm(np.array(plant.sense().tcp_pose[:3]) - np.array(pose[:3])) < 1e-3:
            break
    for _ in range(20):
        plant.actuate(Commands(tcp_target=pose, vacuum_cmd=False))  # release at the slot
        plant.step(CYCLE_S)
    return plant.sense()


def _slots():
    return column_pattern(BOX_LWH_M, PALLET_LW_M, LAYERS)


def test_supported_bottom_slot_counts_as_placed():
    """Release into a deck-supported bottom-layer slot -> placed, z within tol."""
    plant = _picked_plant()
    bottom = next(s for s in _slots() if s.pallet == 0 and s.layer == 0)
    _place_at(plant, plant.slot_world_pose(bottom))
    assert plant.ledger().on_pallet == 1
    assert plant.sense().pallet_count[0] == 1


def test_release_over_missing_support_does_not_count():
    """Release into an upper-layer slot with nothing beneath -> NOT placed (P3/P8)."""
    plant = _picked_plant()
    upper = next(s for s in _slots() if s.pallet == 0 and s.layer == 1)
    _place_at(plant, plant.slot_world_pose(upper))
    assert plant.ledger().on_pallet == 0, "box floated: unsupported release counted"
    assert plant.sense().pallet_count[0] == 0
    assert plant.ledger().conserved()               # the box went somewhere accountable
