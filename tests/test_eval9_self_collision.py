"""eval9_self_collision (P4, P8): the carried box clears the stack, and no two
boxes ever interpenetrate on a carry path.

EVAL_PROVENANCE['eval9_self_collision'] = (P4, P8, "desk: carried box clears the
stack via the pallet via-point; no box-box interpenetration on any carry path").

P4 puts every commanded pose ABOVE the collision floor -- the boxes already on
the pallet; P8 forbids bodies interpenetrating. The plant tracks
`collision_violations`: while a box is held, if its base dips below a placed
box's top within a box-width horizontally, that box was swept through.

Two demands, mirroring the eval3/eval4 shape:
  - POSITIVE: a full seeded fill under the via-point controller records ZERO
    collision violations (the LIFT_CARRY / APPROACH_PLACE / RETREAT_UP via-points
    keep every over-pallet traverse above the stack);
  - TEETH: a deliberate LOW traverse straight through a placed box IS flagged --
    proving the check has bite and the via-point is what earns the pass.

`set_lane_box` / `lane_pick_pose` / `slot_world_pose` are ground-truth accessors
the plant provides for evals; never routed to the controller (P2).
"""
import pytest

from palhil import geometry as geo
from palhil.constitution import EVAL_PROVENANCE
from palhil.interfaces import Commands
from palhil.plant.cell_plant import KinematicPalletCell
from palhil.plc.cell_controller import MockCellController

from conftest import CAPACITY

EVAL = "eval9_self_collision"
CYCLE_S = 0.002


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


# -- POSITIVE: a real fill never self-collides -------------------------------
@pytest.fixture(scope="module")
def filled_first_pallet():
    """Run the mock loop until the first pallet tops out (fill-then-switch), under
    PLC-clock discipline (P1), asserting conservation (P8) each step. Filling all
    five layers exercises many over-stack traverses -- the via-point must clear
    every one."""
    plant = KinematicPalletCell(seed=3)
    ctrl = MockCellController()
    prev = ctrl.time_ns()
    for _ in range(800_000):
        sensors = plant.sense()
        cmd = ctrl.step(sensors)
        now = ctrl.time_ns()
        dt = (now - prev) / 1e9
        prev = now
        plant.actuate(cmd)
        if dt > 0:
            plant.step(dt)
        assert plant.ledger().conserved(), "P8: ledger broke mid-run"
        if plant.sense().pallet_count[0] >= CAPACITY:
            break
    assert plant.sense().pallet_count[0] >= CAPACITY, "first pallet did not fill in budget"
    return plant


def test_no_collision_on_a_full_fill(filled_first_pallet):
    """eval9: the via-point controller clears the stack -- zero interpenetrations."""
    assert filled_first_pallet.collision_violations == 0, (
        f"{filled_first_pallet.collision_violations} carry(s) swept a placed box (P4/P8)")


# -- TEETH: a low traverse through a placed box is flagged --------------------
def _pick_one(plant, max_steps=2000):
    plant.set_lane_box(0, present=True)
    pick = plant.lane_pick_pose(0)
    for _ in range(max_steps):                   # travel varies with start pose
        plant.actuate(Commands(tcp_target=pick, vacuum_cmd=True))
        plant.step(CYCLE_S)
        if plant.sense().part_held:
            break
    assert plant.sense().part_held, "precondition: a box must be on the gripper"


def _carry_release(plant, pose, max_steps=3000):
    import numpy as np
    for _ in range(max_steps):
        plant.actuate(Commands(tcp_target=pose, vacuum_cmd=True))
        plant.step(CYCLE_S)
        if np.linalg.norm(np.array(plant.sense().tcp_pose[:3]) - np.array(pose[:3])) < 1e-3:
            break
    for _ in range(20):
        plant.actuate(Commands(tcp_target=pose, vacuum_cmd=False))
        plant.step(CYCLE_S)


def test_low_traverse_through_a_placed_box_is_flagged():
    """A carried box driven straight through a placed one at deck level must trip
    the collision floor (P4/P8) -- the negative that gives the eval teeth."""
    plant = KinematicPalletCell(seed=0)
    bottom = next(s for s in geo.PATTERN if s.pallet == 0 and s.layer == 0)
    wp = geo.slot_world_pose(bottom)

    _pick_one(plant)
    _carry_release(plant, wp)                    # place one box on the deck
    assert plant.ledger().on_pallet == 1

    _pick_one(plant)                             # grab a second box
    before = plant.collision_violations
    low = (wp[0], wp[1], wp[2], *geo.TOOL_DOWN)  # base level -- BELOW the placed box top
    for _ in range(1500):
        plant.actuate(Commands(tcp_target=low, vacuum_cmd=True))
        plant.step(CYCLE_S)
    assert plant.collision_violations > before, "a box swept through a placed one went undetected"
