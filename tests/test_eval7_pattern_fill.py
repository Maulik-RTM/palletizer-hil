"""eval7_pattern_fill (P8, P2): two pallets fill their patterns from one source.

EVAL_PROVENANCE['eval7_pattern_fill'] = (P8, P2, "desk: 2 pallets fill from one
source, no double-fill, full pallet blocks the lane"). Four demands:
  - both pallets fill their commanded pattern (the source feeds the active one,
    then switches to the other);
  - no slot is ever double-filled (each placed box owns exactly one slot, P8);
  - a full pallet BLOCKS placement past capacity (no over-fill);
  - both pallets make progress -- the single source is shared, not starved.

The fill is expensive (2 x 90 boxes through one robot), so it runs ONCE in a
module-scoped fixture that closes the loop under the PLC-clock discipline (P1)
and asserts conservation (P8) every step; the four checks then read the result.

`placed_slots()` is a ground-truth accessor the plant provides for evals (P2).
"""
import pytest

from palhil import geometry as geo
from palhil.constitution import EVAL_PROVENANCE
from palhil.plant.cell_plant import KinematicPalletCell
from palhil.plc.cell_controller import MockCellController

from conftest import CAPACITY

EVAL = "eval7_pattern_fill"


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


@pytest.fixture(scope="module")
def filled_plant():
    """Close the mock loop until both pallets top out (P1 clock discipline;
    P8 conservation asserted every step). Fails loudly if it can't fill in budget."""
    plant = KinematicPalletCell(seed=3)
    ctrl = MockCellController()
    prev = ctrl.time_ns()
    for i in range(1_200_000):
        sensors = plant.sense()
        cmd = ctrl.step(sensors)
        now = ctrl.time_ns()
        dt = (now - prev) / 1e9
        prev = now
        plant.actuate(cmd)
        if dt > 0:
            plant.step(dt)
        assert plant.ledger().conserved(), "P8: ledger broke mid-run"
        if i % 500 == 0 and all(plant.sense().pallet_full):
            break
    assert all(plant.sense().pallet_full), "loop failed to fill both pallets in budget"
    return plant


def test_both_pallets_reach_capacity(filled_plant):
    counts = filled_plant.sense().pallet_count
    for i in range(geo.N_PALLETS):
        assert counts[i] == CAPACITY, f"pallet {i}: {counts[i]}/{CAPACITY} filled"


def test_no_slot_is_double_filled(filled_plant):
    slots = filled_plant.placed_slots()                 # ground truth
    keys = [(s.pallet, s.layer, s.index) for s in slots]
    assert len(keys) == len(set(keys)), "a slot was filled twice (P8)"
    assert len(keys) == geo.N_PALLETS * CAPACITY        # exactly 90 per pallet, no more


def test_full_pallet_blocks_over_fill(filled_plant):
    s = filled_plant.sense()
    for i in range(geo.N_PALLETS):
        assert s.pallet_full[i], f"pallet {i} full but not flagged"
        assert s.pallet_count[i] == CAPACITY, f"pallet {i} over-filled past capacity"


def test_both_pallets_filled_from_one_source(filled_plant):
    counts = filled_plant.sense().pallet_count
    assert all(c > 0 for c in counts), f"a pallet got no boxes: {counts}"
