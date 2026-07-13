"""eval7_pattern_fill (P8, P2): three pallets fill their patterns, cleanly.

EVAL_PROVENANCE['eval7_pattern_fill'] = (P8, P2, "desk: 3 pallets fill their
commanded patterns, no double-fill, full pallet blocks lane"). Four demands:
  - each of the 3 pallets fills its commanded pattern (lane i -> pallet i);
  - no slot is ever double-filled (each placed box owns exactly one slot, P8);
  - a full pallet BLOCKS its lane (no over-fill past capacity);
  - no lane starves (every pallet makes progress -- round-robin fairness).

The fill is expensive (270 boxes through one robot), so it runs ONCE in a
module-scoped fixture that closes the loop under the PLC-clock discipline (P1)
and asserts conservation (P8) every step; the four checks then read the result.

`placed_slots()` is a ground-truth accessor the plant provides for evals (P2).
"""
import pytest

from palhil.constitution import EVAL_PROVENANCE
from palhil.plant.cell_plant import KinematicPalletCell
from palhil.plc.cell_controller import MockCellController

from conftest import CAPACITY

EVAL = "eval7_pattern_fill"


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


@pytest.fixture(scope="module")
def filled_plant():
    """Close the mock loop until all three pallets top out (P1 clock discipline;
    P8 conservation asserted every step). Fails loudly if it can't fill in budget."""
    plant = KinematicPalletCell(seed=3)
    ctrl = MockCellController()
    prev = ctrl.time_ns()
    for i in range(800_000):
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
    assert all(plant.sense().pallet_full), "loop failed to fill all pallets in budget"
    return plant


def test_all_three_pallets_reach_capacity(filled_plant):
    counts = filled_plant.sense().pallet_count
    for i in range(3):
        assert counts[i] == CAPACITY, f"pallet {i}: {counts[i]}/{CAPACITY} filled"


def test_no_slot_is_double_filled(filled_plant):
    slots = filled_plant.placed_slots()                 # ground truth
    keys = [(s.pallet, s.layer, s.index) for s in slots]
    assert len(keys) == len(set(keys)), "a slot was filled twice (P8)"
    assert len(keys) == 3 * CAPACITY                    # exactly 90 per pallet, no more


def test_full_pallet_blocks_its_lane(filled_plant):
    s = filled_plant.sense()
    for i in range(3):
        assert s.pallet_full[i], f"pallet {i} full but not flagged"
        assert s.pallet_count[i] == CAPACITY, f"pallet {i} over-filled past capacity"


def test_no_lane_starves(filled_plant):
    counts = filled_plant.sense().pallet_count
    assert all(c > 0 for c in counts), f"a lane starved: {counts}"
