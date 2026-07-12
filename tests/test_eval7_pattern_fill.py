"""eval7_pattern_fill (P8, P2): three pallets fill their patterns, cleanly.

EVAL_PROVENANCE['eval7_pattern_fill'] = (P8, P2, "desk: 3 pallets fill their
commanded patterns, no double-fill, full pallet blocks lane"). Four demands:
  - each of the 3 pallets fills its commanded pattern (lane i -> pallet i);
  - no slot is ever double-filled (each placed box owns exactly one slot, P8);
  - a full pallet BLOCKS its lane (no over-fill past capacity);
  - no lane starves (every pallet makes progress -- round-robin fairness).

RED until `MockCellController` + `KinematicPalletCell` + `column_pattern` land.

`placed_slots()` is a ground-truth accessor the phase-3 plant provides for evals.
"""
from palhil.constitution import EVAL_PROVENANCE
from palhil.plant.cell_plant import KinematicPalletCell
from palhil.plant.pallet_pattern import column_pattern
from palhil.plc.cell_controller import MockCellController

from conftest import BOX_LWH_M, LAYERS, PALLET_LW_M

EVAL = "eval7_pattern_fill"


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


def _capacity_per_pallet():
    slots = column_pattern(BOX_LWH_M, PALLET_LW_M, LAYERS)
    return sum(1 for s in slots if s.pallet == 0)


def _run_until_full(run_cell):
    plant = KinematicPalletCell(seed=3)                 # NotImplementedError -> RED
    ctrl = MockCellController()
    # long enough to top out all three pallets; phase 3 tunes the exact horizon.
    run_cell(plant, ctrl, steps=120_000)
    return plant


def test_all_three_pallets_reach_capacity(run_cell):
    plant = _run_until_full(run_cell)
    cap = _capacity_per_pallet()
    counts = plant.sense().pallet_count
    for i in range(3):
        assert counts[i] == cap, f"pallet {i}: {counts[i]}/{cap} filled"


def test_no_slot_is_double_filled(run_cell):
    plant = _run_until_full(run_cell)
    slots = plant.placed_slots()                        # ground truth
    keys = [(s.pallet, s.layer, s.index) for s in slots]
    assert len(keys) == len(set(keys)), "a slot was filled twice (P8)"


def test_full_pallet_blocks_its_lane(run_cell):
    plant = _run_until_full(run_cell)
    s = plant.sense()
    cap = _capacity_per_pallet()
    for i in range(3):
        assert s.pallet_full[i], f"pallet {i} full but not flagged"
        assert s.pallet_count[i] == cap, f"pallet {i} over-filled past capacity"


def test_no_lane_starves(run_cell):
    plant = _run_until_full(run_cell)
    counts = plant.sense().pallet_count
    assert all(c > 0 for c in counts), f"a lane starved: {counts}"
