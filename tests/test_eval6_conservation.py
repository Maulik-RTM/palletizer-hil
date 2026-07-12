"""eval6_conservation (P8): the ledger balances every step over a long seeded run.

EVAL_PROVENANCE['eval6_conservation'] = (P8, "desk: ledger balances every step,
all three lanes"). infed == on_lane + on_gripper + on_pallet + rejected, checked
after EVERY step of a long, seeded, three-lane run (the `run_cell` harness asserts
it each tick; here we also demand the run actually did work, so the balance is
non-trivial).

RED until `MockCellController` + `KinematicPalletCell` land (plan.md phase 2/3).
"""
from palhil.constitution import EVAL_PROVENANCE
from palhil.plant.cell_plant import KinematicPalletCell
from palhil.plc.cell_controller import MockCellController

EVAL = "eval6_conservation"


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


def test_ledger_conserved_every_step_long_run(run_cell):
    """eval6: 40 s of closed loop across all three lanes, conserved throughout."""
    plant = KinematicPalletCell(seed=7)                 # NotImplementedError -> RED
    ctrl = MockCellController()
    run_cell(plant, ctrl, steps=20_000)                 # ~40 s at the 2 ms cycle
    led = plant.ledger()
    assert led.conserved()
    assert led.infed > 0, "nothing was infed -- the balance is vacuous"
    assert led.on_pallet > 0, "no box ever reached a pallet -- loop did no work"
