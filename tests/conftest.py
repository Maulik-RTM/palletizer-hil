"""Shared desk-loop harness for the eval suite (P1, P8).

`run_cell` closes the mock controller <-> plant loop the way the real rig will:
the sim advances ONLY on `dt = (plc_time_ns - prev) / 1e9`, taken from the
controller's own clock -- never `time.time()`, never a fixed dt (plan.md rule 4,
constitution P1). Conservation (P8) is asserted after every step, so a run that
ever breaks the ledger fails loudly instead of drifting.

Cites: P1 (one clock), P8 (ledger conserved every step).
"""
import pytest

# 2 ms nominal PLC cycle (plan.md "cell" spec: TwinCAT cycle 2 ms).
CYCLE_S = 0.002

# --- cell geometry from assets/Palletizer-Master_Format.xlsx -------------------
# Real product data (rows 1-12): shipper 310x190x230 mm, gross ~6.9 kg, single
# SKU size globally. 18 shippers/layer x 5 layers = 90/pallet on a 1200x1000
# pallet. Supersedes plan.md's simplified 1200x800 / 4x3 figures.
BOX_LWH_M = (0.310, 0.190, 0.230)  # shipper L x W x H
BOX_MASS_KG = 6.9                  # gross weight (payload; refinement B derate)
PALLET_LW_M = (1.200, 1.000)       # pallet footprint (data sheet, NOT EUR 1200x800)
LAYERS = 5                         # 18/layer x 5 = 90/pallet (data sheet)

from palhil.plant.pallet_pattern import column_pattern  # noqa: E402

CAPACITY = sum(1 for s in column_pattern(BOX_LWH_M, PALLET_LW_M, LAYERS)
               if s.pallet == 0)   # 90 slots/pallet


@pytest.fixture
def run_cell():
    """Return a closed-loop runner honoring the one-clock discipline (P1)."""
    def _run(plant, controller, steps, assert_conserved=True):
        prev = controller.time_ns()
        for _ in range(steps):
            sensors = plant.sense()
            cmd = controller.step(sensors)      # ticks the controller's own clock
            now = controller.time_ns()
            dt = (now - prev) / 1e9             # P1: dt derives from the controller clock
            prev = now
            plant.actuate(cmd)
            if dt > 0:
                plant.step(dt)                  # advance ONLY on a real tick
            if assert_conserved:
                assert plant.ledger().conserved(), "P8: ledger broke mid-run"
        return plant
    return _run
