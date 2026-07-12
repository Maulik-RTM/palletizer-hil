"""The pure kinematic plant (default backend). BUILD TASK (plan.md phase 3).

Owns: 3 infeed lanes with box streams, the UR20 (ideal servo integrating the
commanded TCP at the derated speed), vacuum grasp adjudication, 3 pallets with
slot occupancy, and the Ledger.

Rules it enforces (never the controller):
  P3  pick latches iff |tcp - box| < POS_TOL and vacuum_cmd inside the dwell
      window and lane_present; place succeeds iff release within SLOT_TOL of
      the slot AND the slot's support exists (deck or the layer below).
  P4  every commanded pose is clamped-checked against reach + collision
      floor; violation -> reject the command, count it, NEVER teleport.
  P8  ledger.conserved() must hold after every step(); a full pallet sets
      pallet_full and the lane blocks.
  B   speed_scale caps integration rate when part_held.
  P1  step(dt) is called with dt derived from the PLC clock, nowhere else.

Mirror delta-hil's plant/cell_plant.py for shape: sense() / actuate() /
step(dt) / ledger(), pure numpy, no renderer imports.
"""
from __future__ import annotations

from ..interfaces import Commands, Ledger, Plant, Sensors  # noqa: F401

POS_TOL_M = 0.005
SLOT_TOL_M = 0.010
GRIP_DWELL_S = 0.10


class KinematicPalletCell:
    """BUILD TASK -- see plan.md phase 3. Must satisfy interfaces.Plant."""

    def __init__(self, seed: int = 0) -> None:
        raise NotImplementedError("phase 3 -- see plan.md")
