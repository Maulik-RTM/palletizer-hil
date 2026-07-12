"""CellAdsLink -- the live TwinCAT seam. BUILD TASK (plan.md phase 5).

Port delta-hil's plc/cell_link.py: one pyads sum-WRITE of all FAST sensor
tags, one sum-READ of all FAST command tags + GVL_Pal.timeNs, per cycle.
m <-> mm conversion happens HERE and only here (tags.py is the contract).

The loop (P1 -- the PLC clock drives the sim, verbatim from the blog):

    dt = (plc_time_ns - prev_plc_time_ns) / 1e9
    if dt > 0:
        plant.step(dt)   # advance ONLY when the PLC clock actually ticked

Reports round-trip latency mean/sigma every run (eval-5 home).
SLOW-tier tags go over a second, lower-rate read -- never in the FAST cycle.
"""
from __future__ import annotations


class CellAdsLink:
    """BUILD TASK -- see plan.md phase 5. Satisfies interfaces.Controller,
    so run scripts swap MockCellController <-> CellAdsLink with one flag."""

    def __init__(self, ams_net_id: str, port: int = 851) -> None:
        raise NotImplementedError("phase 5 -- see plan.md")
