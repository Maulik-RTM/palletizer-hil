"""GVL_Pal tag map -- the single source of truth for the ADS seam (P2, A).

The TwinCAT GVL and CellAdsLink are both generated/checked against this dict,
exactly like delta-hil's tags.py. Units at the seam: mm and LREAL on the PLC
side, m and float on the Python side; the link converts, nobody else does.

FAST tier (eval-5 jitter bound applies) vs SLOW tier (supervisory, exempt).
"""

FAST = {
    # sensors (Python plant -> PLC)
    "GVL_Pal.tcpAct":       "ARRAY[0..5] OF LREAL",  # mm / deg
    "GVL_Pal.jointsAct":    "ARRAY[0..5] OF LREAL",  # deg
    "GVL_Pal.vacuumOn":     "BOOL",
    "GVL_Pal.partHeld":     "BOOL",
    "GVL_Pal.lanePresent":  "ARRAY[0..2] OF BOOL",
    # commands (PLC -> Python plant)
    "GVL_Pal.tcpCmd":       "ARRAY[0..5] OF LREAL",
    "GVL_Pal.speedScale":   "LREAL",
    "GVL_Pal.vacuumCmd":    "BOOL",
    "GVL_Pal.laneRelease":  "ARRAY[0..2] OF BOOL",
    # the clock (P1) -- the sim derives dt from THIS, never its own clock
    "GVL_Pal.timeNs":       "LINT",
    "GVL_Pal.enable":       "BOOL",
}

SLOW = {
    "GVL_Pal.palletCount":  "ARRAY[0..2] OF INT",
    "GVL_Pal.palletFull":   "ARRAY[0..2] OF BOOL",
    "GVL_Pal.palletSwapAck":"ARRAY[0..2] OF BOOL",
    "GVL_Pal.patternId":    "ARRAY[0..2] OF INT",
}
