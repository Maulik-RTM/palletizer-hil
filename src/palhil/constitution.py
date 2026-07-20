"""The session constitution, as data. Every module cites these by number.

Adapted from delta-hil for a UR20 palletizing cell: 3 infeed lanes -> 1 UR20
-> 3 pallet stations. Fixed for the project by explicit sign-off. If the build
exposes a principle we missed or got wrong, STOP and re-confirm rather than
pressing on.

What changed vs delta-hil and why:
  - P4 rewritten: the Delta's closed-chain problem is gone; the UR20 is a
    serial 6R chain whose hard truths are IK branch multiplicity, joint
    limits, and wrist singularities.
  - P8 added: palletizing is a stacking + accounting problem. Gravity and
    support matter (the delta cell dropped tortillas into totes; a palletizer
    BUILDS a structure). Conservation is promoted to a principle.
  - P3 split into pick-coincidence and place-support: a place is only real
    if the box ends up resting in its pattern slot.
"""

PRINCIPLES = {
    "P1": "Real-time closed loop: the PLC free-runs on its own oscillator "
          "(un-pausable); any latency the sim adds to the loop is physically "
          "indistinguishable from real lag.",
    "P2": "I/O contract is the only channel: the PLC acts solely on its sampled "
          "tag map (lane sensors, TCP pose, vacuum state, pallet slot counts); "
          "unsensed plant state changes outcomes but never control.",
    "P3": "Pick and place are physical coincidences: a PICK succeeds iff TCP "
          "pose < tol AND vacuum commanded within the dwell window AND the box "
          "is present at the lane stop; a PLACE succeeds iff release happens "
          "with the box within slot tolerance AND the box is SUPPORTED (pallet "
          "deck or the layer below) -- jointly, adjudicated by the plant, "
          "never assumed by the controller.",
    "P4": "Serial-chain kinematics: the UR20 6R IK has up to 8 branches; a "
          "motion holds ONE configuration branch end-to-end (no elbow/wrist "
          "flips mid-carry), respects joint limits, and stays off wrist "
          "singularities; every commanded pose is inside the reach envelope "
          "and above the collision floor (boxes already on the pallet).",
    "P5": "Calibration corrects bias, not variance: it drives the identifiable "
          "systematic pose error toward zero but cannot touch the stochastic "
          "part; residual noise floors achievable placement reliability.",
    "P6": "Reproducibility is bounded by the live loop: no bit-exact replay with "
          "a free-running PLC; evals must be statistical, not single-trace.",
    "P7": "HIL value is conditional: worth it only if the program under test is "
          "the real deployed one and the sim reproduces faults rarer/costlier/"
          "more dangerous to trigger physically; $0 core.",
    "P8": "Conservation and stacking: every box is counted every step -- "
          "infed == on_lane + on_gripper + on_pallet + rejected; a placed box "
          "occupies exactly one pattern slot (never two, never none); bodies "
          "never interpenetrate; a full pallet BLOCKS its lane until swapped.",
}

REFINEMENTS = {
    "A": "Two-tier I/O: FAST tier (EtherCAT/ADS) for axis+vacuum+lane-sensor "
         "I/O under the eval-5 jitter bound; SLOW tier (OPC UA) for "
         "supervisory tags (pallet counts, pattern selection), exempt from it.",
    "B": "Payload honesty: joint speed/accel limits scale with carried mass; "
         "a UR20 carrying near 20 kg does not move like an empty one. The "
         "plant enforces the derate; the controller plans within it.",
}

# Which evals each module is accountable to.
EVAL_PROVENANCE = {
    "eval1_ik_error":        ("P4",       "desk: FK(IK(pose)) < 0.5 mm on the UR20 chain"),
    "eval2_branch_hold":     ("P4",       "desk: zero configuration flips along any pick->place path"),
    "eval3_pick_coincidence":("P3", "P2", "desk: pick latches only on pose AND vacuum-window AND presence"),
    "eval4_place_supported": ("P3", "P8", "desk: box settles in its slot, supported, z-err < tol"),
    "eval5_latency":         ("P1", "A",  "rig: <10 ms round-trip, sigma<1 ms jitter, FAST tier"),
    "eval6_conservation":    ("P8",       "desk: ledger balances every step, single source"),
    "eval7_pattern_fill":    ("P8", "P2", "desk: 2 pallets fill from one source, no double-fill, full pallet blocks the lane"),
    "eval8_payload_derate":  ("B",  "P4", "desk: loaded-vs-empty carry respects the derated limits"),
    "eval9_self_collision":  ("P4", "P8", "desk: carried box clears the stack via the pallet via-point; no box-box interpenetration on any carry path"),
    "eval10_calibration":    ("P5", "P3", "desk: bias<0.5 mm AND success<=noise ceiling"),
}
