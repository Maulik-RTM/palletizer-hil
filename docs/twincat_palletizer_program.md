# TwinCAT palletizer program (phase 5 deliverable)

Placeholder. After the mock controller is settled (phase 2) and the desk loop
is green (phase 3), this file receives the ST program, in the delta-hil
format (paste each POU into TwinCAT):

- `GVL_Pal` — generated from `src/palhil/tags.py` (the tag map is the contract;
  a test diffs this file against tags.py).
- `FB_PalRobot` — the state machine, a 1:1 translation of
  `plc/cell_controller.py` (IDLE / CLAIM / APPROACH_PICK / PICK / LIFT_CARRY /
  APPROACH_PLACE / PLACE / RETREAT, enable-freeze, pallet-full lane blocking).
- `MAIN` — instantiate FB_PalRobot, publish `timeNs` from the task clock every
  cycle, wire `enable`.

Task cycle: 2 ms, same as delta-hil. The controller commands TCP pose +
speed_scale + vacuum; it never computes IK — joint-space is the plant's world
(that is the honest split for this cell; revisit if the next version targets
joint commands, per the blog's closing note).
