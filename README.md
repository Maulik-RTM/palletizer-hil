# palletizer-hil

**Hardware-in-the-loop palletizing.** A **UR20** serves **3 infeed lanes → 3 pallet
stations** (lane *i* → pallet *i*), closed around a **real Beckhoff TwinCAT PLC** —
the delta-hil architecture, re-derived from first principles for a serial-arm
stacking problem.

Method (from [First Principles in Motion](https://oza.bearblog.dev/first-principles-in-motion/)):

1. **Constitution first** — `src/palhil/constitution.py` is signed off before code.
2. **Evals tied to principles** — `EVAL_PROVENANCE` maps every test to a principle.
3. **Mock first, then fill in** — MockCellController + kinematic plant are the gold
   standard; TwinCAT ST is a 1:1 translation of the settled mock.
4. **One clock** — the sim advances only on the PLC's `timeNs` tick.
5. **Two seams** — controller and plant are swappable behind `interfaces.py`;
   the controller is byte-identical across every backend.

## Status

This is a **template**: constitution, seams, tag map, and eval scaffolding are real;
the plant/controller/link bodies are specified build tasks. `plan.md` is the
phase-by-phase handoff for Claude Code.

```bash
pip install -e ".[dev]"
python -m pytest -q        # constitution + FK scaffolding tests pass today
```

## Layout

```
plan.md                      the Claude Code handoff (read first)
src/palhil/
  constitution.py            P1-P8 + refinements A,B + eval provenance (FIXED)
  interfaces.py              Sensors/Commands/Ledger + Controller/Plant protocols
  tags.py                    GVL_Pal tag map, FAST vs SLOW tier
  plant/ur20_kinematics.py   UR20 DH, FK done, analytic IK = phase 2
  plant/pallet_pattern.py    slot/pattern data model = phase 3
  plant/cell_plant.py        kinematic plant spec = phase 3
  plc/cell_controller.py     mock state machine spec = phase 2
  plc/cell_link.py           pyads ADS link spec = phase 5
tests/                       eval scaffolding (grows per phase)
assets/step/                 drop the UR20 STEP files here
docs/twincat_palletizer_program.md   ST program = phase 5
```

## What changed vs delta-hil

| | delta-hil | palletizer-hil |
|---|---|---|
| Robot | ABB IRB 360 (closed-chain delta) | UR20 (serial 6R) |
| P4 | loop closure / guide joints | IK branches, joint limits, singularities, collision floor |
| Task | velocity-matched conveyor tracking | pattern-slot stacking under gravity |
| P3 grasp | pose ∧ velocity coincidence | pick: pose ∧ vacuum window ∧ presence; place: slot ∧ support |
| New | — | P8 conservation+stacking, refinement B payload derate |
