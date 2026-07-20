# palletizer-hil

**Hardware-in-the-loop palletizing.** A 6-axis arm serves **one infeed source → two
pallet stations**, closed around a **real Beckhoff TwinCAT PLC** — the delta-hil
architecture, re-derived from first principles for a serial-arm stacking problem.
The robot is a **UR20** by default, or a **real-CAD ABB IRB 2600** driven by MuJoCo
IK (`--realbot-abb`); the controller and plant are robot-agnostic, so the robot is
just a render/IK swap.

Method (from [First Principles in Motion](https://oza.bearblog.dev/first-principles-in-motion/)):

1. **Constitution first** — `src/palhil/constitution.py` (P1–P8 + refinements A/B) is
   signed off before code; every module cites the principles it serves.
2. **Evals tied to principles** — `EVAL_PROVENANCE` maps every test to a principle.
3. **Mock first, then fill in** — MockCellController + kinematic plant are the gold
   standard; TwinCAT ST is a 1:1 translation of the settled mock.
4. **One clock** — the sim advances only on the controller's `timeNs` tick.
5. **Two seams** — controller and plant are swappable behind `interfaces.py`;
   the controller is byte-identical across every backend.

## The cell

- **One UR20** (or ABB IRB 2600), floor/pedestal-mounted at the cell centre.
- **One infeed source** feeding the **active pallet**; when it tops out the robot
  switches to the **second** pallet; both full **blocks** the lane until a swap ack.
- **Two pallets**, one on **either side** of the robot (rotated short-side-in for a
  shorter reach), a compact **3×3 layer × 5 = 45 boxes/pallet**.
- Single SKU shipper **310×190×230 mm, ~6.9 kg** (`assets/Palletizer-Master_Format.xlsx`).
- Vacuum gripper, one box per pick. Counts are driven by `geometry.N_LANES` /
  `N_PALLETS`, so the topology is cheap to change.

## Quick start

```bash
pip install -e ".[dev,web,cad,mujoco]"   # dev=pytest, web=viewer, cad=STL/STEP, mujoco=ABB IK

python -m pytest -q                       # 50 passed, 1 skipped (eval5 is rig-only)
python -m palhil.run                      # closed desk loop + self-scored eval-10
```

`python -m palhil.run` closes the MockCellController ↔ kinematic plant loop under
PLC-clock discipline and prints per-pallet fill, the conservation ledger, reach and
collision violations, and a self-scored eval-10.

## Web viewer (GPU-free, no CDN)

```bash
python scripts/run_web_cell.py                # stylized arm  -> http://127.0.0.1:8080
python scripts/run_web_cell.py --realbot      # real UR20 CAD (needs per-link STEP; else stylized)
python scripts/run_web_cell.py --realbot-abb  # ABB IRB 2600 real CAD + MuJoCo IK (no UR20)
python scripts/run_web_cell.py --plc 5.1.204.123.1.1   # live TwinCAT over ADS (phase 5)
```

A Three.js viewer streams ~30 Hz JSON snapshots of the same plant; the browser is a
passive consumer (P2). `--realbot-abb` swaps in the articulated ABB (per-link CAD
placed by MuJoCo forward kinematics) with a vacuum suction pad on the flange.

## Building the robot CAD (glTF)

GLBs are regenerable and gitignored; rebuild them from the committed source meshes:

```bash
python scripts/build_irb2600_glb.py       # ABB: per-link STL (assets/robot_abb/) -> static/robot_abb/*.glb
python scripts/build_ur20_glb.py          # UR20: per-link STEP (assets/step/) -> static/robot/*.glb
```

The ABB IRB 2600-12/1.85 description (STL + xacro) is vendored from
[ros-industrial/abb](https://github.com/ros-industrial/abb) under `assets/robot_abb/`.

## Layout

```
plan.md                        phase-by-phase build handoff
src/palhil/
  constitution.py              P1-P8 + refinements A,B + eval provenance (FIXED)
  interfaces.py                Sensors/Commands/Ledger + Controller/Plant protocols
  geometry.py                  the one shared world map (lane, pallets, poses)
  tags.py                      GVL_Pal tag map, FAST vs SLOW tier
  plant/ur20_kinematics.py     UR20 DH, analytic 6R IK (branch-hold, elbow-up)
  plant/abb_kinematics.py      ABB IRB2600 MuJoCo model + numerical IK (--realbot-abb)
  plant/pallet_pattern.py      3x3 column-stack pattern
  plant/cell_plant.py          kinematic plant: servo, grasp/place adjudication, ledger
  plc/cell_controller.py       mock state machine (the golden reference)
  plc/cell_link.py             pyads ADS link (phase 5)
  render/web/                  Three.js viewer + snapshot server
scripts/
  run_web_cell.py              web viewer runner (stylized / --realbot / --realbot-abb / --plc)
  build_irb2600_glb.py         ABB STL -> glTF
  build_ur20_glb.py            UR20 STEP -> glTF
tests/                         eval suite (each test cites an EVAL_PROVENANCE key)
docs/twincat_palletizer_program.md   ST program (phase 5)
```

## What changed vs delta-hil

| | delta-hil | palletizer-hil |
|---|---|---|
| Robot | ABB IRB 360 (closed-chain delta) | UR20 6R (default) / ABB IRB 2600 6-axis (`--realbot-abb`) |
| P4 | loop closure / guide joints | IK branches, joint limits, singularities, collision floor |
| Task | velocity-matched conveyor tracking | pattern-slot stacking under gravity |
| P3 grasp | pose ∧ velocity coincidence | pick: pose ∧ vacuum ∧ presence; place: slot ∧ support |
| New | — | P8 conservation+stacking, refinement B payload derate, self-collision eval |

## Status

The desk + browser digital twin is **live and green** (`pytest -q` → 50 passed,
1 skipped). Phase 5 (the live TwinCAT/ADS rung, `eval5`) runs on the human's rig.
See `plan.md` for the phase-by-phase detail.
```
