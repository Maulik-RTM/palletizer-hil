# plan.md — UR20 palletizer digital twin (handoff to Claude Code)

Build a HIL palletizing cell: **UR20, 3 infeed lanes → 3 pallet stations**, closed
around a **real TwinCAT PLC**, following the delta-hil method
([blog](https://oza.bearblog.dev/first-principles-in-motion/), sibling repo `../` = delta-hil).

## Ground rules (read before writing any code)

1. **The constitution is fixed.** `src/palhil/constitution.py` (P1–P8, refinements A–B)
   is signed off. Every module docstring cites the principles it serves. If the build
   exposes a principle that is missing or wrong, **STOP and ask the human** — do not
   press on and do not edit the constitution unilaterally.
2. **Evals before implementation.** Each phase below lists its evals first. Write the
   failing test, then the code. Every test cites an `EVAL_PROVENANCE` key.
3. **Mock first, then fill in.** Nothing touches TwinCAT until the mock controller +
   kinematic plant loop is the settled gold standard on a laptop.
4. **One clock.** The sim advances only via `dt = (plc_time_ns - prev) / 1e9` from the
   controller's clock (mock or real). Never `time.time()`, never a fixed dt in the loop.
5. **Seams are sacred.** Everything goes through `interfaces.py` and `tags.py`. Swapping
   controller (mock ↔ ADS) or plant (kinematic ↔ MuJoCo ↔ Isaac) must be a run-script
   flag, and `plc/cell_controller.py` must stay byte-identical across all plants
   (`git diff` empty — it is an eval).
6. **Steal from delta-hil, don't import it.** Port patterns file-for-file
   (`cell_link.py`, `run_web_cell.py`, `build_robot_glb.py`, guard tests), but this repo
   stands alone.
7. Keep `pytest -q` green at the end of every phase. Never delete or weaken an eval to
   make it pass.

## The cell (agreed spec)

- One UR20, floor-mounted, center of the cell. STEP files provided by the human in
  `assets/step/` (per-link).
- 3 infeed lanes (Line 8/9/10) at fixed stop positions (box-present sensor each);
  lane *i* routes to pallet *i*. Boxes arrive stochastically (seeded RNG). Single
  SKU size globally: shipper **310×190×230 mm, ~6.9 kg gross**
  (`assets/Palletizer-Master_Format.xlsx`).
- 3 pallet stations, **1200×1000 mm** deck (data sheet — *not* EUR 1200×800). Column
  stack, **5 layers × 18 boxes = 90/pallet**; a layer is **15 boxes width-wise + 3
  boxes rotated 90° length-wise** in the leftover strip. Full pallet ⇒ lane blocks
  until `pallet_swap_ack`.
- Controller: TwinCAT (mock first). Cycle 2 ms. Commands TCP pose + `speed_scale` +
  vacuum + lane release; observes the tag map in `tags.py` only.
- Vacuum gripper, single box per pick.

## Phase 0 — repo bring-up (½ h)

- `pip install -e ".[dev]"`; `pytest -q` → the constitution + FK tests pass.
- Read, in order: `constitution.py`, `interfaces.py`, `tags.py`, this file. Then skim
  delta-hil's `README.md`, `plant/cell_plant.py`, `plc/cell_controller.py`,
  `plc/cell_link.py`, `scripts/run_web_cell.py`.

## Phase 1 — evals as code

Write the full eval suite as **failing/skipped tests** with docstrings citing
`EVAL_PROVENANCE`: eval1 (IK round-trip < 0.5 mm), eval2 (zero branch flips on any
pick→place path), eval3 (pick latches only on pose ∧ vacuum-window ∧ presence — and
each condition alone must FAIL), eval4 (place: box settles in slot, supported; a
release over a missing support must NOT count as placed), eval6 (ledger conserved
every step, 3 lanes, long seeded run), eval7 (3 pallets fill their patterns, no
double-fill, full pallet blocks its lane, no lane starves), eval8 (loaded carry
respects derate), eval10 (calibration bias<0.5 mm, success ≤ noise ceiling). eval5
(latency) gets its harness now, runs on the rig later.

**Exit:** suite exists, red where implementation is missing; nothing is silently skipped
without a reason string.

## Phase 2 — kinematics + mock controller (desk)

- `ur20_kinematics.ik()`: analytic UR 6R IK (standard shoulder/elbow/wrist branches),
  seed-nearest branch selection, `Unreachable` on out-of-envelope (never clamp),
  wrist-singularity guard band. **Verify the DH table against UR's official UR20
  values first** — flag any discrepancy to the human.
- `plc/cell_controller.py` `MockCellController`: the state machine in its docstring.
  Pure function of `Sensors` → `Commands` plus internal state; owns its own `time_ns`
  (monotonic mock clock). No plant imports (guard test, like delta-hil's
  `test_kinematic_guard.py`).
- **Exit:** eval1, eval2 green; controller unit tests green (claim priority, freeze on
  !enable, pallet-full blocking).

## Phase 3 — kinematic plant + closed desk loop

- `pallet_pattern.column_pattern()`: slots in safe place order (bottom-first,
  far-corner-first — never reach over placed boxes).
- `plant/cell_plant.py` `KinematicPalletCell`: lanes, ideal servo integrating TCP
  commands at derated speed, pick/place adjudication per its docstring, slot occupancy,
  Ledger. Plant never decides control; controller never decides success (P2/P3 split).
- Desk loop runner (`python -m palhil.run`): mock controller ↔ kinematic plant, PLC-clock
  discipline, prints picks/places per pallet + ledger + a self-scored eval-10.
- **Exit:** eval3, eval4, eval6, eval7, eval8, eval10 green. This loop is now the gold
  standard — later backends must reproduce its counts within tolerance.

## Phase 4 — render seam + real CAD

- Port delta-hil `render/web/` (Three.js viewer, ~30 Hz JSON snapshots, no CDN):
  stylized UR + boxes + pallets first.
- STEP pipeline: human drops UR20 STEP into `assets/step/`; port `build_robot_glb.py`
  → per-link glTF; write `plant/ur20_pose.py` (mesh-frame → DH-frame transforms, the
  irb360_pose.py analog) + a JS port for the viewer; `--realbot` flag.
- **Exit:** browser shows the cell palletizing with real CAD, GPU-free; snapshot schema
  documented; controller untouched (invariance eval).

## Phase 5 — TwinCAT (the HIL rung)

- `docs/twincat_palletizer_program.md`: GVL_Pal (generated from `tags.py` — add the
  mapping guard test, cf. delta-hil `test_twincat_mapping.py`), FB_PalRobot (1:1
  translation of the settled mock — same states, same guards, same numbers), MAIN
  (2 ms task, publishes `timeNs`).
- `plc/cell_link.py` `CellAdsLink`: pyads sum-read/sum-write, FAST tier only in the
  cycle, m↔mm at the seam, latency/jitter report every run.
- Run scripts: `run_pal_mock.py <AMS>` (PLC ↔ mock plant, no render — the quickest
  proof the real controller closes the loop), then `run_web_cell.py --plc <AMS>`.
- **Exit (on the human's rig):** eval5 < 10 ms σ < 1 ms; live PLC reproduces the mock
  loop's counts (statistical, P6 — not bit-exact).

## Phase 6 — physics + Isaac (optional rungs, in order of cheapness)

- MuJoCo plant (`mujoco_cell_plant.py`, same contract): real contact — boxes must
  genuinely REST on the stack (P3 support becomes emergent, not adjudicated); friction
  pick-up; cross-backend agreement eval vs kinematic counts.
- Isaac plant + USD scene on the rig (RTX): port `step_to_usd.py`, `isaac_plant.py`,
  `run_twincat_cell.py` pattern. Serial chain ⇒ no guide-joint rigging needed (P4 is
  branch discipline here, not loop closure).

## Ask-the-human list (do not guess)

RESOLVED from `assets/Palletizer-Master_Format.xlsx`: box size/mass (310×190×230 mm,
~6.9 kg gross) · boxes per layer & layer count (18 × 5 = 90) · pallet size
(1200×1000 mm).

STILL OPEN (currently reach-verified ENGINEERING PLACEHOLDERS in `geometry.py`):
lane stop coordinates + cell dimensions · UR20 mounting height · gripper
geometry/length (affects reach + collision floor) · pallet swap mechanics (auto vs
operator ack — assumed operator, no auto-swap) · AMS NetId when phase 5 starts ·
whether picks are from a stopped lane (assumed) or on-the-fly (would add the
delta-hil velocity-match term to P3).

## Definition of done (v1)

Real TwinCAT PLC, running an ST program that is a proven 1:1 mirror of the mock, drives
the UR20 through the browser (and optionally Isaac) to fill 3 pallets from 3 lanes —
conserving every box, holding one IK branch per move, under 10 ms round-trip — and
`pytest -q` is green with every test tracing to a principle.
