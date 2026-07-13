"""Closed desk loop + self-score. Run: ``python -m palhil.run``.

Closes the loop over the MockCellController and the KinematicPalletCell under the
PLC-clock discipline (P1: dt comes from the controller's clock, never wall time),
fills the three pallets, prints picks/places per pallet + the conservation ledger
(P8), and self-scores eval10 (calibration, P5/P3). This loop is the gold standard
the TwinCAT / MuJoCo / Isaac backends must reproduce (P6, statistical).
"""
from __future__ import annotations

from .evals import run_eval10
from .geometry import CAPACITY_PER_PALLET
from .plant.cell_plant import KinematicPalletCell
from .plc.cell_controller import MockCellController
from .scenario import Scenario, default_offset


def _bar(x, width=28):
    n = int(round(x * width))
    return "#" * n + "-" * (width - n)


def run_loop(seed: int = 3, max_steps: int = 800_000):
    """Fill all three pallets; return (plant, steps, conserved_every_step)."""
    plant = KinematicPalletCell(seed=seed)
    ctrl = MockCellController()
    prev = ctrl.time_ns()
    conserved = True
    steps = 0
    for i in range(max_steps):
        sensors = plant.sense()
        cmd = ctrl.step(sensors)
        now = ctrl.time_ns()
        dt = (now - prev) / 1e9
        prev = now
        plant.actuate(cmd)
        if dt > 0:
            plant.step(dt)
        conserved &= plant.ledger().conserved()
        steps = i + 1
        if i % 500 == 0 and all(plant.sense().pallet_full):
            break
    return plant, steps, conserved


def main():
    print("=" * 66)
    print("palletizer HIL -- closed desk loop, mock controller <-> kinematic plant")
    print("=" * 66)

    plant, steps, conserved = run_loop()
    s = plant.sense()
    led = plant.ledger()
    sim_s = plant.t_ns / 1e9

    print(f"\nClosed loop (P1/P2) -- controller sees only Sensors, plant only Commands")
    print(f"  ran {steps} cycles  (~{sim_s:.1f} s sim @ 2 ms)")
    for i in range(3):
        c = s.pallet_count[i]
        print(f"  pallet {i}: {c:3d}/{CAPACITY_PER_PALLET}  "
              f"|{_bar(c / CAPACITY_PER_PALLET)}|  full={s.pallet_full[i]}")
    print(f"\nLedger (P8)  infed={led.infed}  on_lane={led.on_lane}  "
          f"on_gripper={led.on_gripper}  on_pallet={led.on_pallet}  rejected={led.rejected}")
    print(f"  conserved every step: {'YES' if conserved else 'NO -- P8 VIOLATION'}  "
          f"| reach violations (P4): {plant.reach_violations}")

    e10 = run_eval10(Scenario(vision_offset=default_offset(deg=0.4),
                              pose_sigma_mm=0.15, seed=3))
    print("\n" + "-" * 66)
    print("SELF-SCORE  eval10  calibration  [P5, P3]")
    print("-" * 66)
    print(f"  (a) residual bias     {e10.residual_bias_mm:.4f} mm  < {e10.tol_mm}  "
          f"-> {'PASS' if e10.bias_ok else 'FAIL'}")
    print(f"  (b) calibrated succ.  {e10.calibrated_success:.3f}  vs ceiling "
          f"{e10.ceiling:.3f}  (not above) -> {'PASS' if e10.ceiling_ok else 'FAIL'}")
    print(f"      uncalibrated      {e10.uncalibrated_success:.3f}  "
          f"-> improves: {'PASS' if e10.improves else 'FAIL'}")
    print(f"  EVAL 10: {'PASS' if e10.passed else 'FAIL'}")
    print("\nRig-verifiable only (cannot self-score at the desk):")
    print("  eval5  <10 ms round-trip, sigma<1 ms jitter, FAST tier  [P1, A]")
    print("=" * 66)


if __name__ == "__main__":
    main()
