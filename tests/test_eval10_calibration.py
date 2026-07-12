"""eval10_calibration (P5, P3): calibration removes bias, cannot beat the noise.

EVAL_PROVENANCE['eval10_calibration'] = (P5, P3, "desk: bias<0.5 mm AND
success<=noise ceiling"). P5: calibration corrects the identifiable systematic
pose error toward zero but cannot touch the stochastic part; residual noise
floors achievable placement reliability. Mirrors delta-hil's test_calibration.py.

SKIPPED WITH REASON until the calibration module is scaffolded: unlike the other
desk evals, `palhil.calibration` / `palhil.evals` / `palhil.scenario` do not yet
exist (they arrive with the phase-3 desk loop, where eval10 turns green). The
importorskip below records that reason rather than erroring at collection.
"""
import pytest

from palhil.constitution import EVAL_PROVENANCE

EVAL = "eval10_calibration"

calibration = pytest.importorskip(
    "palhil.calibration",
    reason="phase 3: palhil.calibration not yet scaffolded (eval10 lands green there)",
)
evals = pytest.importorskip(
    "palhil.evals",
    reason="phase 3: palhil.evals not yet scaffolded (eval10 lands green there)",
)
scenario = pytest.importorskip(
    "palhil.scenario",
    reason="phase 3: palhil.scenario not yet scaffolded (eval10 lands green there)",
)


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


def _scn(seed):
    return scenario.Scenario(
        vision_offset=scenario.default_offset(deg=0.4), pose_sigma_mm=0.15, seed=seed)


def test_bias_driven_below_ik_threshold():
    e = evals.run_eval10(_scn(0))
    assert e.residual_bias_mm < 0.5, e.residual_bias_mm


def test_calibrated_success_reaches_but_not_exceeds_ceiling():
    e = evals.run_eval10(_scn(1))
    assert e.calibrated_success <= e.ceiling + 0.02   # cannot beat variance (P5)
    assert e.calibrated_success >= e.ceiling - 0.05   # but does reach the floor


def test_calibration_actually_helps():
    e = evals.run_eval10(_scn(2))
    assert e.uncalibrated_success < 0.05
    assert e.improves


def test_eval10_holds_across_seeds():
    # P6: statistical, so require it over several seeds, not one trace.
    assert all(evals.run_eval10(_scn(s)).passed for s in range(6))
