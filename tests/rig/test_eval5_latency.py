"""eval5_latency (P1, A): round-trip < 10 ms, jitter sigma < 1 ms, FAST tier.

EVAL_PROVENANCE['eval5_latency'] = (P1, A, "rig: <10 ms round-trip, sigma<1 ms
jitter, FAST tier"). This one gets its HARNESS now and RUNS ON THE RIG later
(plan.md phase 1/5): the sim adds latency to a free-running PLC's loop, and any
latency is physically indistinguishable from real lag (P1), so it must stay under
the FAST-tier bound (refinement A).

The pure statistics harness (`latency_stats`) is verified on the desk today
(green). The live measurement against `CellAdsLink` is rig-only: skipped unless
PALHIL_RIG=1, with a reason -- never silently.
"""
import os
import statistics

import pytest

from palhil.constitution import EVAL_PROVENANCE

EVAL = "eval5_latency"
ROUNDTRIP_MAX_MS = 10.0
JITTER_MAX_MS = 1.0


def latency_stats(samples_s):
    """(mean_ms, sigma_ms) for a sequence of per-cycle round-trip times in seconds.
    The harness the rig run scores itself against; pure, so it is testable here."""
    ms = [s * 1e3 for s in samples_s]
    mean = statistics.fmean(ms)
    sigma = statistics.pstdev(ms) if len(ms) > 1 else 0.0
    return mean, sigma


def test_cites_constitution():
    assert EVAL in EVAL_PROVENANCE, f"{EVAL} not registered in the constitution"


def test_latency_stats_math():
    """Green now: the scoring harness itself is correct before the rig uses it."""
    mean, sigma = latency_stats([0.005, 0.005, 0.005])
    assert mean == pytest.approx(5.0) and sigma == pytest.approx(0.0)
    mean, sigma = latency_stats([0.004, 0.006])
    assert mean == pytest.approx(5.0) and sigma == pytest.approx(1.0)


@pytest.mark.rig
@pytest.mark.skipif(
    os.environ.get("PALHIL_RIG") != "1",
    reason="eval5 is rig-only: needs a live TwinCAT PLC over ADS (set PALHIL_RIG=1)",
)
def test_roundtrip_under_bound_on_rig():
    """eval5 (rig): sum-read/sum-write round-trip stays under the FAST-tier bound."""
    from palhil.plc.cell_link import CellAdsLink

    ams = os.environ["PALHIL_AMS"]                      # e.g. "5.1.2.3.1.1"
    link = CellAdsLink(ams)
    samples = link.measure_roundtrip(n=500)             # phase-5 CellAdsLink API
    mean, sigma = latency_stats(samples)
    print(f"eval5: mean={mean:.3f} ms  sigma={sigma:.3f} ms  n={len(samples)}")
    assert mean < ROUNDTRIP_MAX_MS, f"eval5 FAIL: mean {mean:.3f} ms >= 10 ms"
    assert sigma < JITTER_MAX_MS, f"eval5 FAIL: jitter {sigma:.3f} ms >= 1 ms"
