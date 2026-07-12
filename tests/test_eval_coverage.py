"""Meta-eval: every principle-accountable eval has a test module (P7 discipline).

The constitution lists which evals each principle is accountable to; this asserts
the suite actually covers them all, so an eval can never be silently dropped.
Green today -- it is the map, not the territory.
"""
from pathlib import Path

from palhil.constitution import EVAL_PROVENANCE

TESTS = Path(__file__).parent


def test_every_eval_has_a_test_module():
    files = {p.name for p in TESTS.rglob("test_*.py")}
    missing = []
    for key in EVAL_PROVENANCE:                    # e.g. "eval3_pick_coincidence"
        short = key.split("_", 1)[0]               # -> "eval3"
        if not any(f.startswith(f"test_{short}_") for f in files):
            missing.append(key)
    assert not missing, f"evals with no test module: {missing}"
