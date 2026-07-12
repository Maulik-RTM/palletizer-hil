"""The constitution is data; keep it well-formed. Every eval must cite
existing principles/refinements, and the P8 ledger identity must be the
one the plant enforces."""
from palhil.constitution import EVAL_PROVENANCE, PRINCIPLES, REFINEMENTS
from palhil.interfaces import Ledger


def test_eval_provenance_cites_real_principles():
    valid = set(PRINCIPLES) | set(REFINEMENTS)
    for eval_name, entry in EVAL_PROVENANCE.items():
        cited = [c for c in entry if c in valid or len(c) <= 2]
        assert cited, f"{eval_name} cites nothing"
        for c in entry[:-1]:  # last element is the description
            assert c in valid, f"{eval_name} cites unknown principle {c!r}"


def test_ledger_conservation_identity():
    led = Ledger(infed=10, on_lane=3, on_gripper=1, on_pallet=5, rejected=1)
    assert led.conserved()
    led.on_pallet += 1  # a box materialized -- P8 violation
    assert not led.conserved()
