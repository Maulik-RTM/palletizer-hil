"""eval-1 scaffolding: FK sanity on the UR20 DH chain (P4).
The IK round-trip test lands here in phase 2."""
import numpy as np

from palhil.plant.ur20_kinematics import DH, REACH_M, fk


def test_fk_zero_pose_is_finite_and_within_reach():
    T = fk(np.zeros(6))
    p = T[:3, 3]
    assert np.all(np.isfinite(T))
    assert np.linalg.norm(p) <= REACH_M + 0.35  # flange offsets ride on reach


def test_dh_arm_lengths_match_nominal_reach():
    # |a2| + |a3| + d6 should land near the published 1750 mm envelope
    horizontal = abs(DH[1, 1]) + abs(DH[2, 1]) + DH[5, 0]
    assert abs(horizontal - REACH_M) < 0.02


def test_fk_rotation_is_orthonormal():
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 6)
        R = fk(q)[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-9)
