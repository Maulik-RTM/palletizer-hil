"""Fault injection, split into the two categories P5 cares about.

  systematic  -- ``vision_offset`` (a fixed frame bias): identifiable, therefore
                 REMOVABLE by calibration.
  stochastic  -- ``pose_sigma_mm`` Gaussian jitter (+ optional discrete faults):
                 unidentifiable, therefore NOT removable; sets the reliability
                 floor no matter how good calibration is.

Seeded, but per P6 the closed loop's timing jitter makes results statistically --
not bit-exactly -- repeatable. Ported from delta-hil.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .calibration import FrameTransform


@dataclass
class Scenario:
    vision_offset: FrameTransform  # SYSTEMATIC: removable by calibration
    pose_sigma_mm: float = 0.15    # STOCHASTIC: irreducible noise floor
    jam_rate: float = 0.0          # STOCHASTIC: no box arrives this cycle
    misfeed_rate: float = 0.0      # STOCHASTIC: box grossly mislocated
    misfeed_mm: float = 8.0
    workspace_mm: float = 150.0
    seed: int = 0

    def __post_init__(self):
        self._rng = np.random.default_rng(self.seed)

    def sample_part(self):
        """Return (true_xyz, reported_xyz, jammed) for one cycle: where the box
        actually is, what vision reports (true through the offset + noise), and
        whether no box is present."""
        true_xyz = self._rng.uniform(-self.workspace_mm, self.workspace_mm, 3)
        if self._rng.random() < self.jam_rate:
            return true_xyz, None, True
        reported = self.vision_offset.apply(true_xyz)
        reported = reported + self._rng.normal(0, self.pose_sigma_mm, 3)
        if self._rng.random() < self.misfeed_rate:
            reported = reported + self._rng.normal(0, self.misfeed_mm, 3)
        return true_xyz, reported, False

    def fixture_points(self, n: int):
        """Known calibration-fixture correspondences (true, reported) -- the input
        to Kabsch. Noise still applies (calibration sees through bias, not noise)."""
        true_pts, rep_pts = [], []
        for _ in range(n):
            p = self._rng.uniform(-self.workspace_mm, self.workspace_mm, 3)
            q = self.vision_offset.apply(p) + self._rng.normal(0, self.pose_sigma_mm, 3)
            true_pts.append(p)
            rep_pts.append(q)
        return np.array(true_pts), np.array(rep_pts)


def default_offset(deg: float = 0.4, t=(3.0, -2.0, 1.5)) -> FrameTransform:
    th = np.radians(deg)
    R = np.array([[np.cos(th), -np.sin(th), 0.0],
                  [np.sin(th), np.cos(th), 0.0],
                  [0.0, 0.0, 1.0]])
    return FrameTransform(R, np.array(t, float))
