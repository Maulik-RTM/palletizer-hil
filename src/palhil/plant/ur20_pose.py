"""UR20 per-link articulation for the render seam -- the serial-chain analog of
delta-hil's irb360_pose.py.

The Delta needed a parallelogram frame-mapping; the UR20 is a serial 6R chain, so
articulation is just the DH forward chain (P4). Given joint angles q, each link i
sits at the base->frame_i transform. A per-link glTF exported in the robot's
HOME (q=0) assembly frame is placed at its posed location by the incremental
transform

    M_i(q) = frame_i(q) @ frame_i(0)^-1

so applying M_i to the home-pose mesh of link i articulates it. This is the
"mesh-frame -> DH-frame" mapping the viewer's JS port (viewer.html) mirrors.

Pure numpy, import-clean off any renderer. Cites P4 (serial-chain kinematics).
"""
from __future__ import annotations

import numpy as np

from .ur20_kinematics import DH, _dh_T, fk

LINK_NAMES = ["base", "shoulder", "upper_arm", "forearm", "wrist1", "wrist2", "flange"]


def link_frames(q: np.ndarray) -> list[np.ndarray]:
    """base->frame_i (4x4) for i=0..6. frame_0 is the base (identity); frame_i is
    the frame after joint i. frame_6 == fk(q) (base->flange)."""
    q = np.asarray(q, float)
    T = np.eye(4)
    frames = [T.copy()]
    for i in range(6):
        d, a, alpha = DH[i]
        T = T @ _dh_T(q[i], d, a, alpha)
        frames.append(T.copy())
    return frames


_HOME = link_frames(np.zeros(6))
_HOME_INV = [np.linalg.inv(f) for f in _HOME]


def link_transforms(q: np.ndarray) -> list[np.ndarray]:
    """Per-link mesh transforms M_i = frame_i(q) @ frame_i(0)^-1 (i=0..6). Applied
    to a link mesh exported in the q=0 assembly frame, these articulate the robot.
    M_0 (base) is identity; link_transforms(0) is all identities."""
    fq = link_frames(q)
    return [fq[i] @ _HOME_INV[i] for i in range(7)]


def flange_frame(q: np.ndarray) -> np.ndarray:
    """base->flange, i.e. fk(q) -- exposed for the viewer to sanity-check the TCP."""
    return fk(q)
