"""ABB IRB2600-12/1.85 kinematics via MuJoCo -- the ABB backend for --realbot-abb.

When the cell runs in ABB mode we do NOT use the UR20 at all: this module owns
the robot. The plant/controller stay TCP-based and robot-agnostic (P2); this
recovers the 6R joints for a commanded TCP by NUMERICAL IK on a MuJoCo model
built from the ROS-Industrial abb_irb2600_support description, and returns each
link's world transform so the viewer can articulate the real per-link CAD.

Kinematics are the abb_irb2600_12_185 macro.xacro joint origins/axes (all zero
rpy): a clean serial 6R. IK targets the tool position and a tool-DOWN approach
(tool0 z-axis -> world -Z), leaving wrist yaw free (1 redundant DOF, damped-least-
squares). Reach ~1.85 m; the compact cell sits well inside it.

Cites P4 (serial-chain kinematics, reach envelope) -- for the ABB instead of the
UR20 when --realbot-abb is selected.
"""
from __future__ import annotations

import numpy as np

try:
    import mujoco
except ImportError:                                  # keep import-clean without the extra
    mujoco = None

REACH_M = 1.85                                        # IRB2600-12/1.85 nominal reach
LINK_NAMES = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]

# joint origins (xyz) and hinge axes from irb2600_12_185_macro.xacro (rpy all 0)
_JOINTS = [
    ("link_1", (0.0,  0.0, 0.445), (0, 0, 1), (-180, 180)),
    ("link_2", (0.15, 0.0, 0.0),   (0, 1, 0), (-95, 155)),
    ("link_3", (0.0,  0.0, 0.900), (0, 1, 0), (-180, 75)),
    ("link_4", (0.0,  0.0, 0.115), (1, 0, 0), (-400, 400)),
    ("link_5", (0.795, 0.0, 0.0),  (0, 1, 0), (-120, 120)),
    ("link_6", (0.085, 0.0, 0.0),  (1, 0, 0), (-400, 400)),
]


def _mjcf() -> str:
    """Nested-body MJCF matching the URDF joint chain; a tool0 site (flange rotated
    +90 deg about y, ROS convention: site z = tool approach axis) is the IK target."""
    head = ('<mujoco model="irb2600"><compiler angle="radian"/>'
            '<worldbody><body name="base_link" pos="0 0 0">')
    body = ""
    for i, (name, xyz, axis, (lo, hi)) in enumerate(_JOINTS, start=1):
        body += (f'<body name="{name}" pos="{xyz[0]} {xyz[1]} {xyz[2]}">'
                 # nominal inertial: MuJoCo requires moving bodies to have mass; we
                 # only use FK/IK (no dynamics), so the value is immaterial
                 f'<inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>'
                 f'<joint name="j{i}" type="hinge" axis="{axis[0]} {axis[1]} {axis[2]}" '
                 f'range="{np.deg2rad(lo):.5f} {np.deg2rad(hi):.5f}"/>')
    # tool0 site at link_6: rpy(0, +90deg, 0) -> quat (w x y z)
    body += ('<site name="tool0" pos="0 0 0" quat="0.70710678 0 0.70710678 0" '
             'size="0.01"/>')
    body += "</body>" * len(_JOINTS)                 # close link_1..6
    tail = "</body></worldbody></mujoco>"
    return head + body + tail


class _Model:
    def __init__(self) -> None:
        if mujoco is None:
            raise ImportError("ABB backend needs the [mujoco] extra: pip install -e '.[mujoco]'")
        self.m = mujoco.MjModel.from_xml_string(_mjcf())
        self.d = mujoco.MjData(self.m)
        self.site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "tool0")
        self.body = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, n) for n in LINK_NAMES]
        self.lo = self.m.jnt_range[:, 0].copy()
        self.hi = self.m.jnt_range[:, 1].copy()

    def fk(self, q):
        self.d.qpos[:6] = q
        mujoco.mj_forward(self.m, self.d)

    def link_transforms(self, q):
        """World (pos, quat[w,x,y,z]) for base_link..link_6 at config q -- the
        viewer places each per-link mesh (authored in its link frame) here."""
        self.fk(q)
        out = []
        for b in self.body:
            pos = self.d.xpos[b].copy()
            quat = self.d.xquat[b].copy()             # MuJoCo: (w, x, y, z)
            out.append((pos.tolist(), quat.tolist()))
        return out

    # full tool-DOWN target orientation (tool0 z -> world -Z, fixed yaw). Fixing the
    # FULL orientation (not just "z down") removes the wrist-yaw redundancy whose
    # null-space drift made the chained seed lose track at base swings -- so IK is
    # square, unique and tracks continuously (no per-frame recovery).
    _R_TARGET = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

    def _solve(self, target, seed, iters=80, lam=0.08):
        """One damped-least-squares run from `seed` to (position, tool-down)."""
        q = np.clip(np.asarray(seed, float), self.lo, self.hi).copy()
        Rt = self._R_TARGET
        jacp = np.zeros((3, self.m.nv))
        jacr = np.zeros((3, self.m.nv))
        perr = np.array([9.0, 9.0, 9.0])
        for _ in range(iters):
            self.fk(q)
            p = self.d.site_xpos[self.site]
            Rc = self.d.site_xmat[self.site].reshape(3, 3)
            perr = target - p
            # small-angle orientation error driving Rc -> Rt
            rerr = 0.5 * (np.cross(Rc[:, 0], Rt[:, 0]) + np.cross(Rc[:, 1], Rt[:, 1])
                          + np.cross(Rc[:, 2], Rt[:, 2]))
            if np.linalg.norm(perr) < 1e-3 and np.linalg.norm(rerr) < 1e-2:
                break
            err = np.concatenate([perr, rerr])
            mujoco.mj_jacSite(self.m, self.d, jacp, jacr, self.site)
            J = np.vstack([jacp[:, :6], jacr[:, :6]])
            dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(6), err)
            q = np.clip(q + dq, self.lo, self.hi)
        return q, float(np.linalg.norm(perr))

    def ik(self, target_pos, q_seed=None):
        """Tool0 to target_pos, tool pointing DOWN, wrist yaw free. The base joint
        is a global DOF a local solver can't discover, so a fresh solve SEEDS
        joint_1 at the target azimuth and multi-restarts from a few postures.

        COST IS BOUNDED so an unreachable pose can never grind the render loop: when
        chaining (q_seed given, the per-frame case) we try the chained seed and at
        most ONE azimuth recovery -- never the full restart every frame. Only the
        initial unseeded solve pays for the full multi-restart (once)."""
        target = np.asarray(target_pos, float)
        azim = float(np.arctan2(target[1], target[0]))

        if q_seed is not None:                        # per-frame tracking: cheap + smooth
            q, e = self._solve(target, np.asarray(q_seed, float))
            # accept a LOOSE (8 mm) chained result: the joints stay continuous
            # (smooth arm) and sub-cm TCP error is invisible on a display robot.
            # This keeps near-singular swing frames on the cheap path instead of the
            # recovery grind. Only a genuine loss-of-track (>8 mm) recovers.
            if e < 8e-3:
                return q
            for j2, j3, j5 in ((0.4, -0.9, -1.0), (0.9, -1.4, -1.2), (0.0, -0.5, -1.0)):
                q2, e2 = self._solve(target, np.array([azim, j2, j3, 0.0, j5, 0.0]))
                if e2 < 2e-3:
                    return q2
            return q if e < 0.05 else None             # best-effort hold, else drop

        best_q, best_e = None, 1e9                     # fresh solve: full multi-restart (once)
        for j1 in (azim, azim + np.pi):
            for j2, j3, j5 in ((0.4, -0.9, -1.0), (0.9, -1.4, -1.2), (0.0, -0.5, -1.0)):
                q, e = self._solve(target, np.array([j1, j2, j3, 0.0, j5, 0.0]))
                if e < 2e-3:
                    return q
                if e < best_e:
                    best_q, best_e = q, e
        return best_q if best_e < 2e-3 else None


_MODEL: _Model | None = None


def _model() -> _Model:
    global _MODEL
    if _MODEL is None:
        _MODEL = _Model()
    return _MODEL


def ik(target_pos, q_seed=None):
    return _model().ik(target_pos, q_seed)


def link_transforms(q):
    return _model().link_transforms(q)
