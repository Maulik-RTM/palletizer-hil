"""The pure kinematic plant (default backend). Phase 3 (plan.md).

Owns: 3 infeed lanes with stochastic box streams, the UR20 as an ideal servo
integrating the commanded TCP at the derated speed, vacuum grasp adjudication,
3 pallets with slot occupancy, and the Ledger. It SENSES and ACTUATES only --
it never decides control (P2/P3 split): the controller says where to go and when
to grip; the PLANT alone decides whether a pick/place actually happened.

Rules it enforces (never the controller):
  P3  pick latches iff |tcp - lane pick pose| < POS_TOL and vacuum is commanded
      through the dwell window and the lane has a box; place succeeds iff release
      happens within SLOT_TOL of an open slot AND that slot is SUPPORTED (deck or
      the slot below). A release anywhere else drops the box (rejected), never a
      phantom placement.
  P4  a commanded pose outside the reach envelope is REJECTED and counted -- the
      servo holds, never teleports/clamps to a fake pose.
  P8  ledger.conserved() holds after every step (infed == on_lane + on_gripper +
      on_pallet + rejected); each slot fills at most once; a full pallet blocks
      its lane (pallet_full set, controller stops releasing/claiming it).
  B   the plant caps carry speed when part_held, regardless of the commanded
      speed_scale -- a loaded UR20 cannot move like an empty one.
  P1  step(dt) advances only on the dt handed in (derived from the PLC clock).

Geometry (lane/slot world poses) comes from geometry.py -- the SAME map the
controller plans against (P2). Pure numpy, no renderer/Isaac imports.
"""
from __future__ import annotations

import numpy as np

from .. import geometry as geo
from ..interfaces import Commands, Ledger, Sensors
from .pallet_pattern import Slot
from .ur20_kinematics import (
    MAX_JOINT_SPEED_EMPTY,
    MAX_JOINT_SPEED_LOADED,
    REACH_M,
)

POS_TOL_M = 0.005
SLOT_TOL_M = 0.010
GRIP_DWELL_S = 0.10

V_TCP_EMPTY_MS = 3.0                                  # ideal-kinematic servo speed
DERATE = MAX_JOINT_SPEED_LOADED / MAX_JOINT_SPEED_EMPTY   # refinement B (=0.5)
REACH_ENVELOPE_M = REACH_M                            # P4 reject radius
ARRIVAL_MEAN_S = 0.20                                # mean lane feed delay (seeded)
_PER_LAYER = geo.CAPACITY_PER_PALLET // geo.LAYERS   # slots per layer (18)


class KinematicPalletCell:
    """Pure kinematic palletizing cell. Satisfies interfaces.Plant."""

    def __init__(self, seed: int = 0) -> None:
        self._rng = np.random.default_rng(seed)
        self.t_ns = 0
        self.tcp = np.array(geo.home_pose(), dtype=float)
        self._cmd = Commands(tcp_target=geo.home_pose())
        self.part_held = False
        self._dwell_s = 0.0
        self.lanes = [{"present": False} for _ in range(3)]
        self.occupied: set[tuple[int, int]] = set()   # (pallet, slot.index)
        self.reach_violations = 0
        # ledger counters (P8)
        self.infed = 0
        self.on_lane = 0
        self.on_gripper = 0
        self.on_pallet = 0
        self.rejected = 0

    # -- interfaces.Plant -----------------------------------------------------
    def sense(self) -> Sensors:
        count = tuple(self._count(p) for p in range(3))
        return Sensors(
            tcp_pose=tuple(self.tcp),
            joints=(0.0,) * 6,                        # kinematic servo tracks TCP directly
            vacuum_on=bool(self._cmd.vacuum_cmd),
            part_held=self.part_held,
            lane_present=tuple(l["present"] for l in self.lanes),
            pallet_count=count,
            pallet_full=tuple(c >= geo.CAPACITY_PER_PALLET for c in count),
            time_ns=self.t_ns,
        )

    def actuate(self, cmd: Commands) -> None:
        self._cmd = cmd

    def step(self, dt: float) -> None:
        self.t_ns += int(round(dt * 1e9))
        cmd = self._cmd
        if not cmd.enable:                            # FREEZE (controller disabled)
            return

        self._feed_lanes(cmd, dt)
        self._servo(cmd, dt)
        if not self.part_held:
            self._adjudicate_pick(cmd, dt)
        elif not cmd.vacuum_cmd:                       # release while holding
            self._adjudicate_release()

    def ledger(self) -> Ledger:
        return Ledger(self.infed, self.on_lane, self.on_gripper,
                      self.on_pallet, self.rejected)

    # -- physics --------------------------------------------------------------
    def _feed_lanes(self, cmd: Commands, dt: float) -> None:
        """Stochastic infeed: a released, empty lane gets a box after ~ARRIVAL_MEAN_S."""
        for i, lane in enumerate(self.lanes):
            if not lane["present"] and cmd.lane_release[i]:
                if self._rng.random() < dt / ARRIVAL_MEAN_S:
                    lane["present"] = True
                    self.infed += 1
                    self.on_lane += 1

    def _servo(self, cmd: Commands, dt: float) -> None:
        target = np.array(cmd.tcp_target, dtype=float)
        if np.linalg.norm(target[:3]) > REACH_ENVELOPE_M:
            self.reach_violations += 1                 # P4: reject, do not move
            return
        v = V_TCP_EMPTY_MS * cmd.speed_scale
        if self.part_held:
            v = min(v, V_TCP_EMPTY_MS * DERATE)        # B: plant enforces the derate
        to = target[:3] - self.tcp[:3]
        d = float(np.linalg.norm(to))
        step = v * dt
        if d <= step or d < 1e-12:
            self.tcp[:3] = target[:3]
        else:
            self.tcp[:3] = self.tcp[:3] + to * (step / d)
        self.tcp[3:] = target[3:]                      # snap orientation (kinematic ideal)

    def _adjudicate_pick(self, cmd: Commands, dt: float) -> None:
        """P3 pick coincidence: pose AND vacuum-window AND presence, jointly."""
        for i, lane in enumerate(self.lanes):
            if not lane["present"]:
                continue
            pick = np.array(geo.lane_pick_pose(i)[:3])
            if cmd.vacuum_cmd and np.linalg.norm(self.tcp[:3] - pick) < POS_TOL_M:
                self._dwell_s += dt
                if self._dwell_s >= GRIP_DWELL_S:      # dwell window satisfied -> latch
                    lane["present"] = False
                    self.on_lane -= 1
                    self.on_gripper += 1
                    self.part_held = True
                    self._dwell_s = 0.0
                return
        self._dwell_s = 0.0                            # any condition missing -> reset

    def _adjudicate_release(self) -> None:
        """P3/P8 place: land in the nearest OPEN slot within tol AND supported,
        else the box drops (rejected) -- never a phantom or floating placement."""
        best, bd = None, 1e9
        for slot in geo.PATTERN:
            if (slot.pallet, slot.index) in self.occupied:
                continue
            sp = np.array(geo.slot_world_pose(slot)[:3])
            d = float(np.linalg.norm(self.tcp[:3] - sp))
            if d < bd:
                bd, best = d, slot
        self.on_gripper -= 1
        self.part_held = False
        if best is not None and bd < SLOT_TOL_M and self._supported(best):
            self.occupied.add((best.pallet, best.index))
            self.on_pallet += 1
        else:
            self.rejected += 1                        # dropped: unsupported or off-slot

    def _supported(self, slot: Slot) -> bool:
        if slot.layer == 0:
            return True                               # rests on the deck
        return (slot.pallet, slot.index - _PER_LAYER) in self.occupied

    def _count(self, pallet: int) -> int:
        return sum(1 for (p, _idx) in self.occupied if p == pallet)

    # -- test / ground-truth hooks (never routed to the controller, P2) -------
    def set_lane_box(self, i: int, present: bool = True) -> None:
        lane = self.lanes[i]
        if present and not lane["present"]:
            lane["present"] = True
            self.infed += 1
            self.on_lane += 1
        elif not present and lane["present"]:
            lane["present"] = False
            self.infed -= 1
            self.on_lane -= 1

    def lane_pick_pose(self, i: int):
        return geo.lane_pick_pose(i)

    def slot_world_pose(self, slot: Slot):
        return geo.slot_world_pose(slot)

    def placed_slots(self) -> list[Slot]:
        return [s for s in geo.PATTERN if (s.pallet, s.index) in self.occupied]
