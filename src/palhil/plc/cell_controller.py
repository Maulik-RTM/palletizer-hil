"""MockCellController -- the golden reference the TwinCAT FB mirrors 1:1.

BUILD TASK (plan.md phase 2). This is the delta-hil discipline: settle the
behavior HERE, in plain Python, on a laptop, before any ST is written. The
TwinCAT FB_PalRobot is then a line-for-line translation, and controller
invariance (git-diff-empty across all plants) is an eval.

One robot, one state machine (per the blog: small on purpose):

    IDLE -> CLAIM(lane)      pick the highest-priority lane with a box
         -> APPROACH_PICK    hover above the lane stop (branch held, P4)
         -> PICK             descend, vacuum on, dwell (P3 window)
         -> LIFT_CARRY       derated speed (refinement B), via-point above
                             the collision floor of the target pallet (P4)
         -> APPROACH_PLACE   hover above next open slot (from pallet_pattern)
         -> PLACE            descend to slot, release (P3 support check is
                             the PLANT's call -- controller only commands)
         -> RETREAT -> IDLE
    any state + !enable -> FREEZE (hold pose, vacuum unchanged)
    pallet_full[k] -> stop claiming lane k until pallet_swap_ack[k]

Lane->pallet routing: lane i feeds pallet i (3-in / 3-out). No lane starves:
round-robin among lanes whose pallet is not full.

The controller NEVER reads plant internals -- only Sensors (P2), and never
decides whether a pick/place succeeded -- it observes part_held / counts.
"""
from __future__ import annotations

import numpy as np

from .. import geometry as geo
from ..interfaces import Commands, Sensors

CYCLE_NS = 2_000_000            # 2 ms task (plan.md cell spec; the mock oscillator)

# states of the one-robot machine (see the module docstring)
IDLE = "IDLE"
APPROACH_PICK = "APPROACH_PICK"
PICK = "PICK"
LIFT_CARRY = "LIFT_CARRY"
APPROACH_PLACE = "APPROACH_PLACE"
PLACE = "PLACE"
RETREAT = "RETREAT"


def _reached(tcp_pose, target) -> bool:
    """Cartesian sequencing gate: has the TCP arrived at `target` (P2 -- decided
    from the sampled pose alone, never plant internals)."""
    return float(np.linalg.norm(np.array(tcp_pose[:3]) - np.array(target[:3]))) < geo.REACH_TOL_M


class MockCellController:
    """The golden reference controller (plan.md phase 2). Pure function of
    Sensors -> Commands plus internal state; owns its own monotonic clock (P1);
    imports no plant (guard test). Satisfies interfaces.Controller.
    """

    def __init__(self, enabled: bool = True) -> None:
        self.enabled = enabled
        self._t = 0                                   # ns, monotonic mock clock (P1)
        self.state = IDLE
        self.lane: int | None = None
        self._rr = 0                                  # round-robin pointer (no starve)
        self._slot = None
        self._target = geo.home_pose()
        self._vacuum = False

    # -- interfaces.Controller ------------------------------------------------
    def time_ns(self) -> int:
        return self._t

    def step(self, sensors: Sensors) -> Commands:
        self._t += CYCLE_NS                           # the oscillator free-runs (P1)

        if not self.enabled:                          # FREEZE: hold pose + vacuum
            return Commands(tcp_target=self._target, speed_scale=0.0,
                            vacuum_cmd=self._vacuum, lane_release=(False, False, False),
                            enable=False)

        self._advance(sensors)                        # run the state machine

        carrying = sensors.part_held or self.state in (LIFT_CARRY, APPROACH_PLACE, PLACE)
        speed = geo.LOADED_SPEED_SCALE if carrying else 1.0    # refinement B
        release = tuple(not full for full in sensors.pallet_full)  # full pallet blocks lane
        return Commands(tcp_target=self._target, speed_scale=speed,
                        vacuum_cmd=self._vacuum, lane_release=release,
                        pallet_swap_ack=(False, False, False), enable=True)

    # -- state machine --------------------------------------------------------
    def _select_lane(self, s: Sensors) -> int | None:
        """Highest-priority eligible lane, scanned round-robin so none starves."""
        for k in range(3):
            i = (self._rr + k) % 3
            if s.lane_present[i] and not s.pallet_full[i]:
                return i
        return None

    def _advance(self, s: Sensors) -> None:
        tcp = s.tcp_pose

        if self.state == IDLE:
            self._vacuum = False
            self._target = geo.home_pose()
            lane = self._select_lane(s)
            if lane is not None:
                self.lane = lane
                self._rr = (lane + 1) % 3             # fairness for the next claim
                self.state = APPROACH_PICK

        elif self.state == APPROACH_PICK:
            self._vacuum = False
            self._target = geo.lane_approach_pose(self.lane)
            if _reached(tcp, self._target):
                self.state = PICK

        elif self.state == PICK:
            self._target = geo.lane_pick_pose(self.lane)
            self._vacuum = True                       # command vacuum in the dwell window
            if s.part_held:                           # the PLANT decides the latch (P3)
                self.state = LIFT_CARRY

        elif self.state == LIFT_CARRY:
            self._vacuum = True
            self._target = geo.carry_pose_over(geo.LANE_STOP_XYZ[self.lane])
            if _reached(tcp, self._target):
                self.state = APPROACH_PLACE

        elif self.state == APPROACH_PLACE:
            self._vacuum = True
            slot = geo.next_open_slot(self.lane, s.pallet_count[self.lane])
            if slot is None:                          # pallet filled under us -> bail
                self.state = RETREAT
            else:
                self._slot = slot
                self._target = geo.slot_approach_pose(slot)
                if _reached(tcp, self._target):
                    self.state = PLACE

        elif self.state == PLACE:
            self._target = geo.slot_world_pose(self._slot)
            if _reached(tcp, self._target):
                self._vacuum = False                  # release; the PLANT adjudicates (P3/P8)
            if not s.part_held:
                self.state = RETREAT

        elif self.state == RETREAT:
            self._vacuum = False
            self._target = geo.home_pose()
            if _reached(tcp, self._target):
                self.lane = None
                self.state = IDLE
