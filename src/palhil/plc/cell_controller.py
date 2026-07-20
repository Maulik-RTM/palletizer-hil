"""MockCellController -- the golden reference the TwinCAT FB mirrors 1:1.

BUILD TASK (plan.md phase 2). This is the delta-hil discipline: settle the
behavior HERE, in plain Python, on a laptop, before any ST is written. The
TwinCAT FB_PalRobot is then a line-for-line translation, and controller
invariance (git-diff-empty across all plants) is an eval.

SIMPLIFIED CELL (human-directed): ONE infeed source -> TWO pallet stations. The
single robot fills the ACTIVE pallet (lowest-index not-full); when it tops out it
switches to the other; both full BLOCKS the lane until pallet_swap_ack. Counts
come from geometry.N_LANES / N_PALLETS -- no lane-index routing survives here.

One robot, one state machine (per the blog: small on purpose):

    IDLE -> CLAIM            claim the source iff a box is present AND a pallet
                             has room; pick the active pallet
         -> APPROACH_PICK    hover above the lane stop (branch held, P4)
         -> PICK             descend, vacuum on, dwell (P3 window)
         -> LIFT_CARRY       derated (refinement B): rise STRAIGHT UP over the
                             lane to PALLET_CLEAR_Z_M (above any stack, P4)
         -> APPROACH_PLACE   traverse horizontally AT clear height to directly
                             above the next open slot -- the pallet-clearing
                             via-point, so the carried box never sweeps a placed
                             one (P4 collision floor, P8; eval9)
         -> PLACE            descend straight down to the slot, release (P3
                             support check is the PLANT's call -- controller only
                             commands)
         -> RETREAT_UP       rise STRAIGHT UP off the stack to clear height
         -> RETREAT -> IDLE  return home
    any state + !enable -> FREEZE (hold pose, vacuum unchanged)
    pallet_full[k] -> stop placing on k; both full -> stop claiming the source.

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
RETREAT_UP = "RETREAT_UP"
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
        self.pallet: int | None = None                # the active pallet being filled
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
                            vacuum_cmd=self._vacuum,
                            lane_release=(False,) * geo.N_LANES,
                            pallet_swap_ack=(False,) * geo.N_PALLETS,
                            enable=False)

        self._advance(sensors)                        # run the state machine

        carrying = sensors.part_held or self.state in (LIFT_CARRY, APPROACH_PLACE, PLACE)
        speed = geo.LOADED_SPEED_SCALE if carrying else 1.0    # refinement B
        # the source presents a box only while SOME pallet has room (P8): both
        # full -> block the lane until a swap frees a pallet.
        has_room = self._active_pallet(sensors) is not None
        release = (has_room,) * geo.N_LANES
        return Commands(tcp_target=self._target, speed_scale=speed,
                        vacuum_cmd=self._vacuum, lane_release=release,
                        pallet_swap_ack=(False,) * geo.N_PALLETS, enable=True)

    # -- state machine --------------------------------------------------------
    def _active_pallet(self, s: Sensors) -> int | None:
        """Lowest-index pallet with room -- fill one, then switch to the next."""
        for p in range(geo.N_PALLETS):
            if not s.pallet_full[p]:
                return p
        return None

    def _select_lane(self, s: Sensors) -> int | None:
        """Claim the single source iff a box waits AND a pallet has room."""
        if s.lane_present[0] and self._active_pallet(s) is not None:
            return 0
        return None

    def _advance(self, s: Sensors) -> None:
        tcp = s.tcp_pose

        if self.state == IDLE:
            self._vacuum = False
            self._target = geo.home_pose()
            lane = self._select_lane(s)
            if lane is not None:
                self.lane = lane
                self.pallet = self._active_pallet(s)
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
            # straight up over the lane to clear height (the first via-point)
            self._target = geo.carry_pose_over(geo.LANE_STOP_XYZ[self.lane])
            if _reached(tcp, self._target):
                self.state = APPROACH_PLACE

        elif self.state == APPROACH_PLACE:
            self._vacuum = True
            pallet = self._active_pallet(s)
            if pallet is None:                        # both pallets filled under us
                self.state = RETREAT
                return
            self.pallet = pallet
            slot = geo.next_open_slot(pallet, s.pallet_count[pallet])
            if slot is None:                          # pattern full -> bail
                self.state = RETREAT
            else:
                self._slot = slot
                # traverse AT clear height to directly above the slot (via-point
                # that clears the pallet stack, P4/P8)
                self._target = geo.slot_clear_pose(slot)
                if _reached(tcp, self._target):
                    self.state = PLACE

        elif self.state == PLACE:
            self._target = geo.slot_world_pose(self._slot)   # descend straight down
            if _reached(tcp, self._target):
                self._vacuum = False                  # release; the PLANT adjudicates (P3/P8)
            if not s.part_held:
                self.state = RETREAT_UP

        elif self.state == RETREAT_UP:
            self._vacuum = False
            # rise straight up off the stack before crossing the cell (P4)
            self._target = geo.slot_clear_pose(self._slot)
            if _reached(tcp, self._target):
                self.state = RETREAT

        elif self.state == RETREAT:
            self._vacuum = False
            self._target = geo.home_pose()
            if _reached(tcp, self._target):
                self.lane = None
                self.pallet = None
                self.state = IDLE
