"""MockCellController unit tests (plan.md phase 2 exit): claim priority, freeze
on !enable, pallet-full blocking. The controller is a pure Sensors->Commands
function plus internal state; these drive it with crafted samples (no plant).

Cites: P2 (acts only on the sampled tag map), P1 (owns the one clock),
P8 (a full pallet blocks its lane), refinement B (loaded derate)."""
from palhil import geometry as geo
from palhil.interfaces import Sensors
from palhil.plc.cell_controller import (
    APPROACH_PICK,
    IDLE,
    MockCellController,
)


def _sensors(**kw):
    d = dict(tcp_pose=geo.home_pose(), joints=(0.0,) * 6, vacuum_on=False,
             part_held=False, lane_present=(False, False, False),
             pallet_count=(0, 0, 0), pallet_full=(False, False, False), time_ns=0)
    d.update(kw)
    return Sensors(**d)


def test_clock_is_monotonic_and_owned():
    c = MockCellController()
    assert c.time_ns() == 0
    c.step(_sensors())
    c.step(_sensors())
    assert c.time_ns() == 2 * 2_000_000            # advanced by two 2 ms cycles (P1)


def test_idle_with_no_boxes_stays_home():
    c = MockCellController()
    cmd = c.step(_sensors(lane_present=(False, False, False)))
    assert c.state == IDLE and c.lane is None
    assert cmd.tcp_target == geo.home_pose() and cmd.vacuum_cmd is False


def test_claim_picks_a_present_lane():
    c = MockCellController()
    c.step(_sensors(lane_present=(False, True, True)))   # lanes 1,2 have boxes
    assert c.lane == 1 and c.state == APPROACH_PICK      # lowest eligible index first


def test_claim_is_round_robin_so_no_lane_starves():
    c = MockCellController()
    # after a claim the pointer advances, so the next scan starts past it
    c._rr = 1
    assert c._select_lane(_sensors(lane_present=(True, True, True))) == 1
    c._rr = 2
    assert c._select_lane(_sensors(lane_present=(True, True, True))) == 2
    c._rr = 2                                             # wraps around, still fair
    assert c._select_lane(_sensors(lane_present=(True, True, False))) == 0


def test_full_pallet_blocks_its_lane():
    c = MockCellController()
    s = _sensors(lane_present=(True, True, True), pallet_full=(True, False, False))
    cmd = c.step(s)
    assert c.lane == 1, "must not claim the lane feeding a full pallet"
    assert cmd.lane_release[0] is False                  # blocked (P8)
    assert cmd.lane_release[1] is True and cmd.lane_release[2] is True


def test_no_claim_when_all_pallets_full():
    c = MockCellController()
    cmd = c.step(_sensors(lane_present=(True, True, True),
                          pallet_full=(True, True, True)))
    assert c.state == IDLE and c.lane is None
    assert cmd.lane_release == (False, False, False)


def test_freeze_holds_pose_and_vacuum_when_disabled():
    c = MockCellController()
    c.step(_sensors(lane_present=(True, False, False)))  # claim -> vacuum plan begins
    held_target, held_vac = c._target, c._vacuum
    c.enabled = False
    cmd = c.step(_sensors(lane_present=(True, False, False)))
    assert cmd.enable is False
    assert cmd.tcp_target == held_target                 # pose held
    assert cmd.vacuum_cmd == held_vac                    # vacuum unchanged
    assert cmd.lane_release == (False, False, False)
    assert cmd.speed_scale == 0.0
    assert c.time_ns() > 0                                # the clock still free-runs (P1)


def test_loaded_carry_is_derated():
    c = MockCellController()
    empty = c.step(_sensors(part_held=False))
    assert empty.speed_scale == 1.0
    loaded = c.step(_sensors(part_held=True))
    assert loaded.speed_scale == geo.LOADED_SPEED_SCALE  # refinement B
