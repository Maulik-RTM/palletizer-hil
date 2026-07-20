"""MockCellController unit tests (plan.md phase 2 exit): claim, active-pallet
fill-then-switch, freeze on !enable, both-full blocking. The controller is a pure
Sensors->Commands function plus internal state; these drive it with crafted
samples (no plant).

SIMPLIFIED CELL: one source, two pallets (geometry.N_LANES / N_PALLETS).

Cites: P2 (acts only on the sampled tag map), P1 (owns the one clock),
P8 (both pallets full blocks the source), refinement B (loaded derate)."""
from palhil import geometry as geo
from palhil.interfaces import Sensors
from palhil.plc.cell_controller import (
    APPROACH_PICK,
    IDLE,
    MockCellController,
)


def _sensors(**kw):
    d = dict(tcp_pose=geo.home_pose(), joints=(0.0,) * 6, vacuum_on=False,
             part_held=False, lane_present=(False,) * geo.N_LANES,
             pallet_count=(0,) * geo.N_PALLETS,
             pallet_full=(False,) * geo.N_PALLETS, time_ns=0)
    d.update(kw)
    return Sensors(**d)


def test_clock_is_monotonic_and_owned():
    c = MockCellController()
    assert c.time_ns() == 0
    c.step(_sensors())
    c.step(_sensors())
    assert c.time_ns() == 2 * 2_000_000            # advanced by two 2 ms cycles (P1)


def test_idle_with_no_box_stays_home():
    c = MockCellController()
    cmd = c.step(_sensors(lane_present=(False,)))
    assert c.state == IDLE and c.lane is None
    assert cmd.tcp_target == geo.home_pose() and cmd.vacuum_cmd is False


def test_claim_when_box_present_and_a_pallet_has_room():
    c = MockCellController()
    c.step(_sensors(lane_present=(True,)))
    assert c.lane == 0 and c.state == APPROACH_PICK
    assert c.pallet == 0                                 # first pallet is the active one


def test_active_pallet_switches_when_the_first_is_full():
    """The single source feeds the next non-full pallet (fill one, then the other)."""
    c = MockCellController()
    c.step(_sensors(lane_present=(True,), pallet_full=(True, False)))
    assert c.pallet == 1, "must target the pallet that still has room"


def test_both_pallets_full_blocks_the_source():
    c = MockCellController()
    cmd = c.step(_sensors(lane_present=(True,), pallet_full=(True, True)))
    assert c.state == IDLE and c.lane is None
    assert cmd.lane_release == (False,)                  # nowhere to place -> block (P8)


def test_source_released_while_any_pallet_has_room():
    c = MockCellController()
    cmd = c.step(_sensors(lane_present=(False,), pallet_full=(True, False)))
    assert cmd.lane_release == (True,)                   # pallet 1 still open


def test_freeze_holds_pose_and_vacuum_when_disabled():
    c = MockCellController()
    c.step(_sensors(lane_present=(True,)))               # claim -> vacuum plan begins
    held_target, held_vac = c._target, c._vacuum
    c.enabled = False
    cmd = c.step(_sensors(lane_present=(True,)))
    assert cmd.enable is False
    assert cmd.tcp_target == held_target                 # pose held
    assert cmd.vacuum_cmd == held_vac                    # vacuum unchanged
    assert cmd.lane_release == (False,) * geo.N_LANES
    assert cmd.speed_scale == 0.0
    assert c.time_ns() > 0                                # the clock still free-runs (P1)


def test_loaded_carry_is_derated():
    c = MockCellController()
    empty = c.step(_sensors(part_held=False))
    assert empty.speed_scale == 1.0
    loaded = c.step(_sensors(part_held=True))
    assert loaded.speed_scale == geo.LOADED_SPEED_SCALE  # refinement B
