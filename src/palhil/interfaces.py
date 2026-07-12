"""The two seams. Everything upstream is written against these Protocols;
swapping mock <-> TwinCAT (controller) or kinematic <-> MuJoCo <-> Isaac
(plant) must never change a caller. Mirrors delta-hil's interfaces.py.

Cites: P1, P2 (the I/O contract IS this file), P7 (the real program is what
is under test, so the seam must be exactly the tag map and nothing more).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol


@dataclass
class Sensors:
    """Plant -> controller, one sample. Units: m, m/s, rad, bool. (P2)"""
    tcp_pose: tuple[float, float, float, float, float, float]  # x y z rx ry rz
    joints: tuple[float, ...]                                  # 6 joint angles
    vacuum_on: bool
    part_held: bool
    lane_present: tuple[bool, bool, bool]     # box at each lane stop
    pallet_count: tuple[int, int, int]        # boxes placed per pallet
    pallet_full: tuple[bool, bool, bool]
    time_ns: int                              # the CONTROLLER's clock (P1)


@dataclass
class Commands:
    """Controller -> plant, one sample. (P2)"""
    tcp_target: tuple[float, float, float, float, float, float]
    speed_scale: float = 1.0                  # derated when loaded (refinement B)
    vacuum_cmd: bool = False
    lane_release: tuple[bool, bool, bool] = (False, False, False)
    pallet_swap_ack: tuple[bool, bool, bool] = (False, False, False)
    enable: bool = True


@dataclass
class Ledger:
    """P8: balances every step or the run is invalid."""
    infed: int = 0
    on_lane: int = 0
    on_gripper: int = 0
    on_pallet: int = 0
    rejected: int = 0

    def conserved(self) -> bool:
        return self.infed == self.on_lane + self.on_gripper + self.on_pallet + self.rejected


class Controller(Protocol):
    """MockCellController and the TwinCAT link both look like this."""
    def step(self, sensors: Sensors) -> Commands: ...
    def time_ns(self) -> int: ...             # P1: the one clock


class Plant(Protocol):
    """Kinematic / MuJoCo / Isaac plants all look like this."""
    def sense(self) -> Sensors: ...
    def actuate(self, cmd: Commands) -> None: ...
    def step(self, dt: float) -> None: ...    # advance ONLY on a controller tick
    def ledger(self) -> Ledger: ...
