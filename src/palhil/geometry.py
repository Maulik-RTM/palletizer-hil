"""Cell layout as pure data -- the shared world map (P2).

The controller plans TCP targets against this; the phase-3 plant adjudicates box
arrivals against the SAME constants, so a pick commanded at lane i lands where
the plant expects a box and a place lands in the slot the plant scores. No
control logic and no plant/physics lives here -- just geometry, so both seams can
import it without coupling (P2: one I/O contract, one world).

Frame: the ROBOT BASE frame (origin at the base flange, z up, x forward), which
is what ur20_kinematics.fk/ik use and what the plant integrates. MOUNT_HEIGHT_M
records the physical base height off the floor (informational for the desk loop).

============================ ENGINEERING PLACEHOLDER ==========================
Box / mass / pallet / pattern are REAL (assets/Palletizer-Master_Format.xlsx).
The cell LAYOUT below -- lane stops, pallet placement, mount height, tool length
-- is NOT from any source; it is a reach-verified placeholder (option (b)) so the
phase-3 desk loop can close. Every commanded target is checked <= REACH_LIMIT_M
by tests/test_geometry_reach.py. It is NOT collision-checked and the three full
pallets around one fixed base are physically tight (a real UR20 palletizing a
1.2x1.0x1.15 m stack is at its reach limit -- expect a pedestal/rail/closer
pallet in the real cell). REPLACE with the surveyed layout when available.
Still on the ask-the-human list: lane coordinates, mount height, gripper length.
===============================================================================

Cites: P2 (single shared map), P4 (targets carry a fixed tool-down orientation
inside the reach envelope), refinement B (LOADED_SPEED_SCALE derate).
"""
from __future__ import annotations

import numpy as np

from .plant.pallet_pattern import Slot, column_pattern
from .plant.ur20_kinematics import REACH_M

Pose = tuple[float, float, float, float, float, float]

# --- REAL product data (assets/Palletizer-Master_Format.xlsx, rows 1-12) ------
# shipper 310x190x230 mm, gross ~6.9 kg, 18/layer x 5 = 90 on a 1200x1000 pallet.
BOX_LWH_M = (0.310, 0.190, 0.230)
BOX_MASS_KG = 6.9                      # gross weight -> refinement B derate
PALLET_LW_M = (1.200, 1.000)           # data sheet (NOT EUR 1200x800 from plan.md)
LAYERS = 5

# --- PLACEHOLDER layout (reach-verified; see banner) --------------------------
TOOL_DOWN = (np.pi, 0.0, 0.0)          # rotation vector: gripper +z points down
MOUNT_HEIGHT_M = 0.735                 # pedestal: base at ~mid-stack (0.16 + 1.15/2)
REACH_LIMIT_M = 0.95 * REACH_M         # keep a margin under the 1.75 m envelope

# lanes: three stops in a front arc (base frame), boxes picked from a stopped lane
LANE_STOP_XYZ = [
    (0.75, -0.35, 0.10),
    (0.85, 0.00, 0.10),
    (0.75, 0.35, 0.10),
]
# pallets fanned around the back hemisphere, each rotated to FACE the base so its
# 1.0 m width runs radially (near) and 1.2 m length tangentially. lane i -> pallet i.
PALLET_YAW_RAD = [np.deg2rad(130.0), np.deg2rad(180.0), np.deg2rad(230.0)]
PALLET_CENTER_R_M = 0.90               # base -> pallet-center radial distance
DECK_Z_M = -0.55                       # pallet deck height in the base frame

APPROACH_CLEARANCE_M = 0.12            # hover height above a pick/place point
CARRY_HEIGHT_M = 0.45                  # via height for carries (over the stack, P4)
HOME_XYZ = (0.40, 0.0, 0.40)
REACH_TOL_M = 0.006                    # controller sequencing tolerance
# gross ~6.9 kg on a 20 kg robot is a light load -> mild derate (placeholder).
LOADED_SPEED_SCALE = 0.75              # refinement B (revisit with the real curve)

# the one pattern, shared by controller (planning) and plant (adjudication)
PATTERN: list[Slot] = column_pattern(BOX_LWH_M, PALLET_LW_M, LAYERS)
CAPACITY_PER_PALLET = sum(1 for s in PATTERN if s.pallet == 0)


def _pose(xyz: tuple[float, float, float]) -> Pose:
    return (xyz[0], xyz[1], xyz[2], *TOOL_DOWN)


def home_pose() -> Pose:
    return _pose(HOME_XYZ)


def lane_pick_pose(i: int) -> Pose:
    return _pose(LANE_STOP_XYZ[i])


def lane_approach_pose(i: int) -> Pose:
    x, y, z = LANE_STOP_XYZ[i]
    return _pose((x, y, z + APPROACH_CLEARANCE_M))


def carry_pose_over(xyz: tuple[float, float, float]) -> Pose:
    return _pose((xyz[0], xyz[1], CARRY_HEIGHT_M))


def slot_world_pose(slot: Slot) -> Pose:
    """Pallet-frame slot -> base-frame TCP pose. The pallet is rotated by its yaw
    so width (sy) runs radially and length (sx) tangentially; the box is centered
    on the pallet. Orientation stays tool-down (the tool descends from above)."""
    pl, pw = PALLET_LW_M
    yaw = PALLET_YAW_RAD[slot.pallet]
    cx = PALLET_CENTER_R_M * np.cos(yaw)
    cy = PALLET_CENTER_R_M * np.sin(yaw)
    u = (np.cos(yaw), np.sin(yaw))              # radial unit (width axis)
    t = (-np.sin(yaw), np.cos(yaw))             # tangential unit (length axis)
    sx, sy, sz, _boxyaw = slot.pose
    d_rad = sy - pw / 2.0                        # width offset from pallet center
    d_tan = sx - pl / 2.0                        # length offset from pallet center
    x = cx + d_rad * u[0] + d_tan * t[0]
    y = cy + d_rad * u[1] + d_tan * t[1]
    z = DECK_Z_M + sz
    return _pose((x, y, z))


def slot_approach_pose(slot: Slot) -> Pose:
    x, y, z, *_ = slot_world_pose(slot)
    return _pose((x, y, z + APPROACH_CLEARANCE_M))


def next_open_slot(pallet: int, count: int) -> Slot | None:
    """The `count`-th slot of `pallet` in place order (from Sensors.pallet_count),
    or None if the pattern is full. The plant confirms the fill; the controller
    only proposes the target (P2/P3)."""
    slots = [s for s in PATTERN if s.pallet == pallet]
    return slots[count] if count < len(slots) else None
