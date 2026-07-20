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
by tests/test_geometry_reach.py. The two full pallets around one fixed base are
still physically tight (a real UR20 palletizing a 1.2x1.0x1.15 m stack is near
its reach limit -- expect a pedestal/rail/closer pallet in the real cell), but
carry paths now clear the stack via PALLET_CLEAR_Z_M and eval9 checks that the
carried box never interpenetrates a placed one. REPLACE with the surveyed
layout when available.
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
# shipper 310x190x230 mm, gross ~6.9 kg.
BOX_LWH_M = (0.310, 0.190, 0.230)
BOX_MASS_KG = 6.9                      # gross weight -> refinement B derate
# SIMPLIFIED (human-directed): a compact 3x3 layer (9 boxes) instead of the full
# 18-box 1200x1000 layer. The smaller deck keeps every slot well inside the reach
# envelope so the arm holds elbow-up even on the top/far layers. Deck = 3 boxes
# long x 3 wide + a small lip. 9/layer x 5 = 45/pallet.
PALLET_LW_M = (3 * 0.310 + 0.02, 3 * 0.190 + 0.02)   # (0.95, 0.59)
LAYERS = 5

# --- PLACEHOLDER layout (reach-verified; see banner) --------------------------
# SIMPLIFIED CELL (human-directed): ONE infeed source -> TWO pallet stations.
# The single lane feeds the ACTIVE pallet; when it tops out the robot switches to
# the other; BOTH full BLOCKS the lane until a swap ack (P8). Counts flow from
# N_LANES / N_PALLETS so the topology stays cheap to change again.
N_LANES = 1
N_PALLETS = 2

TOOL_DOWN = (np.pi, 0.0, 0.0)          # rotation vector: gripper +z points down
MOUNT_HEIGHT_M = 0.735                 # pedestal: base at ~mid-stack (0.16 + 1.15/2)
REACH_LIMIT_M = 0.95 * REACH_M         # keep a margin under the 1.75 m envelope

# one infeed stop in the front arc (base frame); box picked from a stopped lane
LANE_STOP_XYZ = [
    (0.85, 0.00, 0.10),
]
# two pallets, ONE ON EITHER SIDE of the robot (+Y left, -Y right) so they never
# interfere and the single front lane stays clear. Position (where the pallet
# sits) is decoupled from orientation (how it is rotated): each pallet is turned
# so its SHORT side (width) faces the robot radially and its LENGTH runs
# tangentially -- the geometry-banner intent, and it shortens the worst-case reach
# to the far corner (btw note: better reach). Orientation = position + 90 deg.
PALLET_POS_RAD = [np.deg2rad(90.0), np.deg2rad(-90.0)]        # where each pallet sits
PALLET_YAW_RAD = [p + np.pi / 2 for p in PALLET_POS_RAD]      # deck/slot orientation
PALLET_CENTER_R_M = 0.90               # base -> pallet-center radial distance
DECK_Z_M = -0.55                       # pallet deck height in the base frame

APPROACH_CLEARANCE_M = 0.12            # hover height above a pick/place point
# Carry via height OVER a pallet: high enough that the carried box -- which hangs
# BOX_H below the tool -- has its BASE above the FULL stack top (deck + all layers)
# so every horizontal traverse clears already-placed boxes (P4 collision floor;
# eval9). Hence (LAYERS + 1) box-heights + a margin. Descent into a slot is then
# straight vertical.
PALLET_CLEAR_Z_M = DECK_Z_M + (LAYERS + 1) * BOX_LWH_M[2] + 0.06
CARRY_HEIGHT_M = PALLET_CLEAR_Z_M      # lane lift rises to the same clear height
# Home doubles as the between-cycles VIA point. Keep it WIDE of the base: a close,
# low rest pose (e.g. 0.4,0,0.4) lands in a 6-axis robot's inner-radius dead-zone,
# where numerical IK finds no solution and grinds -- stalling the render loop. This
# point sits comfortably in every backend's envelope.
HOME_XYZ = (0.60, 0.0, 0.60)
REACH_TOL_M = 0.006                    # controller sequencing tolerance
# gross ~6.9 kg on a 20 kg robot is a light load -> mild derate (placeholder).
LOADED_SPEED_SCALE = 0.75              # refinement B (revisit with the real curve)

# the one pattern, shared by controller (planning) and plant (adjudication)
PATTERN: list[Slot] = column_pattern(BOX_LWH_M, PALLET_LW_M, LAYERS, N_PALLETS)
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
    """Pallet-frame slot -> base-frame TCP pose. The pallet frame's LENGTH axis
    (sx, 0..pl) maps to the radial unit u and its WIDTH axis (sy, 0..pw) to the
    tangential unit t -- EXACTLY how the viewer draws the deck (BoxGeometry(pl, pw)
    rotated by yaw: local-x=pl -> u, local-y=pw -> t). Keeping this consistent is
    what makes the boxes tile the deck instead of sitting 90 deg across it. The box
    is centered on the pallet; orientation stays tool-down (the tool descends from
    above), the box's own world yaw is applied at render time."""
    pl, pw = PALLET_LW_M
    pos = PALLET_POS_RAD[slot.pallet]           # WHERE the pallet sits (center angle)
    yaw = PALLET_YAW_RAD[slot.pallet]           # HOW it is oriented (deck local-x dir)
    cx = PALLET_CENTER_R_M * np.cos(pos)
    cy = PALLET_CENTER_R_M * np.sin(pos)
    u = (np.cos(yaw), np.sin(yaw))              # deck local-x (length pl) direction
    t = (-np.sin(yaw), np.cos(yaw))             # deck local-y (width pw) direction
    sx, sy, sz, _boxyaw = slot.pose
    d_len = sx - pl / 2.0                        # length offset -> along u (radial)
    d_wid = sy - pw / 2.0                        # width offset  -> along t (tangential)
    x = cx + d_len * u[0] + d_wid * t[0]
    y = cy + d_len * u[1] + d_wid * t[1]
    # z is the TCP target = the box TOP when the box rests in this slot. The vacuum
    # tool grips the top, so the box hangs BOX_H below the TCP; placing the tool at
    # box-top makes the box settle with its BASE on the layer below (DECK + sz), no
    # penetration and no post-placement jump (the render draws the box below the
    # tool the same way).
    z = DECK_Z_M + sz + BOX_LWH_M[2]
    return _pose((x, y, z))


def slot_approach_pose(slot: Slot) -> Pose:
    x, y, z, *_ = slot_world_pose(slot)
    return _pose((x, y, z + APPROACH_CLEARANCE_M))


def slot_clear_pose(slot: Slot) -> Pose:
    """Via-point directly above a slot at PALLET_CLEAR_Z_M: the horizontal traverse
    across the pallet happens HERE (above the full stack), then the tool descends
    straight down to slot_world_pose. Keeps every over-pallet move above the
    collision floor (P4) so the carried box never sweeps through a placed one
    (P8; eval9)."""
    x, y, _z, *_ = slot_world_pose(slot)
    return _pose((x, y, PALLET_CLEAR_Z_M))


def next_open_slot(pallet: int, count: int) -> Slot | None:
    """The `count`-th slot of `pallet` in place order (from Sensors.pallet_count),
    or None if the pattern is full. The plant confirms the fill; the controller
    only proposes the target (P2/P3)."""
    slots = [s for s in PATTERN if s.pallet == pallet]
    return slots[count] if count < len(slots) else None
