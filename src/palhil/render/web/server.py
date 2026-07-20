"""Web render backend for the palletizer cell (GPU-free, no Isaac, no CDN).

Runs the SAME KinematicPalletCell + MockCellController (or, in phase 5, the live
TwinCAT PLC over ADS) headless and streams JSON snapshots to a Three.js viewer at
~30 Hz. This is the RENDER seam only: the controller, the plant contract, and the
tag map are byte-identical to every other backend (controller invariance is the
Phase-4 gate). The browser is a passive snapshot consumer (P2).

Joint angles for the articulated view are recovered from the TCP by display-side
IK (ur20_kinematics.ik) with a held seed -- the same branch discipline (P4) the
controller uses, so the rendered arm never flips mid-carry. aiohttp is imported
lazily inside serve()/make_app(), so cell_config()/snapshot() stay importable and
testable without the optional [web] dependency.
"""
from __future__ import annotations

import asyncio
import json
import os

import numpy as np

from ... import geometry as geo
from ...plant import ur20_pose
from ...plant.cell_plant import KinematicPalletCell
from ...plant.ur20_kinematics import DH, REACH_M, Unreachable, ik
from ...plc.cell_controller import MockCellController

STATIC = os.path.join(os.path.dirname(__file__), "static")
SNAP_HZ = 30
CYCLE_S = 0.002                                   # plant/controller cycle (P1)
CYCLES_PER_FRAME = int(round((1.0 / SNAP_HZ) / CYCLE_S))   # ~16 -> near real-time


def _rotvec_to_T(pose) -> np.ndarray:
    x, y, z, rx, ry, rz = pose
    v = np.array([rx, ry, rz], float)
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        R = np.eye(3)
    else:
        k = v / th
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (x, y, z)
    return T


def display_ik(tcp_pose, seed):
    """Recover joints for rendering. The branch is chosen ELBOW-UP once (the
    practical palletizing posture) when unseeded, then held by nearest-seed
    continuity so the arm stays elbow-up and does not flip mid-cycle. With the
    two-pallets-either-side layout this measured 0 snaps over a full run (the arm
    no longer swings ~180 deg across the cell). These joints are display-only --
    the plant tracks the TCP directly, so a display reconfiguration is never a
    control effect. Falls back to the seed if the pose is momentarily unreachable
    (never crashes the view)."""
    try:
        # Restrict to the elbow-UP branch EVERY frame (nearest-seed only WITHIN that
        # set), so the arm can never drift elbow-down on the far/high layers -- then
        # hold continuity inside the up-set. On the compact 3x3 deck elbow-up is
        # reachable at every slot, so this stays snap-free.
        q = ik(_rotvec_to_T(tcp_pose), q_seed=seed, prefer_elbow_up=True)
        return q, q
    except Unreachable:
        return (seed if seed is not None else np.zeros(6)), seed


ABB_LINK_NAMES = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]


def _links_present(subdir: str, names) -> bool:
    d = os.path.join(STATIC, subdir)
    return all(os.path.exists(os.path.join(d, nm + ".glb")) for nm in names)


def cell_config(robot_model: str = "stylized") -> dict:
    """Geometry the viewer needs, derived from the ONE shared map (geometry.py),
    so the view can never drift from the plant (P2). If 'real'/'abb' is asked but
    the per-link glTF is not built, downgrade to 'stylized' rather than render a
    broken arm (honest fallback)."""
    if robot_model == "real" and not _links_present("robot", ur20_pose.LINK_NAMES):
        print("[web] --realbot: per-link glTF missing -> stylized arm (build them "
              "from per-link STEP: scripts/build_ur20_glb.py). Real pallets/boxes stay.")
        robot_model = "stylized"
    if robot_model == "abb" and not _links_present("robot_abb", ABB_LINK_NAMES):
        print("[web] --realbot-abb: IRB2600 glTF missing -> stylized arm "
              "(run scripts/build_irb2600_glb.py). Pallets/boxes stay.")
        robot_model = "stylized"
    pl, pw = geo.PALLET_LW_M
    pallets = []
    for p in range(geo.N_PALLETS):
        pos = float(geo.PALLET_POS_RAD[p])              # where the pallet sits
        yaw = float(geo.PALLET_YAW_RAD[p])              # how the deck is oriented
        cx = geo.PALLET_CENTER_R_M * np.cos(pos)
        cy = geo.PALLET_CENTER_R_M * np.sin(pos)
        pallets.append({"center": [round(cx, 4), round(cy, 4), round(geo.DECK_Z_M, 4)],
                        "yaw": round(yaw, 4), "lw": [pl, pw], "h": 0.16})
    return {
        "robot_model": robot_model,
        "mount_height": geo.MOUNT_HEIGHT_M,
        "reach": REACH_M,
        "box": list(geo.BOX_LWH_M),
        "capacity": geo.CAPACITY_PER_PALLET,
        "lanes": [[round(v, 4) for v in geo.LANE_STOP_XYZ[i]] for i in range(geo.N_LANES)],
        "pallets": pallets,
        "dh": [[float(DH[i, 0]), float(DH[i, 1]), float(DH[i, 2])] for i in range(6)],
        "link_names": ABB_LINK_NAMES if robot_model == "abb" else ur20_pose.LINK_NAMES,
    }


def snapshot(plant, joints, carry_yaw: float = 0.0) -> dict:
    """JSON-safe snapshot of the plant, built from the public seam only (P2).

    carry_yaw is the world yaw the carried box will be PLACED at -- render-only
    metadata so the viewer can rotate the box in flight to its final orientation
    instead of snapping 90 deg at placement. It never feeds control."""
    s = plant.sense()
    led = plant.ledger()
    placed = []
    for slot in plant.placed_slots():
        x, y, z = geo.slot_world_pose(slot)[:3]
        wyaw = float(geo.PALLET_YAW_RAD[slot.pallet] + slot.pose[3])
        placed.append([round(x, 4), round(y, 4), round(z, 4), round(wyaw, 4)])
    lanes = [[i, *[round(v, 4) for v in geo.LANE_STOP_XYZ[i]]]
             for i, present in enumerate(s.lane_present) if present]
    carry = None
    if s.part_held:
        x, y, z = s.tcp_pose[:3]
        carry = [round(x, 4), round(y, 4), round(z - geo.BOX_LWH_M[2] / 2, 4),
                 round(float(carry_yaw), 4)]
    return {
        "t": round(plant.t_ns / 1e9, 3),
        "tcp": [round(float(v), 4) for v in s.tcp_pose],
        "joints": [round(float(a), 5) for a in joints],
        "vacuum": bool(s.vacuum_on),
        "part_held": bool(s.part_held),
        "lanes": lanes,
        "carry": carry,
        "placed": placed,
        "pallet_count": list(s.pallet_count),
        "pallet_full": [bool(f) for f in s.pallet_full],
        "ledger": {
            "infed": led.infed, "on_lane": led.on_lane, "on_gripper": led.on_gripper,
            "on_pallet": led.on_pallet, "rejected": led.rejected,
            "conserved": bool(led.conserved()), "reach": int(plant.reach_violations),
            "collision": int(getattr(plant, "collision_violations", 0)),
        },
    }


async def _broadcast(clients, snap: dict) -> None:
    data = json.dumps(snap)
    dead = []
    for ws in clients:
        try:
            await ws.send_str(data)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)


async def control_loop(app, plc_ams: str | None = None) -> None:
    """Advance the cell in near real time (P1: dt from the controller clock) and
    broadcast at ~30 Hz. Mock controller by default; live TwinCAT over ADS if
    plc_ams is given (phase 5 -- runs the blocking ADS round-trip in a thread)."""
    plant = KinematicPalletCell(seed=3)
    link = ctrl = None
    if plc_ams:
        from ...plc.cell_link import CellAdsLink       # phase 5
        link = CellAdsLink(plc_ams)
        app["source"] = f"live TwinCAT {plc_ams}"
    else:
        ctrl = MockCellController()
        app["source"] = "mock controller"

    seed = None
    prev = ctrl.time_ns() if ctrl else 0
    try:
        while True:
            for _ in range(CYCLES_PER_FRAME):
                sensors = plant.sense()
                if link is None:
                    cmd = ctrl.step(sensors)
                    now = ctrl.time_ns()
                    dt = (now - prev) / 1e9
                    prev = now
                    plant.actuate(cmd)
                    if dt > 0:
                        plant.step(dt)
                else:
                    def _io():                          # blocking ADS round-trip (phase 5)
                        return link.exchange(sensors)
                    cmd, dt = await asyncio.to_thread(_io)
                    plant.actuate(cmd)
                    if dt > 0:
                        plant.step(dt)
            tcp = plant.sense().tcp_pose
            abb_links = None
            if app.get("robot") == "abb":            # ABB backend: MuJoCo IK, no UR
                from ...plant import abb_kinematics as abb
                q = abb.ik(tcp[:3], seed)
                if q is not None:
                    seed = q
                joints = [0.0] * 6                     # unused by the ABB viewer
                if seed is not None:
                    abb_links = abb.link_transforms(seed)   # per-link world transforms
            else:
                joints, seed = display_ik(tcp, seed)
            # render-only: the yaw the carried box will be placed at, so the viewer
            # can rotate it in flight (mock loop only exposes the controller target;
            # a live PLC does not, so it falls back to 0 -- purely cosmetic).
            carry_yaw = 0.0
            if ctrl is not None and getattr(ctrl, "_slot", None) is not None \
                    and ctrl.pallet is not None:
                carry_yaw = float(geo.PALLET_YAW_RAD[ctrl.pallet] + ctrl._slot.pose[3])
            snap = snapshot(plant, joints, carry_yaw)
            if abb_links is not None:
                snap["links"] = [[[round(v, 4) for v in pos], [round(v, 5) for v in quat]]
                                 for pos, quat in abb_links]
            snap["source"] = app.get("source", "mock controller")
            await _broadcast(app["clients"], snap)
            await asyncio.sleep(1.0 / SNAP_HZ)
    except asyncio.CancelledError:
        if link is not None and hasattr(link, "close"):
            link.close()
        raise


def make_app(plc_ams: str | None = None, robot: str = "stylized"):
    from aiohttp import WSMsgType, web

    app = web.Application()
    app["clients"] = set()
    app["robot"] = robot

    async def _index(request):
        return web.FileResponse(os.path.join(STATIC, "viewer.html"))

    async def _config(request):
        return web.json_response(cell_config(request.app["robot"]))

    async def _ws(request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        request.app["clients"].add(ws)
        try:
            async for msg in ws:                        # viewer is read-only; just drain
                if msg.type == WSMsgType.ERROR:
                    break
        finally:
            request.app["clients"].discard(ws)
        return ws

    app.router.add_get("/", _index)
    app.router.add_get("/config.json", _config)
    app.router.add_get("/ws", _ws)
    app.router.add_static("/static/", STATIC)

    async def _start(a):
        a["loop_task"] = asyncio.create_task(control_loop(a, plc_ams))

    async def _stop(a):
        a["loop_task"].cancel()
        try:
            await a["loop_task"]
        except asyncio.CancelledError:
            pass

    app.on_startup.append(_start)
    app.on_cleanup.append(_stop)
    return app


def serve(host: str = "127.0.0.1", port: int = 8080,
          plc_ams: str | None = None, robot: str = "stylized") -> None:
    from aiohttp import web
    print(f"[web] palletizer cell viewer -> http://{host}:{port}  "
          f"({'live TwinCAT ' + plc_ams if plc_ams else 'mock controller'}, robot={robot})")
    web.run_app(make_app(plc_ams, robot), host=host, port=port, print=None)
