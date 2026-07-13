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
    """Recover joints for rendering, PREFERRING the branch nearest the seed to keep
    the arm continuous. This reduces but does not eliminate flips: a wrist-
    singularity crossing (theta5 through 0) still reconfigures theta4/theta6 by ~pi
    on ~1% of frames -- a cosmetic snap in the view, never a control effect (the
    plant tracks the TCP directly; these joints are display-only). Falls back to
    the seed if the pose is momentarily unreachable (never crashes the view)."""
    try:
        q = ik(_rotvec_to_T(tcp_pose), q_seed=seed)
        return q, q
    except Unreachable:
        return (seed if seed is not None else np.zeros(6)), seed


def cell_config(robot_model: str = "stylized") -> dict:
    """Geometry the viewer needs, derived from the ONE shared map (geometry.py),
    so the view can never drift from the plant (P2)."""
    pl, pw = geo.PALLET_LW_M
    pallets = []
    for p in range(3):
        yaw = float(geo.PALLET_YAW_RAD[p])
        cx = geo.PALLET_CENTER_R_M * np.cos(yaw)
        cy = geo.PALLET_CENTER_R_M * np.sin(yaw)
        pallets.append({"center": [round(cx, 4), round(cy, 4), round(geo.DECK_Z_M, 4)],
                        "yaw": round(yaw, 4), "lw": [pl, pw], "h": 0.16})
    return {
        "robot_model": robot_model,
        "mount_height": geo.MOUNT_HEIGHT_M,
        "reach": REACH_M,
        "box": list(geo.BOX_LWH_M),
        "lanes": [[round(v, 4) for v in geo.LANE_STOP_XYZ[i]] for i in range(3)],
        "pallets": pallets,
        "dh": [[float(DH[i, 0]), float(DH[i, 1]), float(DH[i, 2])] for i in range(6)],
        "link_names": ur20_pose.LINK_NAMES,
    }


def snapshot(plant, joints) -> dict:
    """JSON-safe snapshot of the plant, built from the public seam only (P2)."""
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
        carry = [round(x, 4), round(y, 4), round(z - geo.BOX_LWH_M[2] / 2, 4), 0.0]
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
            joints, seed = display_ik(plant.sense().tcp_pose, seed)
            snap = snapshot(plant, joints)
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
