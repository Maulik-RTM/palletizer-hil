# Web render snapshot schema

The render seam (`palhil.render.web.server`) streams JSON to the Three.js viewer.
It is a **passive projection of the plant's public sensor seam** (P2): the server
reads `plant.sense()`, `plant.ledger()`, `plant.placed_slots()` and nothing
private, so the view can never see what the controller can't. The controller and
plant contract are byte-identical to every other backend (controller invariance).

Two messages: `config.json` (once, on load) and per-frame `snapshot` (~30 Hz).

## `GET /config.json` — static cell geometry (from `geometry.py`)

```jsonc
{
  "robot_model": "stylized" | "real",
  "mount_height": 0.735,            // m, pedestal height (plant z of the floor = -this)
  "reach": 1.75,                    // m, UR20 nominal reach
  "box": [0.310, 0.190, 0.230],     // shipper L, W, H (m)
  "lanes": [[x, y, z], x3],         // lane pick points, robot base frame (m)
  "pallets": [                      // x3
    { "center": [cx, cy, cz], "yaw": <rad>, "lw": [1.2, 1.0], "h": 0.16 }
  ],
  "dh": [[d, a, alpha], x6],        // UR20 DH rows (m, m, rad) -- JS FK uses these
  "link_names": ["base","shoulder","upper_arm","forearm","wrist1","wrist2","flange"]
}
```

Frame: **robot base frame, z-up, metres.** The viewer rotates it to Three.js
y-up via `(x,y,z) -> (x, z, -y)`.

## `ws://…/ws` — per-frame snapshot (~30 Hz)

```jsonc
{
  "t": 12.34,                       // sim seconds (plant.t_ns / 1e9)
  "tcp": [x,y,z, rx,ry,rz],         // TCP pose (m; rx,ry,rz = rotation vector)
  "joints": [q0..q5],               // rad, from display-side IK(tcp) with a held
                                    //   seed (P4 branch hold); the viewer's JS FK
                                    //   places the arm from these
  "vacuum": true,                   // vacuum commanded
  "part_held": true,                // a box is on the gripper
  "carry": [x, y, z, yaw] | null,   // carried box center (m), or null
  "lanes": [[i, x, y, z], ...],     // lanes currently presenting a box
  "placed": [[x, y, z, yaw], ...],  // every placed box: slot base xyz (m) + world yaw
  "pallet_count": [c0, c1, c2],     // boxes placed per pallet
  "pallet_full": [bool, bool, bool],
  "ledger": {                       // P8 conservation, surfaced live
    "infed": N, "on_lane": N, "on_gripper": N, "on_pallet": N, "rejected": N,
    "conserved": true,              // infed == lane+gripper+pallet+rejected
    "reach": 0                      // P4 reach-violation count
  },
  "source": "mock controller"       // or "live TwinCAT <AMS>" (phase 5)
}
```

Notes:
- `joints` are **display-only** — the kinematic plant tracks the TCP directly and
  reports `joints = 0` on the control seam; the render server recovers a posed
  configuration with `ur20_kinematics.ik`, preferring the branch nearest the
  previous frame's seed. This keeps the arm continuous most of the time but does
  **not** fully eliminate flips: a wrist-singularity crossing (θ5 through 0) still
  reconfigures θ4/θ6 by ~π on ~1% of frames — a cosmetic snap in the view, never a
  control effect. The `--realbot` viewer applies
  `M_i = frame_i(q) @ frame_i(0)^-1` (`plant/ur20_pose.py`, JS-ported in
  `viewer.html`) to each per-link glTF.
- `placed` grows monotonically to 3x90 = 270; the viewer pools meshes by index.
