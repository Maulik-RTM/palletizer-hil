# UR20 STEP files go here

Drop the UR20 STEP assembly (or per-link exports) in this folder. Phase 4 of
plan.md converts them:

- Split per link: Base, Shoulder, UpperArm, Forearm, Wrist1, Wrist2, Wrist3 (+ gripper).
- STEP -> glTF for the browser viewer (port delta-hil `scripts/build_robot_glb.py`,
  which uses the OCP/cadquery tessellation path).
- STEP -> USD for Isaac (port delta-hil `scripts/step_to_usd.py`).
- Record each link's mesh-frame -> DH-frame transform in one place
  (`plant/ur20_pose.py`), mirroring delta-hil `plant/irb360_pose.py`.

UR publishes STEP on their support site; per-link splitting is easiest in the
CAD tool before export.
