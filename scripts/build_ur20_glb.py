"""Convert the UR20 STEP CAD -> per-link glTF for the --realbot web viewer.

Port of delta-hil's build_robot_glb.py. Output goes to
src/palhil/render/web/static/robot/<link>.glb, one file per DH link, named to
match plant/ur20_pose.LINK_NAMES so the viewer's JS applies M_i = frame_i(q) @
frame_i(0)^-1 to each. Meshes must be in the robot BASE frame at the HOME (q=0)
pose (metres) -- exactly what ur20_pose.link_transforms expects.

    pip install -e ".[cad]"          # cascadio + trimesh
    python scripts/build_ur20_glb.py

Two input shapes are handled:
  (a) per-link STEP files in assets/step/ (e.g. base.step, shoulder.step, ...):
      each is converted directly to <name>.glb (the clean, delta-hil path).
  (b) a single assembly STEP (assets/step/UR20.step/UR20.step): converted whole,
      then split by scene geometry into the 6+1 links IF it exposes >=7 named/
      separable solids. A fused single solid CANNOT be articulated -- the script
      says so and writes ur20_full.glb (viewer shows a rigid mesh moved by TCP).

Regenerated glb files are meant to be committed so --realbot works offline.
"""
import glob
import os
import sys

SRC_DIR = "assets/step"
OUT = "src/palhil/render/web/static/robot"
TOL_LINEAR, TOL_ANGULAR = 0.5, 0.6      # mm / rad -- coarser = lighter glTF
LINK_NAMES = ["base", "shoulder", "upper_arm", "forearm", "wrist1", "wrist2", "flange"]


def _find_step_files():
    pats = ["*.step", "*.STEP", "*/*.step", "*/*.STEP"]
    files = []
    for p in pats:
        files += glob.glob(os.path.join(SRC_DIR, p))
    # keep real files only: the vendor drop nests the STEP inside a directory that
    # is itself named "UR20.step", which the glob would otherwise return as a file.
    return sorted(f for f in set(files) if os.path.isfile(f))


def main() -> int:
    try:
        import cascadio  # noqa: F401
    except ImportError:
        print("needs: pip install -e \".[cad]\"  (cascadio + trimesh)", file=sys.stderr)
        return 1
    import cascadio
    files = _find_step_files()
    if not files:
        print(f"no STEP files under {SRC_DIR}/", file=sys.stderr)
        return 1
    os.makedirs(OUT, exist_ok=True)

    per_link = [f for f in files if os.path.splitext(os.path.basename(f))[0].lower() in LINK_NAMES]
    if per_link:                                    # (a) clean per-link path
        total = 0
        for f in per_link:
            name = os.path.splitext(os.path.basename(f))[0].lower()
            out = os.path.join(OUT, name + ".glb")
            cascadio.step_to_glb(f, out, TOL_LINEAR, TOL_ANGULAR)
            total += os.path.getsize(out)
        print(f"wrote {len(per_link)} per-link glTF -> {OUT}  ({total/1e6:.2f} MB)")
        return 0

    # (b) single assembly STEP -> whole glb. The z-order split into DH links is a
    # FRAGILE heuristic: a fused vendor assembly exports its solids around the CAD
    # origin, not at each DH frame_i(0), so M_i = frame_i(q) @ frame_i(0)^-1
    # SCATTERS them in the viewer (verified on the UR20 vendor STEP). It is only
    # meaningful if the STEP's solids already sit at their home-frame locations, so
    # it is now OPT-IN behind --split. Default: write ur20_full.glb only; the
    # viewer needs per-link STEP (branch a) for a correct articulated --realbot.
    do_split = "--split" in sys.argv
    src = files[0]
    full = os.path.join(OUT, "ur20_full.glb")
    cascadio.step_to_glb(src, full, TOL_LINEAR, TOL_ANGULAR)
    print(f"converted {src} -> {full}  ({os.path.getsize(full)/1e6:.2f} MB)")
    if not do_split:
        print("[split] fused assembly: NOT auto-splitting (the z-order heuristic "
              "scatters a fused STEP). For an articulated --realbot, export per-link "
              f"STEP named {LINK_NAMES} into assets/step/ and rerun. Pass --split to "
              "force the heuristic anyway (verify visually).")
        return 0
    try:
        import trimesh
        scene = trimesh.load(full)
        geoms = list(getattr(scene, "geometry", {}).items())
    except Exception as exc:
        geoms = []
        print(f"[split] trimesh unavailable/failed ({exc})", file=sys.stderr)

    if len(geoms) >= len(LINK_NAMES):
        # Heuristic: order solids along the base->flange axis (z in the base frame)
        # and map to the DH links. VERIFY visually -- a wrong grouping shows as a
        # link that moves with the wrong joint.
        ordered = sorted(geoms, key=lambda kv: kv[1].centroid[2])
        # collapse extra solids into the nearest of 7 buckets by index
        import numpy as np
        idx = (np.linspace(0, len(LINK_NAMES) - 1, len(ordered))).round().astype(int)
        buckets = {i: [] for i in range(len(LINK_NAMES))}
        for (name, mesh), b in zip(ordered, idx):
            buckets[int(b)].append(mesh)
        for i, name in enumerate(LINK_NAMES):
            if not buckets[i]:
                continue
            merged = trimesh.util.concatenate(buckets[i])
            merged.export(os.path.join(OUT, name + ".glb"))
        print(f"[split] wrote {len(LINK_NAMES)} link glTF by z-order HEURISTIC -- "
              f"verify in --realbot; if links move wrongly, provide per-link STEP "
              f"named {LINK_NAMES}.")
        return 0

    print("[split] single fused solid: cannot articulate. --realbot will show a "
          "rigid UR20 mesh (ur20_full.glb) posed at the TCP. For an articulated "
          f"arm, drop per-link STEP files named {LINK_NAMES} into assets/step/.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
