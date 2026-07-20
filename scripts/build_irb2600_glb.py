"""Convert the ROS-Industrial IRB2600 per-link STL meshes -> glTF for the ABB
viewer (--realbot-abb). Mirror of build_ur20_glb.py, but the meshes come already
split per link (base_link, link_1..6) and authored in each LINK frame, so no
heuristic splitting is needed -- the viewer places each mesh at the link's world
transform from MuJoCo FK (plant/abb_kinematics.link_transforms).

    pip install -e ".[cad]"        # trimesh
    python scripts/build_irb2600_glb.py

STLs are ABB's, in metres (ROS convention). Output -> static/robot_abb/<link>.glb.
"""
import os
import sys

SRC = "assets/robot_abb/meshes"
OUT = "src/palhil/render/web/static/robot_abb"
LINKS = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]


def main() -> int:
    try:
        import trimesh
    except ImportError:
        print("needs: pip install -e \".[cad]\"  (trimesh)", file=sys.stderr)
        return 1
    os.makedirs(OUT, exist_ok=True)
    total = 0
    for name in LINKS:
        src = os.path.join(SRC, name + ".stl")
        if not os.path.exists(src):
            print(f"missing {src}", file=sys.stderr)
            return 1
        mesh = trimesh.load(src, force="mesh")
        out = os.path.join(OUT, name + ".glb")
        mesh.export(out)
        total += os.path.getsize(out)
        ext = mesh.extents
        print(f"  {name:10s} -> {out}  bbox(m) {ext.round(3)}  {os.path.getsize(out)/1e3:.0f} kB")
    print(f"wrote {len(LINKS)} link glTF -> {OUT}  ({total/1e6:.2f} MB total)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
