"""Run the palletizer-hil cell with the WEB (Three.js) renderer -- GPU-free, no Isaac.

The SAME KinematicPalletCell + MockCellController (or, in phase 5, a live TwinCAT
PLC over ADS) as every other backend; only the renderer is a browser viewer
streamed over WebSocket. Controller invariance is the Phase-4 gate.

    python scripts/run_web_cell.py                       # mock controller, http://127.0.0.1:8080
    python scripts/run_web_cell.py --port 9000
    python scripts/run_web_cell.py --realbot             # real UR20 CAD (needs static/robot/*.glb)
    python scripts/run_web_cell.py --plc 5.1.204.123.1.1 # live TwinCAT over ADS (phase 5)

Needs the [web] extra:  pip install -e ".[web]"
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from palhil.render.web.server import serve  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="palletizer-hil cell web viewer")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--plc", default=None, metavar="AMS_NET_ID",
                    help="drive the cell with a live TwinCAT PLC over ADS (phase 5; else mock)")
    ap.add_argument("--realbot", action="store_true",
                    help="render the real UR20 CAD (loads per-link glTF); else a stylized arm")
    args = ap.parse_args()
    serve(args.host, args.port, args.plc, "real" if args.realbot else "stylized")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
