"""Guard: the controller must not import the plant (plan.md phase 2, cf.
delta-hil test_kinematic_guard.py). The mock is the golden reference the TwinCAT
FB mirrors 1:1; if it reached into the simulator it would no longer be a pure
Sensors->Commands function and controller-invariance (P2/P7) would be a fiction.
"""
import inspect
import subprocess
import sys

from palhil.plc import cell_controller


def test_controller_source_has_no_plant_reference():
    src = inspect.getsource(cell_controller)
    for banned in ("cell_plant", "KinematicPalletCell", "mujoco", "isaac"):
        assert banned not in src, f"plant leaked into the controller: {banned!r}"


def test_importing_controller_does_not_import_the_plant():
    # A fresh interpreter (the session's global sys.modules is polluted by the
    # eval tests that legitimately import the plant). geometry/pattern are pure
    # data and allowed; the plant simulator is not.
    code = (
        "import importlib, sys;"
        "importlib.import_module('palhil.plc.cell_controller');"
        "assert 'palhil.plant.cell_plant' not in sys.modules, 'controller imported the plant'"
    )
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
