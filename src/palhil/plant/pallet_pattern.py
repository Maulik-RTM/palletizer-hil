"""Pallet pattern generation (P8). BUILD TASK (plan.md phase 3).

A pattern is data, not behavior: an ordered list of slots, each a box pose on
the pallet plus which layer it belongs to. The plant adjudicates whether a
placed box actually LANDED in its slot (P3/P8); the controller just requests
"next open slot on pallet k".

Start with a column stack pattern per SKU; interlocked (rotated alternate
layers) is a stretch goal. Slots must never overlap (eval-7).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Slot:
    pallet: int          # 0..2
    layer: int
    index: int
    pose: tuple[float, float, float, float]   # x, y, z, yaw -- pallet frame


def _base_layer(box_lwh_m: tuple[float, float, float],
                pallet_lw_m: tuple[float, float]) -> list[tuple[float, float, float]]:
    """One layer's box-center cells as (x, y, yaw) in the pallet frame (X=length,
    Y=width). The real pattern (assets/Palletizer-Master_Format.xlsx): a main grid
    of un-rotated boxes plus a rotated strip in the leftover length, e.g. for the
    310x190 shipper on a 1200x1000 pallet -> 3x5 = 15 width-wise boxes and a
    strip of 3 length-wise boxes = 18/layer.
    """
    bl, bw, _bh = box_lwh_m
    pl, pw = pallet_lw_m
    eps = 1e-9

    ncols = int((pl + eps) // bl)          # un-rotated boxes along the length (X)
    nrows = int((pw + eps) // bw)          # ... along the width (Y)
    if ncols < 1 or nrows < 1:
        raise ValueError("box does not fit on the pallet")

    cells: list[tuple[float, float, float]] = []
    for col in range(ncols):               # main block, yaw 0: L||X, W||Y
        for row in range(nrows):
            cells.append(((col + 0.5) * bl, (row + 0.5) * bw, 0.0))

    rem_x = pl - ncols * bl                 # leftover length for rotated boxes
    if rem_x + eps >= bw:                   # a rotated column fits (W||X, L||Y)
        strip_rows = int((pw + eps) // bl)
        x_strip = ncols * bl + bw / 2.0
        for k in range(strip_rows):
            cells.append((x_strip, (k + 0.5) * bl, math.pi / 2))

    # safe PLACE order within a layer: far corner first (largest x, then y) so the
    # arm never reaches over an already-placed box (P4). NB: "far" assumes the
    # robot approaches from the -X (low length) side -- revisit once the cell
    # layout / robot mount side is fixed (still on the ask-the-human list).
    cells.sort(key=lambda c: (-c[0], -c[1]))
    return cells


def column_pattern(box_lwh_m: tuple[float, float, float],
                   pallet_lw_m: tuple[float, float],
                   layers: int) -> list[Slot]:
    """Column-stack pattern for one SKU across all three pallets (P8).

    Builds the real 18-box base layer (_base_layer) and stacks `layers` identical
    copies. Layers are identical, so a box in layer L rests directly on the box
    at the same (x, y, yaw) in layer L-1 -- the support relation the plant checks
    (P3/P8). Slots come out in PLACE order: bottom layer first, far corner first.

    Slot.pose = (x, y, z, yaw) in the pallet frame: (x, y) box-center on the deck,
    z the box base height for that layer, yaw 0 (width-wise) or pi/2 (length-wise).
    """
    base = _base_layer(box_lwh_m, pallet_lw_m)
    bh = box_lwh_m[2]
    slots: list[Slot] = []
    for pallet in range(3):
        idx = 0
        for layer in range(layers):
            for x, y, yaw in base:
                slots.append(Slot(pallet=pallet, layer=layer, index=idx,
                                  pose=(x, y, layer * bh, yaw)))
                idx += 1
    return slots
