#!/usr/bin/env python3
"""
Parametric, re-editable STL generator for the Macropad enclosure.

This script intentionally keeps all model logic in plain Python parameters so
future sessions can edit it without external CAD dependencies.
"""

from __future__ import annotations

import math
import re
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np
import trimesh


# -------- User-editable parameters --------
PCB_PATH = Path("/home/von/Desktop/Macropad Project/Macropad/Macropad.kicad_pcb")
PRODUCTION_BOM_PATH = Path("/home/von/Desktop/Macropad Project/Macropad/production/bom.csv")
PRODUCTION_POS_PATH = Path("/home/von/Desktop/Macropad Project/Macropad/production/positions.csv")
GERBER_EDGE_PATH = Path("/home/von/Desktop/Macropad Project/Macropad/gerber/Macropad-Edge_Cuts.gm1")
OUTPUT_BOTTOM_STL = Path("/home/von/Desktop/Macropad Project/case/macropad_case_bottom.stl")
OUTPUT_TOP_STL = Path("/home/von/Desktop/Macropad Project/case/macropad_case_top.stl")
OUTPUT_SIDE_BY_SIDE_STL = Path("/home/von/Desktop/Macropad Project/case/macropad_case_side_by_side.stl")
OUTPUT_ASSEMBLED_STL = Path("/home/von/Desktop/Macropad Project/case/macropad_case_assembled.stl")

WALL = 2.4
TOP_THICKNESS = 2.2
BOTTOM_THICKNESS = 2.4
PCB_CLEARANCE_XY = 1.0
CASE_HEIGHT = 26.0

# 4 degree typing/elevation angle requested.
TILT_DEG = 4.0

# MX/Kailh-compatible top cutouts (plate-style opening).
SWITCH_HOLE_SIZE = (14.1, 14.1)

# OLED/LCD flush mount zone at top.
# View window: what you actually see.
OLED_WINDOW_SIZE = (66.0, 50.0)
# Module face size tuned for a large rectangular display module (2.0in-class TFT/OLED board envelope).
LCD_MODULE_FACE_SIZE = (72.0, 56.0)
LCD_MODULE_FACE_CLEARANCE = 0.25
LCD_FLUSH_DEPTH = 1.6
# Hidden underside relief so solder joints/wires fit while top remains flush.
LCD_BACK_RELIEF_SIZE = (69.0, 38.0)
LCD_BACK_RELIEF_CLEARANCE = 0.6
LCD_BACK_RELIEF_DEPTH = 5.5
# Small tool/wire channel on the header side of the module.
LCD_SOLDER_CHANNEL_SIZE = (16.0, 6.0)
LCD_SOLDER_CHANNEL_Y_OFFSET = -28.0
OLED_WINDOW_ANCHOR_REF = "J302"
OLED_OFFSET_FROM_ANCHOR = (-39.10, 13.6)

# Local underside relief for LCD rear connector/solder joints.
LCD_BACK_PORT_ESCAPE_SIZE = (34.0, 12.0)  # (x span, y span)
LCD_BACK_PORT_ESCAPE_Y_OFFSET = -27.0     # toward top edge of case
LCD_BACK_PORT_ESCAPE_DEPTH = 5.5

# Rotary encoder top circle opening (missing previously).
ROTARY_HOLE_ANCHOR_REF = "SW401"
ROTARY_HOLE_DIAMETER = 8.0

# MSS-102545-14A-V-SMT side switch slot.
SIDE_SWITCH_SLOT = (4.6, 1.8)  # visible actuator slot: (y span, z span)
SIDE_SWITCH_ANCHOR_REF = "J302"
SIDE_SWITCH_FROM_ANCHOR_Y = 5.0
SIDE_SWITCH_Z = CASE_HEIGHT * 0.50
SIDE_SWITCH_BODY_CLEARANCE = (7.2, 2.8)  # hidden internal clearance: (y span, z span)

# Bottom reset-button access, aligned to PCB tactile switch.
RESET_BUTTON_ANCHOR_REF = "SW402"
RESET_BUTTON_HOLE_DIAMETER = 4.4
RESET_BUTTON_OFFSET = (0.0, 0.0)
# Ensure reset tunnel reaches through internal riser/supports.
RESET_BUTTON_TUNNEL_TOP_EXTRA = 1.2
# Recess opening a bit so short actuators are easier to use.
RESET_BUTTON_RECESS_DIAMETER = 7.0
RESET_BUTTON_RECESS_DEPTH = 1.4

# IR holes (from J201/J202 positions on PCB, front wall).
IR_HOLE_DIAMETER = 3.3
IR_HOLE_Z = CASE_HEIGHT * 0.50

# USB-C front cutout (from J402).
USB_CUTOUT_ANCHOR_REF = "J402"
USB_CUTOUT_SIZE = (9.2, 3.2)  # USB-C shell opening: (x span, z span)
USB_CUTOUT_Z = BOTTOM_THICKNESS + 6.0
USB_CUTOUT_INNER_SIZE = (8.6, 2.8)  # inner relief behind outer lip: (x span, z span)

# PCB mounting standoffs (from H1..H4 mounting holes).
PCB_STANDOFF_REFS = ("H1", "H2", "H3", "H4")
PCB_STANDOFF_DIAMETER = 7.0
PCB_STANDOFF_HOLE_DIAMETER = 3.4
PCB_STANDOFF_HEIGHT = 2.5

# External bottom stand block: this creates the typing tilt.
BOTTOM_STAND_CONTACT_SPAN_Y = 80.0  # effective front-to-stand contact distance
BOTTOM_STAND_DEPTH = 18.0
BOTTOM_STAND_MARGIN_X = 12.0
BOTTOM_STAND_FROM_USB_Y = 8.0
BOTTOM_STAND_X_SHIFT = 0.0

# LiPo battery cradle for a 3.7V ~2500mAh pack.
BATTERY_ANCHOR_REF = "J400"
BATTERY_OFFSET_FROM_ANCHOR = (28.0, 52.0)  # (x, y) from J400
BATTERY_POCKET_SIZE = (72.0, 52.0, 6.8)  # (x, y, z clearance)
BATTERY_POD_WALL = 1.8
BATTERY_FLOOR_THICKNESS = 1.6
BATTERY_TO_PCB_CLEARANCE = 1.2

# Split and preview spacing.
SPLIT_Z = CASE_HEIGHT * 0.72
PREVIEW_GAP_X = 14.0

# Snap-fit lip between bottom/top shells at the split seam.
SNAP_LIP_ENABLED = True
SNAP_LIP_INSET = 0.55
SNAP_LIP_THICKNESS = 1.10
SNAP_LIP_HEIGHT = 1.80
SNAP_LIP_CLEARANCE = 0.15
SNAP_LIP_GROOVE_EXTRA_DEPTH = 0.60

# Serviceable/re-openable snap tuning.
SNAP_RELEASE_GAP = 16.0
PRY_SLOT_WIDTH = 12.0
PRY_SLOT_HEIGHT = 2.8

# Mirror top-side cutouts across X to match board orientation.
MIRROR_TOP_FEATURES_X = True


@dataclass
class Footprint:
    ref: str
    value: str
    x: float
    y: float
    rot: float


def parse_edge_rect(kicad_text: str) -> Tuple[float, float, float, float]:
    # Expected form found in this PCB: (gr_rect (start x y) (end x y) ... (layer "Edge.Cuts"))
    rect_re = re.compile(
        r"\(gr_rect\s+\(start\s+([-0-9.]+)\s+([-0-9.]+)\)\s+\(end\s+([-0-9.]+)\s+([-0-9.]+)\)[^)]*\)"
    )
    for m in rect_re.finditer(kicad_text):
        snippet_start = m.start()
        snippet = kicad_text[snippet_start : snippet_start + 300]
        if '"Edge.Cuts"' in snippet:
            x1, y1, x2, y2 = map(float, m.groups())
            min_x, max_x = sorted((x1, x2))
            min_y, max_y = sorted((y1, y2))
            return min_x, min_y, max_x, max_y
    raise RuntimeError("Could not find Edge.Cuts gr_rect in PCB.")


def parse_footprints(kicad_lines: List[str]) -> Dict[str, Footprint]:
    footprints: Dict[str, Footprint] = {}
    i = 0
    while i < len(kicad_lines):
        line = kicad_lines[i]
        if line.strip().startswith("(footprint "):
            block: List[str] = [line]
            depth = line.count("(") - line.count(")")
            i += 1
            while i < len(kicad_lines) and depth > 0:
                l = kicad_lines[i]
                block.append(l)
                depth += l.count("(") - l.count(")")
                i += 1
            text = "\n".join(block)

            ref_m = re.search(r'\(property "Reference" "([^"]+)"', text)
            val_m = re.search(r'\(property "Value" "([^"]+)"', text)
            at_m = re.search(r'\n\s*\(at\s+([-0-9.]+)\s+([-0-9.]+)(?:\s+([-0-9.]+))?\)', text)
            if ref_m and at_m:
                ref = ref_m.group(1)
                val = val_m.group(1) if val_m else ""
                x = float(at_m.group(1))
                y = float(at_m.group(2))
                rot = float(at_m.group(3) or 0.0)
                footprints[ref] = Footprint(ref=ref, value=val, x=x, y=y, rot=rot)
        else:
            i += 1
    return footprints


def box_mesh(min_corner: Iterable[float], max_corner: Iterable[float]) -> trimesh.Trimesh:
    min_corner = np.array(list(min_corner), dtype=float)
    max_corner = np.array(list(max_corner), dtype=float)
    ext = max_corner - min_corner
    center = (min_corner + max_corner) / 2.0
    m = trimesh.creation.box(extents=ext)
    m.apply_translation(center)
    return m


def cylinder_y(center_xyz: Tuple[float, float, float], radius: float, length: float, sections: int = 48) -> trimesh.Trimesh:
    # trimesh cylinders are along +Z by default; rotate to +Y.
    c = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    rot = trimesh.transformations.rotation_matrix(math.pi / 2.0, [1, 0, 0])
    c.apply_transform(rot)
    c.apply_translation(center_xyz)
    return c


def cylinder_z(center_xyz: Tuple[float, float, float], radius: float, length: float, sections: int = 40) -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    c.apply_translation(center_xyz)
    return c


def boolean_diff(base: trimesh.Trimesh, cutters: List[trimesh.Trimesh]) -> trimesh.Trimesh:
    if not cutters:
        return base
    # Union cutters first for speed/stability.
    cutter_union = trimesh.boolean.union(cutters, engine="manifold")
    return trimesh.boolean.difference([base, cutter_union], engine="manifold")


def boolean_intersection(meshes: List[trimesh.Trimesh]) -> trimesh.Trimesh:
    return trimesh.boolean.intersection(meshes, engine="manifold")


def boolean_union(meshes: List[trimesh.Trimesh]) -> trimesh.Trimesh:
    return trimesh.boolean.union(meshes, engine="manifold")


def validate_reference_files() -> None:
    # User requested gerber/production directory as reference point.
    for p in (PRODUCTION_BOM_PATH, PRODUCTION_POS_PATH, GERBER_EDGE_PATH):
        if not p.exists():
            raise RuntimeError(f"Missing reference file: {p}")

    with PRODUCTION_BOM_PATH.open(newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    if not any((r.get("Footprint") or "").strip() == "Kailh_Hotswap_MX_1U_Custom" for r in rows):
        raise RuntimeError("Production BOM is missing Kailh hotswap footprint row.")


def main() -> None:
    validate_reference_files()
    kicad_text = PCB_PATH.read_text(encoding="utf-8")
    kicad_lines = kicad_text.splitlines()

    min_x, min_y, max_x, max_y = parse_edge_rect(kicad_text)
    fps = parse_footprints(kicad_lines)

    required = [
        "J201",
        "J202",
        "J302",
        "U901",
        "J400",
        "J402",
        "SW401",
        SIDE_SWITCH_ANCHOR_REF,
        RESET_BUTTON_ANCHOR_REF,
        USB_CUTOUT_ANCHOR_REF,
        OLED_WINDOW_ANCHOR_REF,
        ROTARY_HOLE_ANCHOR_REF,
        *PCB_STANDOFF_REFS,
    ]
    missing = [r for r in required if r not in fps]
    if missing:
        raise RuntimeError(f"Missing required footprints in PCB: {missing}")

    board_w = max_x - min_x
    board_h = max_y - min_y

    outer_x = board_w + 2.0 * (WALL + PCB_CLEARANCE_XY)
    outer_y = board_h + 2.0 * (WALL + PCB_CLEARANCE_XY)
    outer_z = CASE_HEIGHT

    def pcb_to_case(x: float, y: float) -> Tuple[float, float]:
        cx = (x - min_x) + WALL + PCB_CLEARANCE_XY
        cy = (y - min_y) + WALL + PCB_CLEARANCE_XY
        return cx, cy

    # Base closed shell (top plate + walls + bottom).
    outer = box_mesh((0, 0, 0), (outer_x, outer_y, outer_z))
    inner = box_mesh(
        (WALL, WALL, BOTTOM_THICKNESS),
        (outer_x - WALL, outer_y - WALL, outer_z - TOP_THICKNESS),
    )
    shell = boolean_diff(outer, [inner])

    additions: List[trimesh.Trimesh] = []
    cutters: List[trimesh.Trimesh] = []

    # PCB standoffs at equal height; ensure they exceed battery height + clearance.
    required_standoff_height = BATTERY_POCKET_SIZE[2] + BATTERY_TO_PCB_CLEARANCE
    standoff_height = max(PCB_STANDOFF_HEIGHT, required_standoff_height)
    for ref in PCB_STANDOFF_REFS:
        sx, sy = pcb_to_case(fps[ref].x, fps[ref].y)
        height = standoff_height

        additions.append(
            cylinder_z(
                center_xyz=(sx, sy, BOTTOM_THICKNESS + height / 2.0),
                radius=PCB_STANDOFF_DIAMETER / 2.0,
                length=height,
            )
        )
        cutters.append(
            cylinder_z(
                center_xyz=(sx, sy, (BOTTOM_THICKNESS + height) / 2.0),
                radius=PCB_STANDOFF_HOLE_DIAMETER / 2.0,
                length=BOTTOM_THICKNESS + height + 1.0,
            )
        )

    # LiPo battery bay with closed bottom (no underside opening/pod).
    b_anchor = fps[BATTERY_ANCHOR_REF]
    bx, by = pcb_to_case(b_anchor.x + BATTERY_OFFSET_FROM_ANCHOR[0], b_anchor.y + BATTERY_OFFSET_FROM_ANCHOR[1])
    bdx, bdy, bdz = BATTERY_POCKET_SIZE
    board_underside_z = BOTTOM_THICKNESS + standoff_height
    battery_top_z = board_underside_z - BATTERY_TO_PCB_CLEARANCE
    pocket_bottom_z = BOTTOM_THICKNESS + BATTERY_FLOOR_THICKNESS
    if battery_top_z <= pocket_bottom_z:
        raise RuntimeError("Battery bay invalid: increase standoff height or reduce battery size.")

    # Optional low side ring to prevent battery sliding while keeping bottom closed.
    ring_h = min(BATTERY_POD_WALL, max(0.8, bdz * 0.25))
    ring_outer = box_mesh(
        (bx - (bdx + 2.0 * BATTERY_POD_WALL) / 2.0, by - (bdy + 2.0 * BATTERY_POD_WALL) / 2.0, BOTTOM_THICKNESS),
        (bx + (bdx + 2.0 * BATTERY_POD_WALL) / 2.0, by + (bdy + 2.0 * BATTERY_POD_WALL) / 2.0, BOTTOM_THICKNESS + ring_h),
    )
    ring_inner = box_mesh(
        (bx - bdx / 2.0, by - bdy / 2.0, BOTTOM_THICKNESS - 0.2),
        (bx + bdx / 2.0, by + bdy / 2.0, BOTTOM_THICKNESS + ring_h + 0.2),
    )
    additions.append(boolean_diff(ring_outer, [ring_inner]))

    cutters.append(
        box_mesh(
            (bx - bdx / 2.0, by - bdy / 2.0, pocket_bottom_z),
            (bx + bdx / 2.0, by + bdy / 2.0, battery_top_z),
        )
    )

    # Matrix switch holes from SW1..SW26 (general keyboard layout from PCB).
    for idx in range(1, 27):
        ref = f"SW{idx}"
        if ref not in fps:
            continue
        fx_raw, fy = pcb_to_case(fps[ref].x, fps[ref].y)
        fx = outer_x - fx_raw if MIRROR_TOP_FEATURES_X else fx_raw
        sx, sy = SWITCH_HOLE_SIZE
        cutters.append(
            box_mesh(
                (fx - sx / 2.0, fy - sy / 2.0, outer_z - TOP_THICKNESS - 1.0),
                (fx + sx / 2.0, fy + sy / 2.0, outer_z + 1.5),
            )
        )

    # LCD/OLED flush mount: top recess for module face + through view window.
    oled_anchor = fps[OLED_WINDOW_ANCHOR_REF]
    ox_raw, oy = pcb_to_case(
        oled_anchor.x + OLED_OFFSET_FROM_ANCHOR[0],
        oled_anchor.y + OLED_OFFSET_FROM_ANCHOR[1],
    )
    ox = outer_x - ox_raw if MIRROR_TOP_FEATURES_X else ox_raw
    ow, oh = OLED_WINDOW_SIZE
    mw, mh = LCD_MODULE_FACE_SIZE
    pocket_w = mw + 2.0 * LCD_MODULE_FACE_CLEARANCE
    pocket_h = mh + 2.0 * LCD_MODULE_FACE_CLEARANCE

    # Top recess so module face can sit flush with the case top.
    cutters.append(
        box_mesh(
            (ox - pocket_w / 2.0, oy - pocket_h / 2.0, outer_z - LCD_FLUSH_DEPTH - 0.2),
            (ox + pocket_w / 2.0, oy + pocket_h / 2.0, outer_z + 1.0),
        )
    )

    # Visible screen opening through the remaining roof thickness.
    cutters.append(
        box_mesh(
            (ox - ow / 2.0, oy - oh / 2.0, outer_z - TOP_THICKNESS - 1.0),
            (ox + ow / 2.0, oy + oh / 2.0, outer_z + 1.5),
        )
    )
    # Minimal underside relief behind LCD rear port for solder/wire escape.
    exw, exh = LCD_BACK_PORT_ESCAPE_SIZE
    ecy = oy + LCD_BACK_PORT_ESCAPE_Y_OFFSET
    cutters.append(
        box_mesh(
            (ox - exw / 2.0, ecy - exh / 2.0, outer_z - LCD_FLUSH_DEPTH - LCD_BACK_PORT_ESCAPE_DEPTH),
            (ox + exw / 2.0, ecy + exh / 2.0, outer_z - LCD_FLUSH_DEPTH + 0.2),
        )
    )

    # Rotary encoder circle opening.
    enc_anchor = fps[ROTARY_HOLE_ANCHOR_REF]
    ex_raw, ey = pcb_to_case(enc_anchor.x, enc_anchor.y)
    ex = outer_x - ex_raw if MIRROR_TOP_FEATURES_X else ex_raw
    cutters.append(
        cylinder_z(
            center_xyz=(ex, ey, outer_z - TOP_THICKNESS / 2.0),
            radius=ROTARY_HOLE_DIAMETER / 2.0,
            length=TOP_THICKNESS + 3.0,
        )
    )

    # Side switch slot on right wall (MSS-102545-14A-V-SMT): narrow actuator slot + internal body clearance.
    side_anchor = fps[SIDE_SWITCH_ANCHOR_REF]
    side_y_anchor = pcb_to_case(side_anchor.x, side_anchor.y)[1] + SIDE_SWITCH_FROM_ANCHOR_Y
    slot_y, slot_z = SIDE_SWITCH_SLOT
    body_y, body_z = SIDE_SWITCH_BODY_CLEARANCE

    # Visible external actuator slit.
    cutters.append(
        box_mesh(
            (outer_x - WALL - 1.0, side_y_anchor - slot_y / 2.0, SIDE_SWITCH_Z - slot_z / 2.0),
            (outer_x + 2.0, side_y_anchor + slot_y / 2.0, SIDE_SWITCH_Z + slot_z / 2.0),
        )
    )
    # Hidden internal clearance so the slider body/lever does not bind.
    cutters.append(
        box_mesh(
            (outer_x - WALL - 3.2, side_y_anchor - body_y / 2.0, SIDE_SWITCH_Z - body_z / 2.0),
            (outer_x - WALL + 0.6, side_y_anchor + body_y / 2.0, SIDE_SWITCH_Z + body_z / 2.0),
        )
    )

    # Bottom access hole for the tactile reset switch.
    reset_anchor = fps[RESET_BUTTON_ANCHOR_REF]
    rx, ry = pcb_to_case(
        reset_anchor.x + RESET_BUTTON_OFFSET[0],
        reset_anchor.y + RESET_BUTTON_OFFSET[1],
    )

    # Cut all the way from underside up through internal supports to the switch.
    reset_z_min = -1.0
    reset_z_max = board_underside_z + RESET_BUTTON_TUNNEL_TOP_EXTRA
    cutters.append(
        cylinder_z(
            center_xyz=(rx, ry, (reset_z_min + reset_z_max) / 2.0),
            radius=RESET_BUTTON_HOLE_DIAMETER / 2.0,
            length=reset_z_max - reset_z_min,
        )
    )

    # Shallow underside recess to help short pushers reach without protruding.
    cutters.append(
        cylinder_z(
            center_xyz=(rx, ry, RESET_BUTTON_RECESS_DEPTH / 2.0),
            radius=RESET_BUTTON_RECESS_DIAMETER / 2.0,
            length=RESET_BUTTON_RECESS_DEPTH + 0.6,
        )
    )

    # IR holes in front wall from J201/J202.
    # Mirror X because the through-hole IR pair is viewed from the opposite side
    # relative to the case coordinate frame used for front-wall cutouts.
    for ref in ("J201", "J202"):
        fx_raw, _fy = pcb_to_case(fps[ref].x, fps[ref].y)
        fx = outer_x - fx_raw
        cutters.append(
            cylinder_y(
                center_xyz=(fx, WALL / 2.0, IR_HOLE_Z),
                radius=IR_HOLE_DIAMETER / 2.0,
                length=WALL + 3.0,
            )
        )

    # USB-C front cutout: rounded capsule profile to match connector shell.
    usb = fps[USB_CUTOUT_ANCHOR_REF]
    ux, _uy = pcb_to_case(usb.x, usb.y)
    usb_w, usb_h = USB_CUTOUT_SIZE
    r = usb_h / 2.0
    straight = max(0.0, usb_w - 2.0 * r)

    if straight <= 0.05:
        cutters.append(
            cylinder_y(
                center_xyz=(ux, WALL / 2.0, USB_CUTOUT_Z),
                radius=r,
                length=WALL + 3.0,
            )
        )
    else:
        x0 = ux - straight / 2.0
        x1 = ux + straight / 2.0
        cutters.append(
            box_mesh(
                (x0, -1.0, USB_CUTOUT_Z - r),
                (x1, WALL + 2.0, USB_CUTOUT_Z + r),
            )
        )
        cutters.append(cylinder_y(center_xyz=(x0, WALL / 2.0, USB_CUTOUT_Z), radius=r, length=WALL + 3.0))
        cutters.append(cylinder_y(center_xyz=(x1, WALL / 2.0, USB_CUTOUT_Z), radius=r, length=WALL + 3.0))

    # Inner relief pocket behind the outer lip to better clear plug shell.
    in_w, in_h = USB_CUTOUT_INNER_SIZE
    cutters.append(
        box_mesh(
            (ux - in_w / 2.0, WALL - 0.2, USB_CUTOUT_Z - in_h / 2.0),
            (ux + in_w / 2.0, WALL + 2.6, USB_CUTOUT_Z + in_h / 2.0),
        )
    )

    if additions:
        shell = boolean_union([shell, *additions])
    shell = boolean_diff(shell, cutters)

    # Split into separate bottom and top pieces so they can be inspected independently.
    split_lower = box_mesh((-5.0, -5.0, -5.0), (outer_x + 5.0, outer_y + 5.0, SPLIT_Z))
    split_upper = box_mesh((-5.0, -5.0, SPLIT_Z), (outer_x + 5.0, outer_y + 5.0, outer_z + 5.0))
    bottom = boolean_intersection([shell, split_lower])
    top = boolean_intersection([shell, split_upper])

    # Add external underside stand block to create tilt.
    stand_height = math.tan(math.radians(TILT_DEG)) * BOTTOM_STAND_CONTACT_SPAN_Y
    stand = box_mesh(
        (
            BOTTOM_STAND_MARGIN_X + BOTTOM_STAND_X_SHIFT,
            BOTTOM_STAND_FROM_USB_Y,
            -stand_height,
        ),
        (
            outer_x - BOTTOM_STAND_MARGIN_X + BOTTOM_STAND_X_SHIFT,
            BOTTOM_STAND_FROM_USB_Y + BOTTOM_STAND_DEPTH,
            0.0,
        ),
    )
    bottom = boolean_union([bottom, stand])

    # Rebase each part for printability.
    for part in (bottom, top):
        z_min = float(part.bounds[0][2])
        part.apply_translation([0.0, 0.0, -z_min])

    # Add a perimeter lip/groove pair so top and bottom press/snap together.
    if SNAP_LIP_ENABLED:
        bottom_top_z = float(bottom.bounds[1][2])

        lip_outer = box_mesh(
            (SNAP_LIP_INSET, SNAP_LIP_INSET, bottom_top_z),
            (
                outer_x - SNAP_LIP_INSET,
                outer_y - SNAP_LIP_INSET,
                bottom_top_z + SNAP_LIP_HEIGHT,
            ),
        )
        lip_inner = box_mesh(
            (
                SNAP_LIP_INSET + SNAP_LIP_THICKNESS,
                SNAP_LIP_INSET + SNAP_LIP_THICKNESS,
                bottom_top_z - 0.2,
            ),
            (
                outer_x - SNAP_LIP_INSET - SNAP_LIP_THICKNESS,
                outer_y - SNAP_LIP_INSET - SNAP_LIP_THICKNESS,
                bottom_top_z + SNAP_LIP_HEIGHT + 0.2,
            ),
        )
        bottom_lip = boolean_diff(lip_outer, [lip_inner])

        # Break the perimeter lip in 4 places so the seam can flex/release.
        g = SNAP_RELEASE_GAP / 2.0
        release_cutters = [
            # front
            box_mesh(
                (outer_x / 2.0 - g, SNAP_LIP_INSET - 1.0, bottom_top_z - 0.3),
                (outer_x / 2.0 + g, SNAP_LIP_INSET + SNAP_LIP_THICKNESS + 1.0, bottom_top_z + SNAP_LIP_HEIGHT + 0.3),
            ),
            # back
            box_mesh(
                (outer_x / 2.0 - g, outer_y - SNAP_LIP_INSET - SNAP_LIP_THICKNESS - 1.0, bottom_top_z - 0.3),
                (outer_x / 2.0 + g, outer_y - SNAP_LIP_INSET + 1.0, bottom_top_z + SNAP_LIP_HEIGHT + 0.3),
            ),
            # left
            box_mesh(
                (SNAP_LIP_INSET - 1.0, outer_y / 2.0 - g, bottom_top_z - 0.3),
                (SNAP_LIP_INSET + SNAP_LIP_THICKNESS + 1.0, outer_y / 2.0 + g, bottom_top_z + SNAP_LIP_HEIGHT + 0.3),
            ),
            # right
            box_mesh(
                (outer_x - SNAP_LIP_INSET - SNAP_LIP_THICKNESS - 1.0, outer_y / 2.0 - g, bottom_top_z - 0.3),
                (outer_x - SNAP_LIP_INSET + 1.0, outer_y / 2.0 + g, bottom_top_z + SNAP_LIP_HEIGHT + 0.3),
            ),
        ]
        bottom_lip = boolean_diff(bottom_lip, release_cutters)
        bottom = boolean_union([bottom, bottom_lip])

        groove_outer_inset = max(0.05, SNAP_LIP_INSET - SNAP_LIP_CLEARANCE)
        groove_thickness = SNAP_LIP_THICKNESS + 2.0 * SNAP_LIP_CLEARANCE
        groove_depth = SNAP_LIP_HEIGHT + SNAP_LIP_GROOVE_EXTRA_DEPTH

        groove_outer = box_mesh(
            (groove_outer_inset, groove_outer_inset, -0.2),
            (
                outer_x - groove_outer_inset,
                outer_y - groove_outer_inset,
                groove_depth,
            ),
        )
        groove_inner = box_mesh(
            (groove_outer_inset + groove_thickness, groove_outer_inset + groove_thickness, -0.4),
            (
                outer_x - groove_outer_inset - groove_thickness,
                outer_y - groove_outer_inset - groove_thickness,
                groove_depth + 0.2,
            ),
        )
        groove = boolean_diff(groove_outer, [groove_inner])

        # Pry slots on left/right walls at seam level for tool-assisted opening.
        pry_half = PRY_SLOT_WIDTH / 2.0
        pry_cutters = [
            box_mesh(
                (-1.0, outer_y / 2.0 - pry_half, -0.2),
                (WALL + 1.2, outer_y / 2.0 + pry_half, PRY_SLOT_HEIGHT),
            ),
            box_mesh(
                (outer_x - WALL - 1.2, outer_y / 2.0 - pry_half, -0.2),
                (outer_x + 1.0, outer_y / 2.0 + pry_half, PRY_SLOT_HEIGHT),
            ),
        ]

        top = boolean_diff(top, [groove, *pry_cutters])

    OUTPUT_BOTTOM_STL.parent.mkdir(parents=True, exist_ok=True)
    bottom.export(OUTPUT_BOTTOM_STL)
    top.export(OUTPUT_TOP_STL)

    # Side-by-side preview STL.
    top_preview = top.copy()
    top_preview.apply_translation([outer_x + PREVIEW_GAP_X, 0.0, 0.0])
    preview = trimesh.util.concatenate([bottom, top_preview])
    preview.export(OUTPUT_SIDE_BY_SIDE_STL)

    # Fully assembled preview STL (top seated on bottom seam).
    assembled_top = top.copy()
    assembled_top.apply_translation([0.0, 0.0, float(bottom.bounds[1][2])])
    assembled = trimesh.util.concatenate([bottom, assembled_top])
    assembled.export(OUTPUT_ASSEMBLED_STL)

    bb = bottom.bounds
    tb = top.bounds
    print(f"Wrote: {OUTPUT_BOTTOM_STL}")
    print(f"Wrote: {OUTPUT_TOP_STL}")
    print(f"Wrote: {OUTPUT_SIDE_BY_SIDE_STL}")
    print(f"Wrote: {OUTPUT_ASSEMBLED_STL}")
    print(f"Bottom bounds mm: x={bb[1][0]-bb[0][0]:.2f}, y={bb[1][1]-bb[0][1]:.2f}, z={bb[1][2]-bb[0][2]:.2f}")
    print(f"Top bounds mm: x={tb[1][0]-tb[0][0]:.2f}, y={tb[1][1]-tb[0][1]:.2f}, z={tb[1][2]-tb[0][2]:.2f}")


if __name__ == "__main__":
    main()
