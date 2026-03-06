#!/usr/bin/env python3
"""
Parametric STL generator for the Macropad enclosure.

Source-of-truth inputs (per user request):
- Board outline from Gerber: Macropad/gerber/Macropad-Edge_Cuts.gbr
- Drill locations from Excellon: Macropad/gerber/Macropad-NPTH.drl
- Component anchors from CPL: Macropad/production/positions.csv
- (Optional) BOM sanity checks: Macropad/production/bom.csv

No "feature mirroring" is applied. Coordinates are used in the native KiCad /
Gerber / CPL frame: +X to the right, +Y upward (matches your Macropad.jpg).
"""

from __future__ import annotations

import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np
import trimesh


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
OUTPUT_DIR = PROJECT_ROOT / "case_reworking"

EDGE_GERBER_PATH = PROJECT_ROOT / "Macropad" / "gerber" / "Macropad-Edge_Cuts.gbr"
NPTH_DRILL_PATH = PROJECT_ROOT / "Macropad" / "gerber" / "Macropad-NPTH.drl"
CPL_PATH = PROJECT_ROOT / "Macropad" / "production" / "positions.csv"
BOM_PATH = PROJECT_ROOT / "Macropad" / "production" / "bom.csv"

OUTPUT_BOTTOM_STL = OUTPUT_DIR / "macropad_case_bottom.stl"
OUTPUT_TOP_STL = OUTPUT_DIR / "macropad_case_top.stl"
OUTPUT_SIDE_BY_SIDE_STL = OUTPUT_DIR / "macropad_case_side_by_side.stl"
OUTPUT_ASSEMBLED_STL = OUTPUT_DIR / "macropad_case_assembled.stl"
OUTPUT_PCB_PREVIEW_STL = OUTPUT_DIR / "macropad_case_pcb_preview.stl"
OUTPUT_OLED_RETAINER_STL = OUTPUT_DIR / "macropad_oled_retainer.stl"


# -------- User-editable parameters --------
WALL = 2.4
TOP_THICKNESS = 2.2
BOTTOM_THICKNESS = 2.4
PCB_CLEARANCE_XY = 1.0
PCB_THICKNESS = 1.6
CASE_HEIGHT = 26.0

# Add extra length only on the *bottom* edge of the PCB (negative-Y side in KiCad coords).
# Implementation detail: we shift the PCB upward inside the case by this amount.
CASE_EXTRA_Y_BOTTOM = 12.7  # 0.5 in

# 4 degree typing/elevation angle.
TILT_DEG = 4.0

# MX/Kailh-compatible top cutouts (plate-style opening).
SWITCH_HOLE_SIZE = (14.1, 14.1)

# Switch centers are derived from NPTH 4.0 mm drill hits (MX center post).
SWITCH_CENTER_DRILL_DIAMETER = 4.0
SWITCH_CENTER_DRILL_TOL = 0.02

# PCB mounting standoffs are derived from NPTH 3.2 mm drill hits.
MOUNT_HOLE_DRILL_DIAMETER = 3.2
MOUNT_HOLE_DRILL_TOL = 0.02
PCB_STANDOFF_DIAMETER = 7.0
PCB_STANDOFF_HOLE_DIAMETER = 3.4
PCB_STANDOFF_HEIGHT = 2.5

# Display / "OLED" window (big rectangle in screenshot).
# Anchored to the midpoint of J302/J303 (LCD headers) from CPL, then offset.
DISPLAY_ANCHOR_REFS = ("J302", "J303")
DISPLAY_CENTER_OFFSET = (31.0, 0.0)  # (x, y) mm in case coords from anchor midpoint
DISPLAY_WINDOW_SIZE = (70.0, 46.0)  # (x span, y span) visible window cutout
DISPLAY_FRONT_LIP_THICKNESS = 0.90  # leaves a bezel lip thickness at the top surface

# Optional backside locating pocket for the display PCB.
DISPLAY_POCKET_ENABLED = True
DISPLAY_PCB_OUTLINE_SIZE = (81.3, 62.5)  # (x, y)
DISPLAY_PCB_OUTLINE_CLEARANCE = 0.30
DISPLAY_PCB_POCKET_DEPTH = 4.60

# Wire relief for the display (jumper/Dupont wires). Creates an internal notch from the
# display pocket into the case interior, hidden under the front lip.
DISPLAY_WIRE_NOTCH_ENABLED = True
DISPLAY_WIRE_NOTCH_SIDE = "right"  # "left" or "right"
DISPLAY_WIRE_NOTCH_WIDTH_Y = 12.0
DISPLAY_WIRE_NOTCH_DEPTH_X = 7.0

# Simple underside mounting for the OLED module: add screw bosses in the top piece and
# generate a matching retainer plate STL you can screw in to clamp the module.
OLED_MOUNT_ENABLED = True
OLED_MOUNT_BOSS_DIAMETER = 6.0
OLED_MOUNT_BOSS_HEIGHT = 4.6
OLED_MOUNT_PILOT_DIAMETER = 2.0  # self-tap pilot; adjust for your screws
OLED_MOUNT_PILOT_CAP = 0.9  # keep a solid cap at the top to avoid breaking through the face
# Boss locations are offsets (dx, dy) from display center in *case coords*.
# Default places 2 bosses on the left (above switches) + 2 bosses above the window.
OLED_MOUNT_BOSS_OFFSETS = [(-45.0, -6.0), (-45.0, 10.0), (-25.0, 29.0), (25.0, 29.0)]

OLED_RETAINER_ENABLED = True
OLED_RETAINER_THICKNESS = 2.4
OLED_RETAINER_HOLE_DIAMETER = 3.2  # clearance for M3 (use heat-set/self-tap in bosses)
OLED_RETAINER_WINDOW_MARGIN = 3.0  # extra opening beyond DISPLAY_WINDOW_SIZE
OLED_RETAINER_WIRE_NOTCH = True
OLED_RETAINER_WIRE_NOTCH_WIDTH_Y = 14.0
OLED_RETAINER_WIRE_NOTCH_DEPTH_X = 10.0

# Rotary encoder top circle opening.
ROTARY_HOLE_ANCHOR_REF = "SW401"
ROTARY_HOLE_DIAMETER = 8.0
# Some footprints' "mid" point isn't exactly the shaft axis. Keep this as an explicit
# tweak (in PCB/CPL coords, mm). Default is no offset.
ROTARY_HOLE_OFFSET = (0.0, 0.0)  # (dx, dy)

# Side on/off slide switch slot (MSS-102545-14A-V-SMT) on right wall.
SIDE_SWITCH_ENABLED = True
SIDE_SWITCH_ANCHOR_REF = "J302"
# Keep the default slot slightly "down" from J302 (toward the keys): negative in +Y-up coords.
SIDE_SWITCH_FROM_ANCHOR_Y = -5.0
SIDE_SWITCH_SLOT = (4.6, 1.8)  # visible actuator slot: (y span, z span)
SIDE_SWITCH_Z = CASE_HEIGHT * 0.50
SIDE_SWITCH_BODY_CLEARANCE = (7.2, 2.8)  # hidden internal clearance: (y span, z span)

# Bottom reset-button access hole aligned to SW402 tactile.
RESET_BUTTON_ANCHOR_REF = "SW402"
RESET_BUTTON_HOLE_DIAMETER = 4.4
RESET_BUTTON_OFFSET = (0.0, 0.0)
RESET_BUTTON_TUNNEL_TOP_EXTRA = 1.2
RESET_BUTTON_RECESS_DIAMETER = 7.0
RESET_BUTTON_RECESS_DEPTH = 1.4

# IR holes (from J201/J202 positions on PCB, front wall).
IR_HOLE_REFS = ("J201", "J202")
IR_HOLE_DIAMETER = 3.3
IR_HOLE_Z = CASE_HEIGHT * 0.50

# USB-C front cutout (from J402).
USB_CUTOUT_ANCHOR_REF = "J402"
USB_CUTOUT_SIZE = (9.2, 3.2)  # (x span, z span)
USB_CUTOUT_Z = BOTTOM_THICKNESS + 6.0
USB_CUTOUT_INNER_SIZE = (8.6, 2.8)  # inner relief behind outer lip: (x span, z span)

# Split and preview spacing.
SPLIT_Z = CASE_HEIGHT * 0.72
PREVIEW_GAP_X = 14.0

# Battery holder pod on the underside (for the LiPo in macropad.ods, Adafruit 328).
# This is a protective "cup" that protrudes downward from the bottom.
BATTERY_POD_ENABLED = True
BATTERY_SIZE = (65.0, 51.0, 8.0)  # (x, y, z) mm; adjust if your pack differs
BATTERY_CLEARANCE_XY = 0.6
BATTERY_CLEARANCE_Z = 0.8
BATTERY_POD_WALL = 1.8
BATTERY_POD_FLOOR = 1.6
# Additional placement tweak in case coordinates.
BATTERY_CENTER_OFFSET = (0.0, 0.0)  # (x, y) mm in case coords, after auto placement


@dataclass(frozen=True)
class Placement:
    ref: str
    x: float
    y: float
    rot: float
    layer: str


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
    cutter_union = trimesh.boolean.union(cutters, engine="manifold")
    return trimesh.boolean.difference([base, cutter_union], engine="manifold")


def boolean_intersection(meshes: List[trimesh.Trimesh]) -> trimesh.Trimesh:
    return trimesh.boolean.intersection(meshes, engine="manifold")


def boolean_union(meshes: List[trimesh.Trimesh]) -> trimesh.Trimesh:
    return trimesh.boolean.union(meshes, engine="manifold")


def _parse_gerber_format(lines: List[str]) -> Tuple[int, int, bool]:
    # Example: %FSLAX46Y46*% then %MOMM*%
    fmt_re = re.compile(r"%FSLAX(\d)(\d)Y(\d)(\d)\*%")
    int_digits = None
    dec_digits = None
    unit_mm = False
    unit_in = False
    for line in lines:
        if "%MOMM*%" in line:
            unit_mm = True
        if "%MOIN*%" in line:
            unit_in = True
        m = fmt_re.match(line.strip())
        if m:
            int_digits = int(m.group(1))
            dec_digits = int(m.group(2))
            if (int_digits != int(m.group(3))) or (dec_digits != int(m.group(4))):
                raise RuntimeError("Gerber X/Y formats differ; unsupported.")
    if int_digits is None or dec_digits is None:
        raise RuntimeError("Could not find Gerber %FSLAX..Y..*% format line.")
    if unit_in and unit_mm:
        raise RuntimeError("Gerber declares both inch and mm units; unsupported.")
    if not unit_in and not unit_mm:
        raise RuntimeError("Gerber missing units (%MOMM*% or %MOIN*%).")
    return int_digits, dec_digits, unit_mm


def parse_board_bbox_from_edge_gerber(path: Path) -> Tuple[float, float, float, float]:
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    _int_digits, dec_digits, unit_mm = _parse_gerber_format(lines)
    scale = 10 ** (-dec_digits)
    xy_re = re.compile(r"X(-?\d+)Y(-?\d+)")
    xs: List[float] = []
    ys: List[float] = []
    for line in lines:
        m = xy_re.search(line)
        if not m:
            continue
        x = int(m.group(1)) * scale
        y = int(m.group(2)) * scale
        if not unit_mm:
            x *= 25.4
            y *= 25.4
        xs.append(x)
        ys.append(y)
    if not xs:
        raise RuntimeError(f"No coordinates found in edge gerber: {path}")
    min_x_g, max_x_g = min(xs), max(xs)
    min_y_g, max_y_g = min(ys), max(ys)
    # Keep native KiCad/Gerber coords: +Y upward.
    return min_x_g, min_y_g, max_x_g, max_y_g


def parse_excellon_holes(path: Path) -> List[Tuple[float, float, float]]:
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    tool_diams: Dict[int, float] = {}
    current_tool: int | None = None
    holes: List[Tuple[float, float, float]] = []

    tool_re = re.compile(r"^T(\d+)C([0-9.]+)")
    select_re = re.compile(r"^T(\d+)$")
    xy_re = re.compile(r"^X(-?[0-9.]+)Y(-?[0-9.]+)$")

    for raw in lines:
        line = raw.strip()
        if not line or line.startswith(";"):
            continue
        m = tool_re.match(line)
        if m:
            tool_diams[int(m.group(1))] = float(m.group(2))
            continue
        m = select_re.match(line)
        if m:
            current_tool = int(m.group(1))
            continue
        m = xy_re.match(line)
        if m and current_tool in tool_diams:
            x_g = float(m.group(1))
            y_g = float(m.group(2))
            # Keep native KiCad coords: +Y upward.
            holes.append((tool_diams[current_tool], x_g, y_g))
    return holes


def parse_cpl_positions(path: Path) -> Dict[str, Placement]:
    placements: Dict[str, Placement] = {}
    with path.open(encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ref = (row.get("Designator") or "").strip()
            if not ref:
                continue
            try:
                x_g = float(row.get("Mid X") or "nan")
                y_g = float(row.get("Mid Y") or "nan")
            except ValueError:
                continue
            rot = float(row.get("Rotation") or 0.0)
            layer = (row.get("Layer") or "").strip().lower()
            # Keep native KiCad coords: +Y upward.
            placements[ref] = Placement(ref=ref, x=x_g, y=y_g, rot=rot, layer=layer)
    return placements


def parse_bom(path: Path) -> Dict[str, Dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    out: Dict[str, Dict[str, str]] = {}
    for r in rows:
        ref = (r.get("Designator") or "").strip()
        if ref:
            out[ref] = r
    return out


def pick_holes(holes: List[Tuple[float, float, float]], diameter: float, tol: float) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    for d, x, y in holes:
        if abs(d - diameter) <= tol:
            pts.append((x, y))
    return pts


def main() -> None:
    for p in (EDGE_GERBER_PATH, NPTH_DRILL_PATH, CPL_PATH):
        if not p.exists():
            raise RuntimeError(f"Missing required input file: {p}")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    min_x, min_y, max_x, max_y = parse_board_bbox_from_edge_gerber(EDGE_GERBER_PATH)
    board_w = max_x - min_x
    board_h = max_y - min_y

    outer_x = board_w + 2.0 * (WALL + PCB_CLEARANCE_XY)
    outer_y = board_h + 2.0 * (WALL + PCB_CLEARANCE_XY) + CASE_EXTRA_Y_BOTTOM
    outer_z = CASE_HEIGHT

    def pcb_to_case(x: float, y: float) -> Tuple[float, float]:
        cx = (x - min_x) + WALL + PCB_CLEARANCE_XY
        # Shift PCB upward inside the case to create extra room at the bottom edge.
        cy = (y - min_y) + WALL + PCB_CLEARANCE_XY + CASE_EXTRA_Y_BOTTOM
        return cx, cy

    holes_np = parse_excellon_holes(NPTH_DRILL_PATH)
    switch_centers = pick_holes(holes_np, SWITCH_CENTER_DRILL_DIAMETER, SWITCH_CENTER_DRILL_TOL)
    mount_holes = pick_holes(holes_np, MOUNT_HOLE_DRILL_DIAMETER, MOUNT_HOLE_DRILL_TOL)
    if len(switch_centers) < 10:
        raise RuntimeError(f"Too few switch centers found in NPTH drills: {len(switch_centers)}")
    if len(mount_holes) < 4:
        raise RuntimeError(f"Too few mount holes found in NPTH drills: {len(mount_holes)}")

    cpl = parse_cpl_positions(CPL_PATH)
    # Optional sanity checks on key refs.
    required_refs = [
        ROTARY_HOLE_ANCHOR_REF,
        RESET_BUTTON_ANCHOR_REF,
        USB_CUTOUT_ANCHOR_REF,
        *IR_HOLE_REFS,
        *DISPLAY_ANCHOR_REFS,
        SIDE_SWITCH_ANCHOR_REF,
    ]
    missing = [r for r in required_refs if r and r not in cpl]
    if missing:
        raise RuntimeError(f"Missing required designators in CPL: {missing}")
    if BOM_PATH.exists():
        _bom = parse_bom(BOM_PATH)

    # Base closed shell (top plate + walls + bottom).
    outer = box_mesh((0, 0, 0), (outer_x, outer_y, outer_z))
    inner = box_mesh(
        (WALL, WALL, BOTTOM_THICKNESS),
        (outer_x - WALL, outer_y - WALL, outer_z - TOP_THICKNESS),
    )
    shell = boolean_diff(outer, [inner])

    additions: List[trimesh.Trimesh] = []
    cutters: List[trimesh.Trimesh] = []
    post_cut_additions: List[trimesh.Trimesh] = []
    post_cut_cutters: List[trimesh.Trimesh] = []

    # Saved for optional OLED retainer output.
    oled_boss_points_xy: List[Tuple[float, float]] = []
    oled_boss_top_z: float | None = None
    oled_boss_bottom_z: float | None = None
    disp_pocket_w: float | None = None
    disp_pocket_h: float | None = None

    # PCB standoffs from mount holes.
    board_underside_z = BOTTOM_THICKNESS + PCB_STANDOFF_HEIGHT
    for (hx, hy) in mount_holes:
        sx, sy = pcb_to_case(hx, hy)
        additions.append(
            cylinder_z(
                center_xyz=(sx, sy, BOTTOM_THICKNESS + PCB_STANDOFF_HEIGHT / 2.0),
                radius=PCB_STANDOFF_DIAMETER / 2.0,
                length=PCB_STANDOFF_HEIGHT,
            )
        )
        cutters.append(
            cylinder_z(
                center_xyz=(sx, sy, (BOTTOM_THICKNESS + PCB_STANDOFF_HEIGHT) / 2.0),
                radius=PCB_STANDOFF_HOLE_DIAMETER / 2.0,
                length=BOTTOM_THICKNESS + PCB_STANDOFF_HEIGHT + 1.0,
            )
        )

    # Switch cutouts: plate-style openings in top plate.
    sx, sy = SWITCH_HOLE_SIZE
    for (kx, ky) in switch_centers:
        cx, cy = pcb_to_case(kx, ky)
        cutters.append(
            box_mesh(
                (cx - sx / 2.0, cy - sy / 2.0, outer_z - TOP_THICKNESS - 1.0),
                (cx + sx / 2.0, cy + sy / 2.0, outer_z + 1.5),
            )
        )

    # Display window + (optional) backside pocket.
    anchor_pts = [pcb_to_case(cpl[r].x, cpl[r].y) for r in DISPLAY_ANCHOR_REFS]
    anchor_cx = sum(p[0] for p in anchor_pts) / len(anchor_pts)
    anchor_cy = sum(p[1] for p in anchor_pts) / len(anchor_pts)
    disp_cx = anchor_cx + DISPLAY_CENTER_OFFSET[0]
    disp_cy = anchor_cy + DISPLAY_CENTER_OFFSET[1]
    win_w, win_h = DISPLAY_WINDOW_SIZE
    lip_bottom_z = outer_z - DISPLAY_FRONT_LIP_THICKNESS
    top_only_cut_z_min = SPLIT_Z + 0.05

    # True through-window.
    cutters.append(
        box_mesh(
            (disp_cx - win_w / 2.0, disp_cy - win_h / 2.0, top_only_cut_z_min),
            (disp_cx + win_w / 2.0, disp_cy + win_h / 2.0, outer_z + 1.5),
        )
    )
    pocket_top_z = outer_z - TOP_THICKNESS  # default roof underside in non-pocket areas
    if DISPLAY_POCKET_ENABLED:
        pocket_w = DISPLAY_PCB_OUTLINE_SIZE[0] + 2.0 * DISPLAY_PCB_OUTLINE_CLEARANCE
        pocket_h = DISPLAY_PCB_OUTLINE_SIZE[1] + 2.0 * DISPLAY_PCB_OUTLINE_CLEARANCE
        pocket_top_z = outer_z - DISPLAY_FRONT_LIP_THICKNESS - 0.15
        pocket_bottom_z = pocket_top_z - DISPLAY_PCB_POCKET_DEPTH
        disp_pocket_w = pocket_w
        disp_pocket_h = pocket_h
        cutters.append(
            box_mesh(
                (disp_cx - pocket_w / 2.0, disp_cy - pocket_h / 2.0, pocket_bottom_z),
                (disp_cx + pocket_w / 2.0, disp_cy + pocket_h / 2.0, pocket_top_z),
            )
        )
        # Rear relief up to lip plane to preserve a clean front bezel.
        cutters.append(
            box_mesh(
                (disp_cx - (win_w + 1.2) / 2.0, disp_cy - (win_h + 1.2) / 2.0, top_only_cut_z_min),
                (disp_cx + (win_w + 1.2) / 2.0, disp_cy + (win_h + 1.2) / 2.0, lip_bottom_z),
            )
        )
        if DISPLAY_WIRE_NOTCH_ENABLED:
            notch_z0 = top_only_cut_z_min
            notch_z1 = lip_bottom_z
            notch_y0 = disp_cy - DISPLAY_WIRE_NOTCH_WIDTH_Y / 2.0
            notch_y1 = disp_cy + DISPLAY_WIRE_NOTCH_WIDTH_Y / 2.0
            left_x = disp_cx - pocket_w / 2.0
            right_x = disp_cx + pocket_w / 2.0
            if DISPLAY_WIRE_NOTCH_SIDE.strip().lower() == "right":
                cutters.append(
                    box_mesh(
                        (right_x - 0.2, notch_y0, notch_z0),
                        (right_x + DISPLAY_WIRE_NOTCH_DEPTH_X, notch_y1, notch_z1),
                    )
                )
            else:
                cutters.append(
                    box_mesh(
                        (left_x - DISPLAY_WIRE_NOTCH_DEPTH_X, notch_y0, notch_z0),
                        (left_x + 0.2, notch_y1, notch_z1),
                    )
                )

    # OLED mounting bosses (underside of top) to clamp module with a retainer plate.
    if OLED_MOUNT_ENABLED:
        oled_boss_top_z = pocket_top_z
        oled_boss_bottom_z = oled_boss_top_z - OLED_MOUNT_BOSS_HEIGHT
        boss_center_z = (oled_boss_top_z + oled_boss_bottom_z) / 2.0
        for (dx, dy) in OLED_MOUNT_BOSS_OFFSETS:
            bx = disp_cx + float(dx)
            by = disp_cy + float(dy)
            # Basic sanity: keep boss within the case envelope.
            if not (WALL + 0.8 <= bx <= outer_x - WALL - 0.8 and WALL + 0.8 <= by <= outer_y - WALL - 0.8):
                raise RuntimeError(
                    f"OLED boss at ({bx:.2f},{by:.2f}) is outside inner walls. Adjust OLED_MOUNT_BOSS_OFFSETS."
                )
            oled_boss_points_xy.append((bx, by))
            post_cut_additions.append(
                cylinder_z(
                    center_xyz=(bx, by, boss_center_z),
                    radius=OLED_MOUNT_BOSS_DIAMETER / 2.0,
                    length=OLED_MOUNT_BOSS_HEIGHT,
                )
            )
            # Blind pilot hole (from bottom of boss upward).
            z0 = oled_boss_bottom_z - 0.6
            z1 = oled_boss_top_z - OLED_MOUNT_PILOT_CAP
            if z1 <= z0 + 0.2:
                raise RuntimeError("OLED pilot hole depth invalid; check OLED_MOUNT_PILOT_CAP / OLED_MOUNT_BOSS_HEIGHT.")
            post_cut_cutters.append(
                cylinder_z(
                    center_xyz=(bx, by, (z0 + z1) / 2.0),
                    radius=OLED_MOUNT_PILOT_DIAMETER / 2.0,
                    length=(z1 - z0),
                )
            )

    # Rotary encoder circle opening.
    enc = cpl[ROTARY_HOLE_ANCHOR_REF]
    ex, ey = pcb_to_case(enc.x + ROTARY_HOLE_OFFSET[0], enc.y + ROTARY_HOLE_OFFSET[1])
    cutters.append(
        cylinder_z(
            center_xyz=(ex, ey, outer_z - TOP_THICKNESS / 2.0),
            radius=ROTARY_HOLE_DIAMETER / 2.0,
            length=TOP_THICKNESS + 3.0,
        )
    )

    # Side switch slot on right wall.
    if SIDE_SWITCH_ENABLED:
        side_anchor = cpl[SIDE_SWITCH_ANCHOR_REF]
        side_y_anchor = pcb_to_case(side_anchor.x, side_anchor.y)[1] + SIDE_SWITCH_FROM_ANCHOR_Y
        slot_y, slot_z = SIDE_SWITCH_SLOT
        body_y, body_z = SIDE_SWITCH_BODY_CLEARANCE
        cutters.append(
            box_mesh(
                (outer_x - WALL - 1.0, side_y_anchor - slot_y / 2.0, SIDE_SWITCH_Z - slot_z / 2.0),
                (outer_x + 2.0, side_y_anchor + slot_y / 2.0, SIDE_SWITCH_Z + slot_z / 2.0),
            )
        )
        cutters.append(
            box_mesh(
                (outer_x - WALL - 3.2, side_y_anchor - body_y / 2.0, SIDE_SWITCH_Z - body_z / 2.0),
                (outer_x - WALL + 0.6, side_y_anchor + body_y / 2.0, SIDE_SWITCH_Z + body_z / 2.0),
            )
        )

    # Bottom access hole for tactile reset switch.
    reset = cpl[RESET_BUTTON_ANCHOR_REF]
    rx, ry = pcb_to_case(reset.x + RESET_BUTTON_OFFSET[0], reset.y + RESET_BUTTON_OFFSET[1])
    reset_z_min = -1.0
    reset_z_max = board_underside_z + PCB_THICKNESS + RESET_BUTTON_TUNNEL_TOP_EXTRA
    cutters.append(
        cylinder_z(
            center_xyz=(rx, ry, (reset_z_min + reset_z_max) / 2.0),
            radius=RESET_BUTTON_HOLE_DIAMETER / 2.0,
            length=reset_z_max - reset_z_min,
        )
    )
    cutters.append(
        cylinder_z(
            center_xyz=(rx, ry, RESET_BUTTON_RECESS_DEPTH / 2.0),
            radius=RESET_BUTTON_RECESS_DIAMETER / 2.0,
            length=RESET_BUTTON_RECESS_DEPTH + 0.6,
        )
    )

    front_wall_y = outer_y - WALL / 2.0

    # IR holes on the front wall (top edge): drill through +Y wall.
    for ref in IR_HOLE_REFS:
        ir = cpl[ref]
        ix, _iy = pcb_to_case(ir.x, ir.y)
        cutters.append(
            cylinder_y(
                center_xyz=(ix, front_wall_y, IR_HOLE_Z),
                radius=IR_HOLE_DIAMETER / 2.0,
                length=WALL + 3.0,
            )
        )

    # USB-C front cutout: capsule profile.
    usb = cpl[USB_CUTOUT_ANCHOR_REF]
    ux, _uy = pcb_to_case(usb.x, usb.y)
    usb_w, usb_h = USB_CUTOUT_SIZE
    r = usb_h / 2.0
    straight = max(0.0, usb_w - 2.0 * r)
    if straight <= 0.05:
        cutters.append(
            cylinder_y(
                center_xyz=(ux, front_wall_y, USB_CUTOUT_Z),
                radius=r,
                length=WALL + 3.0,
            )
        )
    else:
        x0 = ux - straight / 2.0
        x1 = ux + straight / 2.0
        cutters.append(
            box_mesh(
                (x0, outer_y - WALL - 2.0, USB_CUTOUT_Z - r),
                (x1, outer_y + 1.0, USB_CUTOUT_Z + r),
            )
        )
        cutters.append(cylinder_y(center_xyz=(x0, front_wall_y, USB_CUTOUT_Z), radius=r, length=WALL + 3.0))
        cutters.append(cylinder_y(center_xyz=(x1, front_wall_y, USB_CUTOUT_Z), radius=r, length=WALL + 3.0))
    # Inner relief behind outer lip.
    in_w, in_h = USB_CUTOUT_INNER_SIZE
    cutters.append(
        box_mesh(
            (ux - in_w / 2.0, outer_y - WALL - 2.6, USB_CUTOUT_Z - in_h / 2.0),
            (ux + in_w / 2.0, outer_y - WALL + 0.2, USB_CUTOUT_Z + in_h / 2.0),
        )
    )

    if additions:
        shell = boolean_union([shell, *additions])
    shell = boolean_diff(shell, cutters)
    if post_cut_additions:
        shell = boolean_union([shell, *post_cut_additions])
    if post_cut_cutters:
        shell = boolean_diff(shell, post_cut_cutters)

    # Split into separate bottom and top pieces.
    split_lower = box_mesh((-5.0, -5.0, -5.0), (outer_x + 5.0, outer_y + 5.0, SPLIT_Z))
    split_upper = box_mesh((-5.0, -5.0, SPLIT_Z), (outer_x + 5.0, outer_y + 5.0, outer_z + 5.0))
    bottom = boolean_intersection([shell, split_lower])
    top = boolean_intersection([shell, split_upper])

    # Battery pod (underside).
    battery_outer_min_x = battery_outer_min_y = battery_outer_max_x = battery_outer_max_y = None
    if BATTERY_POD_ENABLED:
        # Auto-place under the switch matrix (keeps it away from the top electronics area like SW402).
        switch_case_pts = [pcb_to_case(kx, ky) for (kx, ky) in switch_centers]
        sw_x0 = min(p[0] for p in switch_case_pts)
        sw_x1 = max(p[0] for p in switch_case_pts)
        sw_y0 = min(p[1] for p in switch_case_pts)
        sw_y1 = max(p[1] for p in switch_case_pts)
        sw_cx = (sw_x0 + sw_x1) / 2.0
        sw_cy = (sw_y0 + sw_y1) / 2.0

        bat_inner_x = BATTERY_SIZE[0] + 2.0 * BATTERY_CLEARANCE_XY
        bat_inner_y = BATTERY_SIZE[1] + 2.0 * BATTERY_CLEARANCE_XY
        bat_outer_x = bat_inner_x + 2.0 * BATTERY_POD_WALL
        bat_outer_y = bat_inner_y + 2.0 * BATTERY_POD_WALL
        bat_depth = BATTERY_SIZE[2] + BATTERY_CLEARANCE_Z + BATTERY_POD_FLOOR

        bat_cx = sw_cx + BATTERY_CENTER_OFFSET[0]
        bat_cy = sw_cy + BATTERY_CENTER_OFFSET[1]

        # Clamp within outer footprint.
        bat_cx = max(bat_outer_x / 2.0, min(outer_x - bat_outer_x / 2.0, bat_cx))
        bat_cy = max(bat_outer_y / 2.0, min(outer_y - bat_outer_y / 2.0, bat_cy))

        x0 = bat_cx - bat_outer_x / 2.0
        x1 = bat_cx + bat_outer_x / 2.0
        y0 = bat_cy - bat_outer_y / 2.0
        y1 = bat_cy + bat_outer_y / 2.0
        battery_outer_min_x, battery_outer_min_y, battery_outer_max_x, battery_outer_max_y = x0, y0, x1, y1

        pod_outer = box_mesh((x0, y0, -bat_depth), (x1, y1, 0.0))
        pod_inner = box_mesh(
            (x0 + BATTERY_POD_WALL, y0 + BATTERY_POD_WALL, -bat_depth + BATTERY_POD_FLOOR),
            (x1 - BATTERY_POD_WALL, y1 - BATTERY_POD_WALL, 1.0),  # open top
        )
        pod_shell = boolean_diff(pod_outer, [pod_inner])
        bottom = boolean_union([bottom, pod_shell])

        # Create opening through the main bottom plate into the pod (keeps the pod floor intact).
        open_x0 = x0 + BATTERY_POD_WALL
        open_x1 = x1 - BATTERY_POD_WALL
        open_y0 = y0 + BATTERY_POD_WALL
        open_y1 = y1 - BATTERY_POD_WALL
        opening = box_mesh((open_x0, open_y0, -0.2), (open_x1, open_y1, BOTTOM_THICKNESS + 0.4))
        bottom = boolean_diff(bottom, [opening])

    # Add external underside stand block for tilt (near the top edge).
    stand_height = math.tan(math.radians(TILT_DEG)) * 80.0
    stand_y0 = outer_y - 26.0
    stand_y1 = outer_y - 8.0
    stand = box_mesh((12.0, stand_y0, -stand_height), (outer_x - 12.0, stand_y1, 0.0))
    if BATTERY_POD_ENABLED and battery_outer_min_x is not None:
        # Avoid filling the battery pod region with the stand (keep a small margin).
        mx = 1.2
        stand_cut_y0 = max(stand_y0, float(battery_outer_min_y) - mx)
        stand_cut_y1 = min(stand_y1, float(battery_outer_max_y) + mx)
        if stand_cut_y1 > stand_cut_y0:
            stand_notch = box_mesh(
                (float(battery_outer_min_x) - mx, stand_cut_y0, -stand_height - 1.0),
                (float(battery_outer_max_x) + mx, stand_cut_y1, 1.0),
            )
            stand = boolean_diff(stand, [stand_notch])
    bottom = boolean_union([bottom, stand])

    # Ensure the reset access hole is visible/reachable from the lowest exterior surface by
    # adding a small tunnel/boss down to the bottom-most Z of the bottom part.
    #
    # Without this, the battery pod sets the global Z-min and the reset hole can end up on
    # a "raised" plane that is easy to miss when looking at the underside.
    min_z_before_rebase = float(bottom.bounds[0][2])
    if min_z_before_rebase < -0.05:
        boss_len = -min_z_before_rebase
        boss_cz = min_z_before_rebase / 2.0
        reset_boss_outer = cylinder_z(
            center_xyz=(rx, ry, boss_cz),
            radius=RESET_BUTTON_RECESS_DIAMETER / 2.0,
            length=boss_len,
        )
        bottom = boolean_union([bottom, reset_boss_outer])
        reset_boss_hole = cylinder_z(
            center_xyz=(rx, ry, boss_cz),
            radius=RESET_BUTTON_HOLE_DIAMETER / 2.0,
            length=boss_len + 2.0,
        )
        bottom = boolean_diff(bottom, [reset_boss_hole])

    # Rebase each part so Z min is 0.
    bottom_rebase_shift = -float(bottom.bounds[0][2])
    top_rebase_shift = -float(top.bounds[0][2])
    bottom.apply_translation([0.0, 0.0, bottom_rebase_shift])
    top.apply_translation([0.0, 0.0, top_rebase_shift])

    # Export parts.
    bottom.export(OUTPUT_BOTTOM_STL)
    top.export(OUTPUT_TOP_STL)

    # Assembled + side-by-side previews.
    top_preview = top.copy()
    top_preview.apply_translation([outer_x + PREVIEW_GAP_X, 0.0, 0.0])
    trimesh.util.concatenate([bottom.copy(), top_preview]).export(OUTPUT_SIDE_BY_SIDE_STL)

    assembled_top = top.copy()
    assembled_top.apply_translation([0.0, 0.0, float(bottom.bounds[1][2])])
    assembled = trimesh.util.concatenate([bottom.copy(), assembled_top])
    assembled.export(OUTPUT_ASSEMBLED_STL)

    # Optional: OLED retainer plate (separate part) that screws into the bosses and
    # clamps the OLED module in the pocket. Exported in the *assembled* coordinate frame.
    if OLED_MOUNT_ENABLED and OLED_RETAINER_ENABLED and oled_boss_points_xy and oled_boss_bottom_z is not None:
        bottom_height = float(bottom.bounds[1][2])
        win_margin = float(OLED_RETAINER_WINDOW_MARGIN)

        # Window opening on retainer (larger than the visible window to avoid interference).
        wx0 = disp_cx - (win_w / 2.0 + win_margin)
        wx1 = disp_cx + (win_w / 2.0 + win_margin)
        wy0 = disp_cy - (win_h / 2.0 + win_margin)
        wy1 = disp_cy + (win_h / 2.0 + win_margin)

        xs = [p[0] for p in oled_boss_points_xy] + [wx0, wx1]
        ys = [p[1] for p in oled_boss_points_xy] + [wy0, wy1]

        edge_margin = 4.0
        rx0 = max(WALL + 0.4, min(xs) - edge_margin)
        rx1 = min(outer_x - WALL - 0.4, max(xs) + edge_margin)
        ry0 = max(WALL + 0.4, min(ys) - edge_margin)
        ry1 = min(outer_y - WALL - 0.4, max(ys) + edge_margin)

        # Place retainer with its top face flush to the boss bottom.
        z1_shell = float(oled_boss_bottom_z)
        z0_shell = z1_shell - float(OLED_RETAINER_THICKNESS)
        # Convert from shell coords -> top STL coords -> assembled coords.
        z0 = z0_shell + top_rebase_shift + bottom_height
        z1 = z1_shell + top_rebase_shift + bottom_height

        plate = box_mesh((rx0, ry0, z0), (rx1, ry1, z1))
        ret_cutters: List[trimesh.Trimesh] = []
        # Window cut.
        ret_cutters.append(box_mesh((wx0, wy0, z0 - 1.0), (wx1, wy1, z1 + 1.0)))
        # Boss clearance holes.
        hole_len = (z1 - z0) + 2.0
        hole_cz = (z0 + z1) / 2.0
        for (bx, by) in oled_boss_points_xy:
            ret_cutters.append(
                cylinder_z(
                    center_xyz=(bx, by, hole_cz),
                    radius=OLED_RETAINER_HOLE_DIAMETER / 2.0,
                    length=hole_len,
                )
            )
        # Wire notch.
        if OLED_RETAINER_WIRE_NOTCH:
            notch_w = float(OLED_RETAINER_WIRE_NOTCH_WIDTH_Y)
            notch_d = float(OLED_RETAINER_WIRE_NOTCH_DEPTH_X)
            ny0 = disp_cy - notch_w / 2.0
            ny1 = disp_cy + notch_w / 2.0
            pocket_half_w = (disp_pocket_w / 2.0) if disp_pocket_w else (win_w / 2.0)
            side = (DISPLAY_WIRE_NOTCH_SIDE or "right").strip().lower()
            if side == "left":
                left_x = disp_cx - pocket_half_w
                nx0 = left_x - notch_d
                nx1 = left_x + 0.2
            else:
                right_x = disp_cx + pocket_half_w
                nx0 = right_x - 0.2
                nx1 = right_x + notch_d
            # Clip notch to plate bounds.
            nx0 = max(rx0 - 0.5, nx0)
            nx1 = min(rx1 + 0.5, nx1)
            if nx1 > nx0 + 0.2:
                ret_cutters.append(box_mesh((nx0, ny0, z0 - 1.0), (nx1, ny1, z1 + 1.0)))

        retainer = boolean_diff(plate, ret_cutters)
        retainer.export(OUTPUT_OLED_RETAINER_STL)
        print(f"Wrote: {OUTPUT_OLED_RETAINER_STL}")

    # PCB preview slab in mounting position, exported in assembled coordinate frame.
    pcb_min_x = WALL + PCB_CLEARANCE_XY
    pcb_min_y = WALL + PCB_CLEARANCE_XY + CASE_EXTRA_Y_BOTTOM
    pcb_max_x = pcb_min_x + board_w
    pcb_max_y = pcb_min_y + board_h
    pcb_bottom_z = board_underside_z + bottom_rebase_shift
    pcb_preview = box_mesh(
        (pcb_min_x, pcb_min_y, pcb_bottom_z),
        (pcb_max_x, pcb_max_y, pcb_bottom_z + PCB_THICKNESS),
    )
    pcb_preview.export(OUTPUT_PCB_PREVIEW_STL)

    print(f"Wrote: {OUTPUT_BOTTOM_STL}")
    print(f"Wrote: {OUTPUT_TOP_STL}")
    print(f"Wrote: {OUTPUT_SIDE_BY_SIDE_STL}")
    print(f"Wrote: {OUTPUT_ASSEMBLED_STL}")
    print(f"Wrote: {OUTPUT_PCB_PREVIEW_STL}")
    bb = bottom.bounds
    tb = top.bounds
    print(f"Bottom bounds mm: x={bb[1][0]-bb[0][0]:.2f}, y={bb[1][1]-bb[0][1]:.2f}, z={bb[1][2]-bb[0][2]:.2f}")
    print(f"Top bounds mm: x={tb[1][0]-tb[0][0]:.2f}, y={tb[1][1]-tb[0][1]:.2f}, z={tb[1][2]-tb[0][2]:.2f}")


if __name__ == "__main__":
    main()
