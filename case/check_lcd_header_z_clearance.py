#!/usr/bin/env python3
"""
Check Z-clearance for LCD-related headers against the case interior roof.

This does not generate or modify STL files.
"""

from __future__ import annotations

import argparse
import ast
import re
import sys
from pathlib import Path
from typing import Dict, Tuple


ROOT = Path(__file__).resolve().parents[1]
PCB_PATH = ROOT / "Macropad" / "Macropad.kicad_pcb"
CASE_SCRIPT_PATH = ROOT / "case" / "generate_macropad_case.py"


def read_case_constants(path: Path) -> Dict[str, object]:
    needed = {
        "WALL",
        "PCB_CLEARANCE_XY",
        "CASE_HEIGHT",
        "TOP_THICKNESS",
        "BOTTOM_THICKNESS",
        "BATTERY_POCKET_SIZE",
        "BATTERY_TO_PCB_CLEARANCE",
        "PCB_STANDOFF_HEIGHT",
    }
    out: Dict[str, object] = {}
    assign_re = re.compile(r"^([A-Z0-9_]+)\s*=\s*(.+?)\s*(?:#.*)?$")

    for line in path.read_text(encoding="utf-8").splitlines():
        m = assign_re.match(line.strip())
        if not m:
            continue
        name, expr = m.group(1), m.group(2)
        if name not in needed:
            continue
        try:
            out[name] = ast.literal_eval(expr)
        except Exception:
            continue

    missing = sorted(needed - set(out))
    if missing:
        raise RuntimeError(f"Missing constants in {path}: {missing}")
    return out


def parse_edge_rect(kicad_text: str) -> Tuple[float, float, float, float]:
    m = re.search(
        r"\(gr_rect\s+\(start\s+([-0-9.]+)\s+([-0-9.]+)\)\s+\(end\s+([-0-9.]+)\s+([-0-9.]+)\)",
        kicad_text,
    )
    if not m:
        raise RuntimeError("Edge.Cuts gr_rect not found in PCB")
    x1, y1, x2, y2 = map(float, m.groups())
    min_x, max_x = sorted((x1, x2))
    min_y, max_y = sorted((y1, y2))
    return min_x, min_y, max_x, max_y


def parse_footprint_at(kicad_lines: list[str]) -> Dict[str, Tuple[float, float, float]]:
    out: Dict[str, Tuple[float, float, float]] = {}
    i = 0
    while i < len(kicad_lines):
        line = kicad_lines[i]
        if line.strip().startswith("(footprint "):
            block = [line]
            depth = line.count("(") - line.count(")")
            i += 1
            while i < len(kicad_lines) and depth > 0:
                l = kicad_lines[i]
                block.append(l)
                depth += l.count("(") - l.count(")")
                i += 1
            text = "\n".join(block)
            ref_m = re.search(r'\(property "Reference" "([^"]+)"', text)
            at_m = re.search(r'\n\s*\(at\s+([-0-9.]+)\s+([-0-9.]+)(?:\s+([-0-9.]+))?\)', text)
            if ref_m and at_m:
                ref = ref_m.group(1)
                x = float(at_m.group(1))
                y = float(at_m.group(2))
                rot = float(at_m.group(3) or 0.0)
                out[ref] = (x, y, rot)
        else:
            i += 1
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="LCD header Z-clearance check")
    parser.add_argument("--header-height", type=float, default=8.8, help="Header pin height above PCB top (mm)")
    parser.add_argument("--min-clearance", type=float, default=1.0, help="Required minimum Z clearance (mm)")
    args = parser.parse_args()

    pcb_text = PCB_PATH.read_text(encoding="utf-8")
    pcb_lines = pcb_text.splitlines()
    consts = read_case_constants(CASE_SCRIPT_PATH)
    min_x, min_y, max_x, max_y = parse_edge_rect(pcb_text)
    fps = parse_footprint_at(pcb_lines)

    refs = ("J302", "J303", "J304")
    missing = [r for r in refs if r not in fps]
    if missing:
        print(f"FAIL: missing footprints: {missing}")
        return 2

    board_w = max_x - min_x
    board_h = max_y - min_y
    wall = float(consts["WALL"])
    pcb_clear = float(consts["PCB_CLEARANCE_XY"])
    _ = board_h  # kept for completeness
    outer_x = board_w + 2.0 * (wall + pcb_clear)

    bottom_thickness = float(consts["BOTTOM_THICKNESS"])
    battery_pocket_z = float(consts["BATTERY_POCKET_SIZE"][2])
    battery_gap = float(consts["BATTERY_TO_PCB_CLEARANCE"])
    standoff = max(float(consts["PCB_STANDOFF_HEIGHT"]), battery_pocket_z + battery_gap)
    pcb_thickness = 1.6
    board_top_z = bottom_thickness + standoff + pcb_thickness

    case_height = float(consts["CASE_HEIGHT"])
    top_thickness = float(consts["TOP_THICKNESS"])
    roof_inner_z = case_height - top_thickness

    z_top = board_top_z + args.header_height
    clearance = roof_inner_z - z_top

    print("LCD Header Z Clearance Check")
    print(f"  board_top_z: {board_top_z:.2f} mm")
    print(f"  case_roof_inner_z: {roof_inner_z:.2f} mm")
    print(f"  header_height: {args.header_height:.2f} mm")
    print(f"  required_min_clearance: {args.min_clearance:.2f} mm")

    ok = clearance >= args.min_clearance
    for ref in refs:
        x, y, _rot = fps[ref]
        # Keep this for traceability to case-space placement users refer to.
        cx = (x - min_x) + wall + pcb_clear
        cx = outer_x - cx
        cy = (y - min_y) + wall + pcb_clear
        status = "PASS" if ok else "FAIL"
        print(f"  {ref}: case_xy=({cx:.2f}, {cy:.2f}), z_clearance={clearance:.2f} mm [{status}]")

    if not ok:
        print("FAIL: clearance is below threshold")
        return 1

    print("PASS: clearance meets threshold")
    return 0


if __name__ == "__main__":
    sys.exit(main())
