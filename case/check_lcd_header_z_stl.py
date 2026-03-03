#!/usr/bin/env python3
"""
STL-based Z collision check for J302/J303/J304 header tops.

This script does not edit PCB or STL files.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
import trimesh


def parse_edge_rect(kicad_text: str) -> tuple[float, float, float, float]:
    m = re.search(
        r"\(gr_rect\s+\(start\s+([-0-9.]+)\s+([-0-9.]+)\)\s+\(end\s+([-0-9.]+)\s+([-0-9.]+)\)",
        kicad_text,
    )
    if not m:
        raise RuntimeError("Edge.Cuts rectangle not found")
    x1, y1, x2, y2 = map(float, m.groups())
    min_x, max_x = sorted((x1, x2))
    min_y, max_y = sorted((y1, y2))
    return min_x, min_y, max_x, max_y


def parse_footprints(lines: list[str]) -> dict[str, tuple[float, float]]:
    out: dict[str, tuple[float, float]] = {}
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.strip().startswith("(footprint "):
            block = [line]
            depth = line.count("(") - line.count(")")
            i += 1
            while i < len(lines) and depth > 0:
                l = lines[i]
                block.append(l)
                depth += l.count("(") - l.count(")")
                i += 1
            text = "\n".join(block)
            rm = re.search(r'\(property "Reference" "([^"]+)"', text)
            am = re.search(r'\n\s*\(at\s+([-0-9.]+)\s+([-0-9.]+)(?:\s+[-0-9.]+)?\)', text)
            if rm and am:
                out[rm.group(1)] = (float(am.group(1)), float(am.group(2)))
        else:
            i += 1
    return out


def point_inside_mesh_manual(triangles: np.ndarray, point: np.ndarray) -> bool:
    # Moller-Trumbore against all triangles, odd hit-count = inside.
    ray_dir = np.array([0.931, 0.221, 0.289], dtype=float)
    ray_dir /= np.linalg.norm(ray_dir)
    eps = 1e-9

    v0 = triangles[:, 0, :]
    v1 = triangles[:, 1, :]
    v2 = triangles[:, 2, :]
    e1 = v1 - v0
    e2 = v2 - v0

    h = np.cross(np.tile(ray_dir, (len(triangles), 1)), e2)
    a = np.einsum("ij,ij->i", e1, h)
    mask = np.abs(a) > eps
    if not np.any(mask):
        return False

    f = np.zeros_like(a)
    f[mask] = 1.0 / a[mask]
    s = point - v0
    u = f * np.einsum("ij,ij->i", s, h)
    mask &= (u >= -eps) & (u <= 1.0 + eps)
    if not np.any(mask):
        return False

    q = np.cross(s, e1)
    v = f * np.einsum("ij,ij->i", np.tile(ray_dir, (len(triangles), 1)), q)
    mask &= (v >= -eps) & ((u + v) <= 1.0 + eps)
    if not np.any(mask):
        return False

    t = f * np.einsum("ij,ij->i", e2, q)
    mask &= t > 1e-6
    hits = int(np.count_nonzero(mask))
    return (hits % 2) == 1


def main() -> int:
    parser = argparse.ArgumentParser(description="STL collision check for LCD headers")
    parser.add_argument("--pcb", default="Macropad/Macropad.kicad_pcb")
    parser.add_argument("--top-stl", default="case/macropad_case_top.stl")
    parser.add_argument("--header-height", type=float, default=8.8, help="Header pin height above PCB top (mm)")
    parser.add_argument("--sample-radius", type=float, default=1.2, help="XY sample radius around center (mm)")
    args = parser.parse_args()

    pcb_path = Path(args.pcb)
    stl_path = Path(args.top_stl)
    if not pcb_path.exists():
        print(f"FAIL: missing PCB file: {pcb_path}")
        return 2
    if not stl_path.exists():
        print(f"FAIL: missing STL file: {stl_path}")
        return 2

    pcb_text = pcb_path.read_text(encoding="utf-8")
    min_x, min_y, max_x, _max_y = parse_edge_rect(pcb_text)
    fps = parse_footprints(pcb_text.splitlines())

    for ref in ("J302", "J303", "J304"):
        if ref not in fps:
            print(f"FAIL: {ref} missing in PCB")
            return 2

    # Keep in sync with case script constants.
    wall = 2.4
    pcb_clear_xy = 1.0
    case_height = 26.0
    top_thickness = 2.2
    bottom_thickness = 2.4
    battery_pocket_z = 6.8
    battery_to_pcb = 1.2
    pcb_standoff = 2.5
    pcb_thickness = 1.6
    mirror_top_features_x = True
    split_z = case_height * 0.72

    board_w = max_x - min_x
    outer_x = board_w + 2.0 * (wall + pcb_clear_xy)
    standoff = max(pcb_standoff, battery_pocket_z + battery_to_pcb)
    pcb_top_z = bottom_thickness + standoff + pcb_thickness
    header_top_global_z = pcb_top_z + float(args.header_height)
    z_in_top_stl = header_top_global_z - split_z
    roof_inner_z = case_height - top_thickness

    mesh = trimesh.load(str(stl_path), force="mesh")
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)
    triangles = np.asarray(mesh.triangles)

    print("STL Header Z Collision Check")
    print(f"  pcb: {pcb_path}")
    print(f"  top_stl: {stl_path}")
    print(f"  header_top_global_z: {header_top_global_z:.2f} mm")
    print(f"  header_top_in_top_stl_z: {z_in_top_stl:.2f} mm")
    print(f"  analytic_roof_clearance: {roof_inner_z - header_top_global_z:.2f} mm")

    offsets = [
        (0.0, 0.0),
        (args.sample_radius, 0.0),
        (-args.sample_radius, 0.0),
        (0.0, args.sample_radius),
        (0.0, -args.sample_radius),
    ]

    any_collision = False
    for ref in ("J302", "J303", "J304"):
        x, y = fps[ref]
        cx = (x - min_x) + wall + pcb_clear_xy
        cy = (y - min_y) + wall + pcb_clear_xy
        if mirror_top_features_x:
            cx = outer_x - cx
        hits = 0
        for dx, dy in offsets:
            p = np.array([cx + dx, cy + dy, z_in_top_stl], dtype=float)
            if point_inside_mesh_manual(triangles, p):
                hits += 1
        collided = hits > 0
        any_collision |= collided
        print(f"  {ref}: case_xy=({cx:.2f},{cy:.2f}) collision_samples={hits}/{len(offsets)}")

    if any_collision:
        print("FAIL: collision detected at header-top sample points")
        return 1

    print("PASS: no sampled STL collisions at header-top plane")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
