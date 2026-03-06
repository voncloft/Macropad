#!/usr/bin/env python3
"""Quick FreeCAD headless analysis of Macropad.step to locate the rotary shaft/knob center."""

from __future__ import annotations

from pathlib import Path
import math


def main() -> None:
    print("STEP_ANALYZE_START")
    try:
        import FreeCAD as App  # type: ignore
        import Import  # type: ignore
    except Exception as e:
        raise SystemExit(f"Run this with freecadcmd. FreeCAD modules not available: {e}")

    project_root = Path(__file__).resolve().parents[1]
    step_path = Path("/home/von/Desktop/Macropad.step")
    if not step_path.exists():
        raise SystemExit(f"Missing STEP: {step_path}")

    # SW401 per CPL (Macropad/production/positions.csv)
    target_x = 8.0
    target_y = 10.0
    r = 40.0

    doc = App.newDocument("MacropadStepAnalyze")
    Import.insert(str(step_path), doc.Name)
    doc.recompute()

    rows = []
    for obj in doc.Objects:
        bb = getattr(obj, "Shape", None)
        if bb is None:
            continue
        try:
            b = obj.Shape.BoundBox
        except Exception:
            continue
        # Filter out FreeCAD's infinite construction planes/axes and any giant artifacts.
        label = (getattr(obj, "Label", "") or "").lower()
        name = (getattr(obj, "Name", "") or "").lower()
        if "plane" in label or "axis" in label or "plane" in name or "axis" in name:
            continue
        dx = b.XMax - b.XMin
        dy = b.YMax - b.YMin
        dz = b.ZMax - b.ZMin
        if max(dx, dy, dz) > 500.0:
            continue
        cx = (b.XMin + b.XMax) / 2.0
        cy = (b.YMin + b.YMax) / 2.0
        cz = (b.ZMin + b.ZMax) / 2.0
        d = math.hypot(cx - target_x, cy - target_y)
        if d <= r:
            rows.append(
                (
                    d,
                    -b.ZMax,
                    obj.Name,
                    obj.Label,
                    cx,
                    cy,
                    cz,
                    dx,
                    dy,
                    dz,
                    b.XMin,
                    b.XMax,
                    b.YMin,
                    b.YMax,
                    b.ZMin,
                    b.ZMax,
                )
            )

    rows.sort()
    print(f"Objects within {r}mm of SW401({target_x},{target_y}) in STEP: {len(rows)}")
    for i, row in enumerate(rows[:40], 1):
        (d, _negz, name, label, cx, cy, cz, dx, dy, dz, x0, x1, y0, y1, z0, z1) = row
        print(
            f"{i:02d} d={d:6.2f} zmax={z1:7.2f}  size=({dx:5.1f},{dy:5.1f},{dz:5.1f})  {name}  label={label}  center=({cx:7.2f},{cy:7.2f},{cz:7.2f})  "
            f"bb=([{x0:7.2f},{y0:7.2f},{z0:7.2f}]..[{x1:7.2f},{y1:7.2f},{z1:7.2f}])"
        )

    # Also show the tallest nearby parts, which is usually the knob/shaft model if present.
    tall = sorted(rows, key=lambda t: (-t[15], t[0]))  # by zmax desc, then distance
    print("")
    print("Tallest candidates (zmax desc):")
    for i, row in enumerate(tall[:25], 1):
        (d, _negz, name, label, cx, cy, cz, dx, dy, dz, x0, x1, y0, y1, z0, z1) = row
        print(
            f"{i:02d} zmax={z1:7.2f} d={d:6.2f} size=({dx:5.1f},{dy:5.1f},{dz:5.1f})  {name}  label={label}  center=({cx:7.2f},{cy:7.2f},{cz:7.2f})"
        )


# NOTE: FreeCAD's `freecadcmd some_script.py` doesn't reliably set __name__ == "__main__".
# Execute unconditionally so this works both as a CLI script and as a FreeCAD macro.
main()
