#!/usr/bin/env python3
"""
Create a FreeCAD document (.FCStd) that contains the generated case STL and a
simple PCB preview STL, so you can inspect fit/orientation in FreeCAD.

Run (headless):
  /tmp/freecad_appimage/squashfs-root/usr/bin/freecadcmd case_reworking/make_preview_fcstd.py
"""

from __future__ import annotations

from pathlib import Path


def main() -> None:
    try:
        import FreeCAD as App  # type: ignore
        import Mesh  # type: ignore
        import Import  # type: ignore
    except Exception as e:  # pragma: no cover
        raise SystemExit(f"FreeCAD Python modules not available. Run this with freecadcmd. Error: {e}")

    # When executed via `exec(open(...).read())` in FreeCAD console mode, __file__
    # is not defined. Fall back to the expected project layout (cwd/project root).
    try:
        here = Path(__file__).resolve().parent  # type: ignore[name-defined]
    except NameError:  # pragma: no cover
        here = (Path.cwd() / "case_reworking").resolve()
    case_stl = here / "macropad_case_assembled.stl"
    pcb_stl = here / "macropad_case_pcb_preview.stl"
    oled_retainer_stl = here / "macropad_oled_retainer.stl"
    out = here / "macropad_case_preview.FCStd"
    step_path = Path("/home/von/Desktop/Macropad.step")

    missing = [p for p in (case_stl, pcb_stl) if not p.exists()]
    if missing:
        raise SystemExit(f"Missing required STL(s): {missing}")

    doc = App.newDocument("MacropadCasePreview")

    def add_mesh(name: str, path: Path) -> None:
        obj = doc.addObject("Mesh::Feature", name)
        obj.Mesh = Mesh.Mesh(str(path))

    add_mesh("Case_Assembled", case_stl)
    add_mesh("PCB_Preview", pcb_stl)
    if oled_retainer_stl.exists():
        add_mesh("OLED_Retainer", oled_retainer_stl)

    # Optional: import the STEP assembly for a richer preview (components/board),
    # then translate it to align the PCB with the generated PCB preview slab.
    if step_path.exists():
        # Capture objects so we only translate the newly imported STEP items.
        before = {o.Name for o in doc.Objects}
        Import.insert(str(step_path), doc.Name)
        doc.recompute()
        imported = [o for o in doc.Objects if o.Name not in before]

        # Identify the PCB solid in the STEP import by approximate dimensions.
        pcb_obj = doc.getObject("PCB_Preview")
        pcb_bb = pcb_obj.Mesh.BoundBox  # type: ignore[union-attr]

        board_candidate = None
        best_score = None
        for o in imported:
            shape = getattr(o, "Shape", None)
            if shape is None:
                continue
            try:
                b = o.Shape.BoundBox
            except Exception:
                continue
            dx = b.XMax - b.XMin
            dy = b.YMax - b.YMin
            dz = b.ZMax - b.ZMin
            # PCB is ~112 x 163 x ~1.6 mm.
            if not (105.0 <= dx <= 125.0 and 155.0 <= dy <= 175.0 and 1.0 <= dz <= 3.5):
                continue
            # Prefer the closest match to the expected board dims.
            score = abs(dx - 112.25) + abs(dy - 163.0) + abs(dz - 1.6)
            if best_score is None or score < best_score:
                best_score = score
                board_candidate = (o, b)

        if board_candidate is not None:
            _board, b = board_candidate
            dx = pcb_bb.XMin - b.XMin
            dy = pcb_bb.YMin - b.YMin
            dz = pcb_bb.ZMin - b.ZMin
            v = App.Vector(dx, dy, dz)
            for o in imported:
                pl = getattr(o, "Placement", None)
                if pl is None:
                    continue
                o.Placement.Base = o.Placement.Base.add(v)
            doc.recompute()

    doc.recompute()
    doc.saveAs(str(out))
    print(f"Wrote: {out}")


# NOTE: FreeCAD's `freecadcmd some_script.py` doesn't reliably set __name__ == "__main__".
# Execute unconditionally so this works both as a CLI script and as a FreeCAD macro.
main()
