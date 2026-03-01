#!/usr/bin/env python3
"""
Generate deterministic per-key RGB placement and chain mapping from SW1..SW26.

Outputs:
  - RGB_ALL26_PLACEMENT.csv
  - RGB_ALL26_CHAIN.csv
"""

from __future__ import annotations

import re
from pathlib import Path


PCB_PATH = Path("Macropad.kicad_pcb")
PLACEMENT_CSV = Path("RGB_ALL26_PLACEMENT.csv")
CHAIN_CSV = Path("RGB_ALL26_CHAIN.csv")

# Conservative LED center offset from switch center (mm).
# Adjust after phase-1 visual validation in KiCad.
RGB_OFFSET_X = 0.0
RGB_OFFSET_Y = 6.0


def parse_switches(kicad_lines: list[str]) -> list[tuple[int, str, float, float, float]]:
    switches: list[tuple[int, str, float, float, float]] = []
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
            if not ref_m or not at_m:
                continue
            ref = ref_m.group(1)
            if not (ref.startswith("SW") and ref[2:].isdigit()):
                continue
            idx = int(ref[2:])
            if not (1 <= idx <= 26):
                continue
            x = float(at_m.group(1))
            y = float(at_m.group(2))
            rot = float(at_m.group(3) or 0.0)
            switches.append((idx, ref, x, y, rot))
        else:
            i += 1
    switches.sort(key=lambda t: t[0])
    return switches


def main() -> None:
    lines = PCB_PATH.read_text(encoding="utf-8").splitlines()
    switches = parse_switches(lines)
    if len(switches) != 26:
        raise RuntimeError(f"Expected 26 switches, found {len(switches)}")

    placement_rows = [
        "rgb_ref,sw_ref,sw_x_mm,sw_y_mm,sw_rot_deg,rgb_x_mm,rgb_y_mm,rgb_rot_deg",
    ]
    chain_rows = [
        "rgb_ref,din_net,dout_net,notes",
    ]

    for idx, sw_ref, sw_x, sw_y, sw_rot in switches:
        rgb_ref = f"RGB{idx}"
        rgb_x = sw_x + RGB_OFFSET_X
        rgb_y = sw_y + RGB_OFFSET_Y
        placement_rows.append(
            f"{rgb_ref},{sw_ref},{sw_x:.3f},{sw_y:.3f},{sw_rot:.1f},{rgb_x:.3f},{rgb_y:.3f},{sw_rot:.1f}"
        )

        din = "LED_DIN" if idx == 1 else f"RGB{idx-1}_DOUT"
        dout = f"RGB{idx}_DOUT"
        note = "phase-1 validate first 4" if idx <= 4 else "replicate after phase-1"
        chain_rows.append(f"{rgb_ref},{din},{dout},{note}")

    PLACEMENT_CSV.write_text("\n".join(placement_rows) + "\n", encoding="utf-8")
    CHAIN_CSV.write_text("\n".join(chain_rows) + "\n", encoding="utf-8")
    print(f"Wrote {PLACEMENT_CSV}")
    print(f"Wrote {CHAIN_CSV}")


if __name__ == "__main__":
    main()
