#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PCB="$ROOT/Macropad/Macropad.kicad_pcb"
DRC_OUT="$ROOT/Macropad/drc_jlc_preflight.rpt"

echo "[1/6] DRC"
kicad-cli pcb drc "$PCB" --output "$DRC_OUT" --format report >/tmp/jlc_drc.log 2>&1 || {
  cat /tmp/jlc_drc.log
  echo "FAIL: DRC command failed"
  exit 1
}
if ! grep -q "Found 0 DRC violations" "$DRC_OUT"; then
  if grep -q "hole_to_hole" "$DRC_OUT" && grep -q "J304" "$DRC_OUT" && grep -q "J302" "$DRC_OUT"; then
    echo "FAIL: locked-header placement blocker:"
    echo "  J304 pad5 (TOUCH_INT) drill overlaps J302 pad1 (3V3) drill."
    echo "  This cannot be fabricated reliably until one header location changes."
  else
    echo "FAIL: DRC violations found"
  fi
  cat "$DRC_OUT"
  exit 1
fi
if ! grep -q "Found 0 unconnected items" "$DRC_OUT"; then
  echo "FAIL: unconnected items found"
  cat "$DRC_OUT"
  exit 1
fi

echo "[2/6] Locked connector coordinates (J302/J303/J304)"
python3 - "$PCB" <<'PY'
import re, sys
from pathlib import Path
txt = Path(sys.argv[1]).read_text()
coords = {}
for m in re.finditer(r'\(footprint "[^"]+"\s+.*?\(at\s+([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)\).*?\(property "Reference" "(J30[234])"', txt, re.S):
    x,y,r,ref = float(m.group(1)), float(m.group(2)), float(m.group(3)), m.group(4)
    coords[ref] = (x,y,r)
expected = {
    "J302": (36.50, -6.50, 90.0),
    "J303": (26.73, -6.50, 90.0),
    "J304": (46.85, -6.50, 90.0),
}
for ref in ("J302","J303","J304"):
    if ref not in coords:
        print(f"FAIL: {ref} not found")
        raise SystemExit(1)
    got = coords[ref]
    exp = expected[ref]
    if any(abs(got[i]-exp[i]) > 0.02 for i in range(3)):
        print(f"FAIL: {ref} moved. got={got} expected={exp}")
        raise SystemExit(1)
print("PASS: J302/J303/J304 are locked")
PY

echo "[3/6] Z analytic check"
python3 "$ROOT/case/check_lcd_header_z_clearance.py" --header-height 8.8 --min-clearance 1.0

echo "[4/6] Z STL check"
python3 "$ROOT/case/check_lcd_header_z_stl.py" --header-height 8.8

echo "[5/6] Production artifacts exist"
for f in \
  "$ROOT/Macropad/production/bom.csv" \
  "$ROOT/Macropad/production/positions.csv" \
  "$ROOT/Macropad/production/Macropad.zip"; do
  if [[ ! -f "$f" ]]; then
    echo "FAIL: missing $f"
    exit 1
  fi
done
echo "PASS: BOM/CPL/zip present"

echo "[6/6] Schematic power-block warning gate"
if grep -q 'Portable power: onboard USB-C charging (J402+U403)' "$ROOT/Macropad/Macropad.kicad_sch"; then
  if ! grep -q '"U402"' "$ROOT/Macropad/Macropad.kicad_sch" || \
     ! grep -q '"U403"' "$ROOT/Macropad/Macropad.kicad_sch" || \
     ! grep -q '"U404"' "$ROOT/Macropad/Macropad.kicad_sch" || \
     ! grep -q '"J402"' "$ROOT/Macropad/Macropad.kicad_sch"; then
    echo "WARN: schematic power symbols still incomplete; manufacturing can proceed from PCB artifacts, but schematic sync is not yet fixed."
  fi
fi

echo "READY: preflight checks passed."
