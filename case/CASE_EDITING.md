# Editable Case Workflow (No Manifest)

Files:
- `generate_macropad_case.py` - the parametric source of truth
- `macropad_case_bottom.stl` - generated bottom piece
- `macropad_case_top.stl` - generated top piece
- `macropad_case_side_by_side.stl` - both pieces laid out side by side for preview

Reference inputs (authoritative):
- `/home/von/Desktop/Macropad/production/bom.csv`
- `/home/von/Desktop/Macropad/production/positions.csv`
- `/home/von/Desktop/Macropad/gerber/Macropad-Edge_Cuts.gm1`

## Regenerate
```bash
python3 /home/von/Desktop/case/generate_macropad_case.py
```

## What to edit
Open `generate_macropad_case.py` and adjust top-level parameters:
- `WALL`, `TOP_THICKNESS`, `BOTTOM_THICKNESS`, `CASE_HEIGHT`
- `TILT_DEG` (typing/elevation angle)
- `SWITCH_HOLE_SIZE`
- `OLED_WINDOW_SIZE`, `OLED_WINDOW_ANCHOR_REF`, `OLED_OFFSET_FROM_ANCHOR`
- `ROTARY_HOLE_ANCHOR_REF`, `ROTARY_HOLE_DIAMETER`
- `SIDE_SWITCH_SLOT`, `SIDE_SWITCH_ANCHOR_REF`, `SIDE_SWITCH_FROM_ANCHOR_Y`, `SIDE_SWITCH_Z`
- `RESET_HOLE_DIAMETER`, `RESET_HOLE_SPACING`, `RESET_ANCHOR_REF`, `RESET_OFFSET_FROM_ANCHOR`
- `IR_HOLE_DIAMETER`, `IR_HOLE_Z`
- `USB_CUTOUT_ANCHOR_REF`, `USB_CUTOUT_SIZE`, `USB_CUTOUT_Z`
- `PCB_STANDOFF_REFS`, `PCB_STANDOFF_DIAMETER`, `PCB_STANDOFF_HOLE_DIAMETER`, `PCB_STANDOFF_HEIGHT`
- `BOTTOM_STAND_CONTACT_SPAN_Y`, `BOTTOM_STAND_DEPTH`, `BOTTOM_STAND_MARGIN_X`, `BOTTOM_STAND_FROM_REAR_Y` (external tilt stand block)
- `BATTERY_ANCHOR_REF`, `BATTERY_OFFSET_FROM_ANCHOR`, `BATTERY_POCKET_SIZE`, `BATTERY_POD_WALL`, `BATTERY_FLOOR_THICKNESS`, `BATTERY_TO_PCB_CLEARANCE`
- `SPLIT_Z`, `PREVIEW_GAP_X`

PCB source path:
- `PCB_PATH = /home/von/Desktop/Macropad/Macropad.kicad_pcb`

The script parses footprint coordinates from the PCB each run (including SW1..SW26, SW401, J201/J202/J302/J400/J402, U901, H1..H4), so feature placement stays tied to your board.
