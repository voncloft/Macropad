Macropad Rev-B Firmware Scaffold

Fixed MCU Target (Set In Stone)
- `ESP32-S3-WROOM-1-N8R8` only.
- Firmware pin mapping is `IOxx` / `GPIOxx` style.
- Do not use RP2040-style `GPxx` in this project.

Purpose
- Provide a starting firmware structure for:
  - Per-key RGB effects (#1)
  - OLED pages/animations (#3)
  - Encoder profiles (#4)
  - Hall analog control (#5)

Current hardware state
- RGB control net is `RGB_DATA` from MCU (pad 23), through `R401`, to `LED_DIN` (`J301` header).
- Hall input net is reserved as `HALL_OUT` on MCU pad 35.
- Display bus uses `LCD_SDA` / `LCD_SCL`.
- Encoder uses `ENC_A`, `ENC_B`, `ENC_SW`.
- USB-C (`J402`) is now configured for both charge/power and USB data flashing:
  - `VBUS` + `CC` keep battery charge/power path.
  - `USB_D+`/`USB_D-` are routed to ESP32-S3 USB pins.
  - `TOUCH_INT` moved to module pad `16` in firmware/PCB mapping.

What this scaffold is
- A practical template (`firmware/kmk/main.py`) showing feature logic and structure.
- Pin names are placeholders; map them to your board definition before flashing.

What this scaffold is not
- A complete, board-tested firmware binary.
- A substitute for selecting final firmware stack (KMK/QMK/custom ESP-IDF).

Next steps
1. Bind actual GPIO numbers to each signal in `firmware/kmk/main.py`.
2. Select final LED count/order for the chain.
3. Implement power-limit policy for RGB brightness.
4. Fill OLED page rendering functions with your final UI.
