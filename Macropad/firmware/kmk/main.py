# Rev-B integrated firmware template (KMK-style) for Macropad.
# Update board-specific pin mappings before flashing.
# MCU target is fixed for this project: ESP32-S3-WROOM-1-N8R8.

from dataclasses import dataclass, field
import json
import time
import os

from kmk.kmk_keyboard import KMKKeyboard
from kmk.keys import KC, make_key
from kmk.modules.layers import Layers
from kmk.modules.encoder import EncoderHandler
from kmk.extensions.rgb import RGB
from kmk.extensions.oled import Oled, OledData
from kmk.scanners import DiodeOrientation

try:
    from kmk.modules import Module
except Exception:
    class Module:  # type: ignore[override]
        pass

try:
    import board  # type: ignore
except Exception:
    board = None

try:
    from kmk.handlers.sequences import send_string  # type: ignore
except Exception:
    send_string = None


keyboard = KMKKeyboard()
layers = Layers()
keyboard.modules.append(layers)

TARGET_MCU = "ESP32-S3-WROOM-1-N8R8"
TARGET_PIN_PREFIX = "IO"

# ESP32-S3-WROOM-1 module pad number -> GPIO number.
# KMK/CircuitPython uses GPIO naming (board.IOxx), not module pad numbers.
MODULE_PAD_TO_GPIO = {
    4: 4,
    5: 5,
    6: 6,
    7: 7,
    8: 15,
    9: 16,
    10: 17,
    11: 18,
    12: 8,
    13: 19,
    14: 20,
    15: 3,
    16: 46,
    17: 9,
    18: 10,
    19: 11,
    20: 12,
    21: 13,
    22: 14,
    23: 21,
    24: 47,
    25: 48,
    26: 45,
    27: 0,
    28: 35,
    29: 36,
    30: 37,
    31: 38,
    32: 39,
    33: 40,
    34: 41,
    35: 42,
    36: 44,
    37: 43,
    38: 2,
    39: 1,
}

# I2C touch + shared SPI SD setup:
# - Touch controller (#2090-style): SDA=U901 pad 24, SCL=U901 pad 26
# - SD card on shared SPI: MISO=U901 pad 25, CS=U901 pad 27
TOUCH_SDA_PAD = 24
SPI_MISO_PAD = 25
TOUCH_SCL_PAD = 26
SD_CS_PAD = 27
LCD_SPI_MOSI_PAD = 33
LCD_SPI_SCLK_PAD = 34

ROW0_PAD = 4
ROW1_PAD = 5
ROW2_PAD = 6
ROW3_PAD = 7
ROW4_PAD = 12
ROW5_PAD = 17
COL0_PAD = 18
COL1_PAD = 19
COL2_PAD = 20
COL3_PAD = 21
COL4_PAD = 22

RGB_DATA_PAD = 23
TOUCH_INT_PAD = 16
ENC_A_PAD = 30
ENC_B_PAD = 31
ENC_SW_PAD = 32
IR_TX_PAD = 29

OLED_SDA_PAD = TOUCH_SDA_PAD
OLED_SCL_PAD = TOUCH_SCL_PAD


def _resolve_pin(gpio: int):
    if board is None:
        return None
    # Hard requirement: ESP32-S3 style board pin naming.
    # This intentionally rejects RP2040-style GPxx to prevent target drift.
    name = f"{TARGET_PIN_PREFIX}{gpio}"
    if hasattr(board, name):
        return getattr(board, name)
    # Some board definitions expose raw GPIOxx symbols; accept as secondary.
    alt = f"GPIO{gpio}"
    if hasattr(board, alt):
        return getattr(board, alt)
    return None


def _must_pin(pad: int, net_name: str):
    gpio = MODULE_PAD_TO_GPIO.get(pad)
    if gpio is None:
        raise ValueError(
            f"[{TARGET_MCU}] no module-pad->GPIO map for {net_name}: pad {pad}"
        )
    pin = _resolve_pin(gpio)
    if pin is None:
        raise ValueError(
            f"[{TARGET_MCU}] missing pin mapping for {net_name}: "
            f"module pad {pad} -> expected IO{gpio}/GPIO{gpio}"
        )
    return pin


def _optional_pin(pad: int):
    gpio = MODULE_PAD_TO_GPIO.get(pad)
    if gpio is None:
        return None
    return _resolve_pin(gpio)

# Placeholder extension setup (replace with your board pin mapping).
rgb = RGB(
    pixel_pin=_must_pin(RGB_DATA_PAD, "RGB_DATA"),
    num_pixels=26,
    val_default=28,          # conservative default brightness
)
keyboard.extensions.append(rgb)

oled = None
_oled_init_attempted = False

encoder_handler = EncoderHandler()
encoder_handler.pins = (
    (
        _must_pin(ENC_A_PAD, "ENC_A"),
        _must_pin(ENC_B_PAD, "ENC_B"),
        _must_pin(ENC_SW_PAD, "ENC_SW"),
    ),
)
keyboard.modules.append(encoder_handler)

keyboard.row_pins = (
    _must_pin(ROW0_PAD, "ROW0"),
    _must_pin(ROW1_PAD, "ROW1"),
    _must_pin(ROW2_PAD, "ROW2"),
    _must_pin(ROW3_PAD, "ROW3"),
    _must_pin(ROW4_PAD, "ROW4"),
    _must_pin(ROW5_PAD, "ROW5"),
)
keyboard.col_pins = (
    _must_pin(COL0_PAD, "COL0"),
    _must_pin(COL1_PAD, "COL1"),
    _must_pin(COL2_PAD, "COL2"),
    _must_pin(COL3_PAD, "COL3"),
    _must_pin(COL4_PAD, "COL4"),
)
keyboard.diode_orientation = DiodeOrientation.COL2ROW


def _try_init_oled():
    global oled, _oled_init_attempted
    if oled is not None:
        return True
    if _oled_init_attempted:
        return False
    _oled_init_attempted = True
    try:
        oled = Oled(
            OledData(
                pin_sda=_must_pin(OLED_SDA_PAD, "OLED_SDA"),
                pin_scl=_must_pin(OLED_SCL_PAD, "OLED_SCL"),
                i2c_addr=0x3C,
            )
        )
        keyboard.extensions.append(oled)
        _log("OLED initialized")
        return True
    except Exception as e:
        _log(f"OLED init skipped: {e}")
        return False


# Layer indices
BASE = 0
FN = 1
MEDIA = 2

# Per-layer RGB colors (R, G, B).
LAYER_COLORS = {
    BASE: (0, 40, 90),
    FN: (60, 10, 70),
    MEDIA: (0, 80, 25),
}

# Power status color overlay
POWER_OK = (0, 70, 0)
POWER_LOW = (90, 10, 0)
POWER_CHARGING = (80, 50, 0)

HALL_SMOOTH_ALPHA = 0.20
HALL_DEADZONE = 24
HALL_GESTURE_NEAR = 760
HALL_GESTURE_FAR = 300
HALL_GESTURE_COOLDOWN_S = 0.7

FUEL_GAUGE_I2C_ADDR = 0x36
LOW_BATTERY_PERCENT = 20.0
DEFERRED_INIT_DELAY_S = 1.5
VERBOSE_BOOT_LOG = True

IR_MAX_SLOTS = 12
MACRO_MAX_SLOTS = 12

POMODORO_WORK_MIN = 25
POMODORO_BREAK_MIN = 5

SW1_TAP_WINDOW_S = 0.35
IR_SCAN_PERIOD_S = 0.65
IR_TX_DEFAULT_FREQ_HZ = 38000
HALL_KEY_REPEAT_S = 0.12
WIFI_STATUS_POLL_S = 1.0

LCD_W = 320
LCD_H = 240
ICON_LONG_PRESS_S = 0.7
RGB_MAX_CHANNEL = 48
MACRO_STRING_MAX_LEN = 64
KEYLIST_VISIBLE_ROWS = 8
SWIPE_MIN_X = 44
SWIPE_MAX_Y_DRIFT = 28
SD_MOUNT_POINT = "/sd"
SD_CONFIG_FILE = "/sd/macropad_config.json"

# LCD panel + touch pin mapping placeholders. Update to your board mapping.
# Resolver supports both explicit board pin names and RP2040-style pad numbers.
LCD_DC_PIN_NAME = "LCD_DC"
LCD_CS_PIN_NAME = "LCD_CS"
LCD_RST_PIN_NAME = "LCD_RST"
LCD_BL_PIN_NAME = "LCD_BL"
LCD_SPI_SCK_PIN_NAME = "LCD_SCL"
LCD_SPI_MOSI_PIN_NAME = "LCD_SDA"
LCD_SPI_MISO_PIN_NAME = "LCD_MISO"


def _log(msg: str) -> None:
    if VERBOSE_BOOT_LOG:
        try:
            print(f"[macropad] {msg}")
        except Exception:
            pass


@dataclass
class RuntimeState:
    layer: int = BASE
    power_status: str = "ok"        # ok | low | charging
    oled_page: int = 0
    hall_raw: int = 0
    hall_filtered: float = 0.0
    hall_mode: str = "volume"       # volume | scrub | zoom
    hall_gesture_armed: bool = True
    hall_last_gesture_at: float = 0.0
    hall_last_emit_at: float = 0.0
    battery_percent: float = 100.0
    battery_voltage: float = 4.1
    ir_learning: bool = False
    ir_last_slot: int = -1
    macro_recording: bool = False
    macro_record_slot: int = -1
    webui_enabled: bool = False
    webui_url: str = "http://macropad.local/"
    wifi_connected: bool = False
    wifi_ssid: str = ""
    wifi_ip: str = ""
    wifi_subnet: str = ""
    wifi_gateway: str = ""
    wifi_last_poll_at: float = 0.0
    ir_scan_status: str = "idle"
    ui_page: str = "home"
    ui_status: str = "ready"
    context_app: str = "default"
    context_auto: bool = True


state = RuntimeState()


@dataclass
class FuelGaugeState:
    available: bool = False
    source: str = "none"


fuel_gauge = FuelGaugeState()


@dataclass
class IRCode:
    freq_hz: int
    pulses_us: list[int] = field(default_factory=list)


@dataclass
class IRLearnStore:
    slots: dict[int, IRCode] = field(default_factory=dict)
    capture_buf: list[int] = field(default_factory=list)


ir_store = IRLearnStore()
_ir_tx_init_attempted = False
_ir_pulseout = None


@dataclass
class MacroEvent:
    keycode: object
    delay_ms: int


@dataclass
class MacroStore:
    slots: dict[int, list[MacroEvent]] = field(default_factory=dict)
    key_bindings: dict[str, list[MacroEvent]] = field(default_factory=dict)
    _record_buf: list[MacroEvent] = field(default_factory=list)
    _record_last_at: float = 0.0


macro_store = MacroStore()


@dataclass
class MacroProgramState:
    active: bool = False
    selected_layer: int = BASE
    selected_key: int = -1
    wait_for_key: bool = False
    key_mode: str = "keys"  # keys | string
    key_tokens: list[str] = field(default_factory=list)
    string_value: str = ""


macro_program = MacroProgramState()


@dataclass
class IRLearnWizardState:
    active: bool = False
    candidate_idx: int = 0
    learned_code: IRCode | None = None
    selected_layer: int = MEDIA
    selected_key: int = -1
    wait_for_key: bool = False


ir_wizard = IRLearnWizardState()


@dataclass
class PomodoroState:
    running: bool = False
    on_break: bool = False
    work_s: int = POMODORO_WORK_MIN * 60
    break_s: int = POMODORO_BREAK_MIN * 60
    remaining_s: int = POMODORO_WORK_MIN * 60
    _last_tick: float = 0.0


pomodoro = PomodoroState()


@dataclass
class IRSceneStep:
    kind: str                   # ir | delay_ms | macro | layer
    value: object


@dataclass
class IRSceneStore:
    scenes: dict[str, list[IRSceneStep]] = field(default_factory=dict)


ir_scenes = IRSceneStore()


@dataclass
class TapDanceState:
    tap_count: int = 0
    last_tap_at: float = 0.0
    pending: bool = False


sw1_tap = TapDanceState()


@dataclass
class IRScanState:
    active: bool = False
    candidate_idx: int = 0
    last_send_at: float = 0.0
    period_s: float = IR_SCAN_PERIOD_S
    current: IRCode | None = None
    # Layer-3 key index (2..25) -> IR slot
    l3_key_slots: dict[int, int] = field(default_factory=dict)
    candidates: list[IRCode] = field(default_factory=list)


ir_scan = IRScanState()


@dataclass
class ToggleState:
    rgb_dynamic: bool = True
    haptics_enabled: bool = True
    focus_lock: bool = False
    safe_mode: bool = True


toggles = ToggleState()


@dataclass
class IconButton:
    id: str
    label: str
    x: int
    y: int
    w: int
    h: int
    action: str
    payload: object = None
    dangerous: bool = False
    user_defined: bool = False


@dataclass
class LCDUIState:
    pages: dict[str, list[IconButton]] = field(default_factory=dict)
    custom_buttons: dict[str, list[IconButton]] = field(default_factory=dict)
    nav_order: list[str] = field(default_factory=lambda: ["home", "dashboard", "macro", "ir", "hall", "toggles", "context", "keylist"])
    pressed_icon_id: str = ""
    press_started_at: float = 0.0
    touch_down_xy: tuple[int, int] = (0, 0)
    touch_active: bool = False
    battery_badge_pct: int = 100
    keylist_layer: int = BASE
    keylist_offset: int = 0
    edit_page: str = "home"
    edit_selected_idx: int = -1
    edit_preset_idx: int = 0


lcd_ui = LCDUIState()
_boot_started_at = time.monotonic()
_deferred_init_done = False
_web_server = None
_web_server_poll = None


def _run_deferred_init() -> None:
    global _deferred_init_done
    if _deferred_init_done:
        return
    if (time.monotonic() - _boot_started_at) < DEFERRED_INIT_DELAY_S:
        return
    _deferred_init_done = True
    _ensure_fuel_gauge()
    _try_init_oled()


class RuntimeHooks(Module):
    def before_matrix_scan(self, *_args, **_kwargs):
        apply_runtime()

    def after_hid_send(self, *_args, **_kwargs):
        apply_runtime()


keyboard.modules.append(RuntimeHooks())


UI_ACTION_PRESETS: list[tuple[str, str, object]] = [
    ("Go Home", "nav", "home"),
    ("Go Dash", "nav", "dashboard"),
    ("Go Macros", "nav", "macro"),
    ("Go IR", "nav", "ir"),
    ("Go Hall", "nav", "hall"),
    ("Go Toggles", "nav", "toggles"),
    ("Go Context", "nav", "context"),
    ("Go Keys", "nav", "keylist"),
    ("Pomo Toggle", "pomo_toggle", None),
    ("Start WebUI", "webui_start", None),
    ("Save Cfg", "cfg_save", None),
    ("Load Cfg", "cfg_load", None),
    ("Macro M0", "macro_play", 0),
    ("Macro M1", "macro_play", 1),
    ("IR Slot0", "ir_slot", 0),
    ("IR Slot1", "ir_slot", 1),
    ("Send A", "send_key", "A"),
    ("Send B", "send_key", "B"),
    ("Send Enter", "send_key", "ENT"),
    ("Send Esc", "send_key", "ESC"),
]


def set_layer_color(layer):
    color = LAYER_COLORS.get(layer, (20, 20, 20))
    rgb.set_hsv_fill(0, 0, 0)  # reset
    r, g, b = capped_rgb(color[0], color[1], color[2])
    rgb.set_rgb_fill(r, g, b)


def apply_power_overlay(status: str):
    # Keep this simple: tint key 0 as status indicator.
    if status == "ok":
        c = POWER_OK
    elif status == "low":
        c = POWER_LOW
    else:
        c = POWER_CHARGING
    r, g, b = capped_rgb(c[0], c[1], c[2])
    rgb.set_rgb(0, r, g, b)


def capped_rgb(r: int, g: int, b: int) -> tuple[int, int, int]:
    peak = max(int(r), int(g), int(b))
    if peak <= RGB_MAX_CHANNEL:
        return int(r), int(g), int(b)
    scale = RGB_MAX_CHANNEL / float(peak)
    return int(r * scale), int(g * scale), int(b * scale)


def _try_init_fuel_gauge():
    # Runtime-safe: if the board/lib is missing, keep firmware functional.
    try:
        import busio  # type: ignore
        from adafruit_max1704x import MAX17048  # type: ignore
    except Exception:
        return None
    try:
        if board is None:
            return None
        scl = _must_pin(TOUCH_SCL_PAD, "FUEL_GAUGE_SCL")
        sda = _must_pin(TOUCH_SDA_PAD, "FUEL_GAUGE_SDA")
        i2c = busio.I2C(scl, sda)
        return MAX17048(i2c, address=FUEL_GAUGE_I2C_ADDR)
    except Exception:
        return None


_fuel_gauge = None
_fuel_gauge_init_attempted = False


def _ensure_fuel_gauge():
    global _fuel_gauge, _fuel_gauge_init_attempted
    if _fuel_gauge is not None:
        return True
    if _fuel_gauge_init_attempted:
        return False
    _fuel_gauge_init_attempted = True
    _fuel_gauge = _try_init_fuel_gauge()
    if _fuel_gauge is not None:
        fuel_gauge.available = True
        fuel_gauge.source = "MAX17048"
        _log("Fuel gauge initialized")
        return True
    _log("Fuel gauge init skipped")
    return False


def update_power_status():
    # If a fuel gauge exists, use it; otherwise preserve the current status.
    if not fuel_gauge.available:
        return
    try:
        state.battery_percent = float(_fuel_gauge.cell_percent)  # type: ignore[attr-defined]
        state.battery_voltage = float(_fuel_gauge.cell_voltage)  # type: ignore[attr-defined]
        state.power_status = "low" if state.battery_percent <= LOW_BATTERY_PERCENT else "ok"
    except Exception:
        pass


def update_wifi_status(force: bool = False) -> None:
    now = time.monotonic()
    if not force and (now - state.wifi_last_poll_at) < WIFI_STATUS_POLL_S:
        return
    state.wifi_last_poll_at = now
    try:
        import wifi  # type: ignore
    except Exception:
        state.wifi_connected = False
        state.wifi_ssid = ""
        state.wifi_ip = ""
        state.wifi_subnet = ""
        state.wifi_gateway = ""
        return
    try:
        radio = wifi.radio
        ip = getattr(radio, "ipv4_address", None)
        subnet = getattr(radio, "ipv4_subnet", None)
        gateway = getattr(radio, "ipv4_gateway", None)
        ssid = ""
        try:
            ap_info = getattr(radio, "ap_info", None)
            if ap_info is not None and getattr(ap_info, "ssid", None):
                ssid = str(ap_info.ssid)
        except Exception:
            ssid = ""
        state.wifi_connected = ip is not None
        state.wifi_ssid = ssid
        state.wifi_ip = str(ip) if ip is not None else ""
        state.wifi_subnet = str(subnet) if subnet is not None else ""
        state.wifi_gateway = str(gateway) if gateway is not None else ""
    except Exception:
        state.wifi_connected = False
        state.wifi_ssid = ""
        state.wifi_ip = ""
        state.wifi_subnet = ""
        state.wifi_gateway = ""


# OLED page framework
OLED_PAGE_LAYER = 0
OLED_PAGE_POWER = 1
OLED_PAGE_PROFILE = 2


def oled_next_page():
    state.oled_page = (state.oled_page + 1) % 3


def oled_render() -> str:
    # Return display text payload; render method binding depends on board/OLED stack.
    if state.oled_page == OLED_PAGE_LAYER:
        return f"LAYER:{state.layer} RGB:{LAYER_COLORS.get(state.layer, (0, 0, 0))}"
    if state.oled_page == OLED_PAGE_POWER:
        return (
            f"POWER:{state.power_status} "
            f"{state.battery_percent:0.0f}% "
            f"{state.battery_voltage:0.2f}V "
            f"HALL:{int(state.hall_filtered)} "
            f"WIFI:{'UP' if state.wifi_connected else 'DOWN'}"
        )
    if pomodoro.running:
        mode = "BREAK" if pomodoro.on_break else "WORK"
        return f"POMO:{mode} T-{pomodoro.remaining_s}s MODE:{state.hall_mode}"
    if ir_wizard.active:
        total = len(ir_scan.candidates)
        return (
            f"IR_LEARN C{ir_wizard.candidate_idx + 1}/{max(1, total)} "
            f"L{ir_wizard.selected_layer + 1} K{ir_wizard.selected_key} "
            f"{state.ui_status}"
        )
    if ir_scan.active:
        return (
            f"IRSCAN:{ir_scan.candidate_idx + 1}/{len(ir_scan.candidates)} "
            "K26=STOP K2-25=SAVE"
        )
    icons = lcd_ui.pages.get(state.ui_page, [])
    preview = ",".join(i.label for i in icons[:4])
    return f"UI:{state.ui_page} {state.ui_status} [{preview}]"


def _macro_grid_icons() -> list[IconButton]:
    icons: list[IconButton] = []
    key = 2
    for r in range(6):
        for c in range(5):
            if key > 26:
                break
            x = 8 + c * 60
            y = 60 + r * 34
            icons.append(IconButton(f"mk_{key}", f"K{key}", x, y, 54, 28, "macro_pick_key", key))
            key += 1
    return icons


def _ir_grid_icons() -> list[IconButton]:
    icons: list[IconButton] = []
    key = 2
    for r in range(6):
        for c in range(5):
            if key > 26:
                break
            x = 8 + c * 60
            y = 60 + r * 34
            icons.append(IconButton(f"ik_{key}", f"K{key}", x, y, 54, 28, "ir_pick_key", key))
            key += 1
    return icons


def _macro_qwerty_icons() -> list[IconButton]:
    icons: list[IconButton] = []
    rows = ["QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"]
    y = 58
    for row in rows:
        x = 8
        for ch in row:
            icons.append(IconButton(f"mq_{ch}", ch, x, y, 28, 28, "macro_add_key", ch))
            x += 30
        y += 30
    return icons


def _macro_fn_icons() -> list[IconButton]:
    specs = [
        ("CTRL", "CTRL"), ("ALT", "ALT"), ("GUI", "GUI"), ("SHIFT", "SHIFT"),
        ("TAB", "TAB"), ("ESC", "ESC"), ("ENT", "ENT"), ("SPC", "SPC"),
        ("BSPC", "BSPC"), ("DEL", "DEL"), ("LEFT", "LEFT"), ("RIGHT", "RIGHT"),
        ("UP", "UP"), ("DOWN", "DOWN"), ("F1", "F1"), ("F2", "F2"),
        ("F3", "F3"), ("F4", "F4"), ("F5", "F5"), ("F6", "F6"),
    ]
    icons: list[IconButton] = []
    x = 8
    y = 58
    for idx, (label, token) in enumerate(specs):
        icons.append(IconButton(f"mfn_{token}", label, x, y, 58, 28, "macro_add_key", token))
        x += 62
        if (idx + 1) % 5 == 0:
            x = 8
            y += 32
    return icons


def _macro_string_icons() -> list[IconButton]:
    return [
        IconButton("mstr_fx", "firefox %U", 8, 54, 148, 32, "macro_set_string", "firefox %U"),
        IconButton("mstr_ch", "chrome %U", 162, 54, 148, 32, "macro_set_string", "chrome %U"),
        IconButton("mstr_term", "terminal", 8, 90, 96, 28, "macro_set_string", "terminal"),
        IconButton("mstr_code", "code .", 108, 90, 96, 28, "macro_set_string", "code ."),
        IconButton("mstr_obs", "obs", 208, 90, 102, 28, "macro_set_string", "obs"),
    ]


def _macro_string_char_icons() -> list[IconButton]:
    icons: list[IconButton] = []
    rows = [("qwertyuiop", 8), ("asdfghjkl", 22), ("zxcvbnm", 52)]
    y = 122
    for letters, x0 in rows:
        x = x0
        for ch in letters:
            icons.append(IconButton(f"msc_{ch}", ch, x, y, 26, 24, "macro_add_char", ch))
            x += 30
        y += 28
    icons.extend(
        [
            IconButton("msc_sp", "Space", 8, 206, 96, 24, "macro_add_char", " "),
            IconButton("msc_pct", "%", 108, 206, 30, 24, "macro_add_char", "%"),
            IconButton("msc_u", "U", 142, 206, 30, 24, "macro_add_char", "U"),
            IconButton("msc_dot", ".", 176, 206, 30, 24, "macro_add_char", "."),
            IconButton("msc_sl", "/", 210, 206, 30, 24, "macro_add_char", "/"),
            IconButton("msc_ds", "_", 244, 206, 30, 24, "macro_add_char", "_"),
            IconButton("msc_dash", "-", 278, 206, 32, 24, "macro_add_char", "-"),
        ]
    )
    return icons


def _keycode_label(keycode) -> str:
    if isinstance(keycode, tuple) and len(keycode) == 2:
        kind = str(keycode[0])
        if kind == "STRING":
            return f'STR:"{str(keycode[1])}"'
        if kind == "IR_SLOT":
            return f"IR_SLOT:{int(keycode[1])}"
        return f"{kind}:{keycode[1]}"
    return str(keycode)


def _macro_events_label(events: list[MacroEvent]) -> str:
    if not events:
        return "none"
    return "+".join(_keycode_label(ev.keycode) for ev in events[:3])


def _default_key_label(layer: int, key_index: int) -> str:
    if layer == BASE:
        return str(BASE_FALLBACKS.get(key_index, KC.TRNS))
    if layer == FN:
        return "TRNS"
    return "MEDIA"


def _key_action_label(layer: int, key_index: int) -> str:
    events = macro_get_key_binding(layer, key_index)
    if events:
        return f"Macro:{_macro_events_label(events)}"
    if layer == MEDIA and key_index in ir_scan.l3_key_slots:
        return f"IR_SLOT:{ir_scan.l3_key_slots[key_index]}"
    return f"Default:{_default_key_label(layer, key_index)}"


def ui_keylist_rows(layer: int) -> list[str]:
    rows: list[str] = []
    for key_index in range(2, 27):
        rows.append(f"K{key_index}: {_key_action_label(layer, key_index)}")
    return rows


def ui_keylist_visible_rows() -> list[str]:
    rows = ui_keylist_rows(lcd_ui.keylist_layer)
    start = max(0, min(lcd_ui.keylist_offset, max(0, len(rows) - KEYLIST_VISIBLE_ROWS)))
    end = min(len(rows), start + KEYLIST_VISIBLE_ROWS)
    return rows[start:end]


def _ui_set_keylist_status() -> None:
    rows = ui_keylist_rows(lcd_ui.keylist_layer)
    max_offset = max(0, len(rows) - KEYLIST_VISIBLE_ROWS)
    lcd_ui.keylist_offset = max(0, min(lcd_ui.keylist_offset, max_offset))
    start = lcd_ui.keylist_offset + 1
    end = min(len(rows), lcd_ui.keylist_offset + KEYLIST_VISIBLE_ROWS)
    state.ui_status = (
        f"keys:l{lcd_ui.keylist_layer + 1} rows {start}-{end}/{len(rows)} "
        f"bat:{lcd_ui.battery_badge_pct}%"
    )


def _preset_apply_to_icon(icon: IconButton, preset_idx: int) -> None:
    name, action, payload = UI_ACTION_PRESETS[preset_idx % len(UI_ACTION_PRESETS)]
    icon.label = str(name)[:12]
    icon.action = str(action)
    icon.payload = payload


def _ui_next_free_slot(page: str) -> tuple[int, int] | None:
    # Keep custom buttons below top nav band.
    slots = []
    for r in range(4):
        for c in range(4):
            slots.append((8 + c * 76, 114 + r * 30))
    existing = lcd_ui.pages.get(page, [])
    for x, y in slots:
        overlap = False
        for i in existing:
            if not (x + 70 < i.x or i.x + i.w < x or y + 26 < i.y or i.y + i.h < y):
                overlap = True
                break
        if not overlap:
            return (x, y)
    return None


def ui_editor_add_button() -> bool:
    page = lcd_ui.edit_page
    pos = _ui_next_free_slot(page)
    if pos is None:
        state.ui_status = "ui_edit:no_space"
        return False
    buttons = lcd_ui.custom_buttons.setdefault(page, [])
    btn = IconButton(
        id=f"usr_{page}_{len(buttons)}",
        label="NewBtn",
        x=pos[0],
        y=pos[1],
        w=70,
        h=26,
        action="nav",
        payload="home",
        user_defined=True,
    )
    buttons.append(btn)
    lcd_ui.edit_selected_idx = len(buttons) - 1
    state.ui_status = f"ui_edit:added_{page}"
    ui_define_pages()
    return True


def ui_editor_select(delta: int) -> bool:
    buttons = lcd_ui.custom_buttons.get(lcd_ui.edit_page, [])
    if not buttons:
        lcd_ui.edit_selected_idx = -1
        state.ui_status = "ui_edit:none"
        return False
    lcd_ui.edit_selected_idx = (lcd_ui.edit_selected_idx + delta) % len(buttons)
    state.ui_status = f"ui_edit:sel_{lcd_ui.edit_selected_idx + 1}"
    return True


def ui_editor_set_page(delta: int) -> bool:
    if not lcd_ui.nav_order:
        return False
    idx = lcd_ui.nav_order.index(lcd_ui.edit_page) if lcd_ui.edit_page in lcd_ui.nav_order else 0
    lcd_ui.edit_page = lcd_ui.nav_order[(idx + delta) % len(lcd_ui.nav_order)]
    lcd_ui.edit_selected_idx = 0 if lcd_ui.custom_buttons.get(lcd_ui.edit_page) else -1
    state.ui_status = f"ui_edit:page_{lcd_ui.edit_page}"
    return True


def ui_editor_apply_preset(delta: int) -> bool:
    buttons = lcd_ui.custom_buttons.get(lcd_ui.edit_page, [])
    if not buttons or lcd_ui.edit_selected_idx < 0:
        state.ui_status = "ui_edit:none"
        return False
    lcd_ui.edit_preset_idx = (lcd_ui.edit_preset_idx + delta) % len(UI_ACTION_PRESETS)
    icon = buttons[lcd_ui.edit_selected_idx]
    _preset_apply_to_icon(icon, lcd_ui.edit_preset_idx)
    state.ui_status = f"ui_edit:preset_{icon.label}"
    ui_define_pages()
    return True


def ui_editor_delete_selected() -> bool:
    buttons = lcd_ui.custom_buttons.get(lcd_ui.edit_page, [])
    if not buttons or lcd_ui.edit_selected_idx < 0:
        state.ui_status = "ui_edit:none"
        return False
    del buttons[lcd_ui.edit_selected_idx]
    if not buttons:
        lcd_ui.edit_selected_idx = -1
    else:
        lcd_ui.edit_selected_idx = min(lcd_ui.edit_selected_idx, len(buttons) - 1)
    state.ui_status = "ui_edit:deleted"
    ui_define_pages()
    return True


def ui_swipe_page(delta: int) -> bool:
    if state.ui_page not in lcd_ui.nav_order:
        return False
    idx = lcd_ui.nav_order.index(state.ui_page)
    state.ui_page = lcd_ui.nav_order[(idx + delta) % len(lcd_ui.nav_order)]
    if state.ui_page == "keylist":
        _ui_set_keylist_status()
    else:
        state.ui_status = f"swipe:{state.ui_page}"
    return True


def ui_define_pages() -> None:
    static_pages = {
        "home": [
            IconButton("to_dash", "Dash", 8, 8, 70, 48, "nav", "dashboard"),
            IconButton("to_macro", "Macros", 84, 8, 70, 48, "nav", "macro"),
            IconButton("to_ir", "IR", 160, 8, 70, 48, "nav", "ir"),
            IconButton("to_hall", "Hall", 236, 8, 70, 48, "nav", "hall"),
            IconButton("to_tog", "Toggles", 8, 62, 95, 48, "nav", "toggles"),
            IconButton("to_ctx", "Context", 109, 62, 95, 48, "nav", "context"),
            IconButton("to_web", "WebUI", 210, 62, 95, 48, "webui_start"),
            IconButton("to_pomo", "Pomodoro", 8, 116, 95, 48, "pomo_toggle"),
            IconButton("to_keys", "All Keys", 109, 116, 95, 48, "nav", "keylist"),
            IconButton("to_uiedit", "UI Edit", 210, 116, 95, 48, "nav", "ui_edit"),
        ],
        "keylist": [
            IconButton("kls_back", "Back", 8, 8, 56, 36, "nav", "home"),
            IconButton("kls_l1", "L1", 70, 8, 44, 36, "keylist_layer", BASE),
            IconButton("kls_l2", "L2", 118, 8, 44, 36, "keylist_layer", FN),
            IconButton("kls_l3", "L3", 166, 8, 44, 36, "keylist_layer", MEDIA),
            IconButton("kls_up", "Up", 216, 8, 44, 36, "keylist_scroll", -1),
            IconButton("kls_dn", "Down", 264, 8, 48, 36, "keylist_scroll", 1),
        ],
        "dashboard": [
            IconButton("dash_back", "Back", 8, 8, 70, 40, "nav", "home"),
            IconButton("dash_power", "Power", 84, 8, 70, 40, "nav", "dashboard_power"),
            IconButton("dash_hall", "Hall", 160, 8, 70, 40, "nav", "hall"),
            IconButton("dash_ir", "IR", 236, 8, 70, 40, "nav", "ir"),
        ],
        "macro": [
            IconButton("macro_back", "Back", 8, 8, 70, 40, "nav", "home"),
            IconButton("m0", "M0", 84, 8, 56, 40, "macro_play", 0),
            IconButton("m1", "M1", 146, 8, 56, 40, "macro_play", 1),
            IconButton("m2", "M2", 208, 8, 56, 40, "macro_play", 2),
            IconButton("m3", "M3", 270, 8, 40, 40, "macro_play", 3),
            IconButton("mclr", "Clear", 8, 54, 70, 40, "macro_clear", None, True),
            IconButton("mprog", "Program Key", 84, 54, 130, 40, "macro_prog_start"),
        ],
        "ir": [
            IconButton("ir_back", "Back", 8, 8, 70, 40, "nav", "home"),
            IconButton("ir_movie", "Movie", 84, 8, 70, 40, "ir_scene", "movie"),
            IconButton("ir0", "IR0", 160, 8, 56, 40, "ir_slot", 0),
            IconButton("ir1", "IR1", 222, 8, 56, 40, "ir_slot", 1),
            IconButton("irscan", "Scan", 8, 54, 70, 40, "ir_scan_toggle"),
            IconButton("irlearn", "IR Learn", 84, 54, 90, 40, "ir_learn_start"),
        ],
        "ir_learn_scan": [
            IconButton("ils_cancel", "Esc", 8, 8, 56, 36, "ir_learn_cancel"),
            IconButton("ils_prev", "Prev", 70, 8, 56, 36, "ir_learn_prev"),
            IconButton("ils_send", "Send", 132, 8, 56, 36, "ir_learn_send"),
            IconButton("ils_next", "Next", 194, 8, 56, 36, "ir_learn_next"),
            IconButton("ils_ok", "Works", 256, 8, 56, 36, "ir_learn_confirm"),
        ],
        "ir_learn_pick": [
            IconButton("ilp_cancel", "Esc", 8, 8, 56, 36, "ir_learn_cancel"),
            IconButton("ilp_l1", "L1", 70, 8, 44, 36, "ir_learn_layer_pick", BASE),
            IconButton("ilp_l2", "L2", 118, 8, 44, 36, "ir_learn_layer_pick", FN),
            IconButton("ilp_l3", "L3", 166, 8, 44, 36, "ir_learn_layer_pick", MEDIA),
            IconButton("ilp_wait", "Press Key", 214, 8, 98, 36, "ir_learn_wait_key"),
        ],
        "ir_learn_grid": [
            IconButton("ilg_back", "Back", 8, 8, 56, 36, "nav", "ir_learn_pick"),
            IconButton("ilg_wait", "Press Key", 70, 8, 86, 36, "ir_learn_wait_key"),
            IconButton("ilg_esc", "Esc", 162, 8, 56, 36, "ir_learn_cancel"),
        ] + _ir_grid_icons(),
        "ir_learn_overwrite": [
            IconButton("ilo_esc", "Esc", 8, 8, 56, 36, "ir_learn_cancel"),
            IconButton("ilo_no", "No", 70, 8, 56, 36, "ir_learn_overwrite_no"),
            IconButton("ilo_yes", "Yes", 132, 8, 56, 36, "ir_learn_overwrite_yes"),
            IconButton("ilo_pick", "Pick Again", 194, 8, 118, 36, "nav", "ir_learn_pick"),
        ],
        "hall": [
            IconButton("hall_back", "Back", 8, 8, 70, 40, "nav", "home"),
            IconButton("hall_vol", "Volume", 84, 8, 70, 40, "hall_mode", "volume"),
            IconButton("hall_scr", "Scrub", 160, 8, 70, 40, "hall_mode", "scrub"),
            IconButton("hall_zoom", "Zoom", 236, 8, 70, 40, "hall_mode", "zoom"),
        ],
        "toggles": [
            IconButton("tog_back", "Back", 8, 8, 70, 40, "nav", "home"),
            IconButton("tog_rgb", "RGB", 84, 8, 70, 40, "toggle", "rgb_dynamic"),
            IconButton("tog_hap", "Haptic", 160, 8, 70, 40, "toggle", "haptics_enabled"),
            IconButton("tog_focus", "Focus", 236, 8, 70, 40, "toggle", "focus_lock"),
            IconButton("tog_safe", "Safe", 8, 54, 70, 40, "toggle", "safe_mode"),
        ],
        "context": [
            IconButton("ctx_back", "Back", 8, 8, 70, 40, "nav", "home"),
            IconButton("ctx_auto", "Auto", 84, 8, 70, 40, "context_auto"),
            IconButton("ctx_code", "Code", 160, 8, 70, 40, "context_set", "code"),
            IconButton("ctx_game", "Game", 236, 8, 70, 40, "context_set", "game"),
            IconButton("ctx_media", "Media", 8, 54, 70, 40, "context_set", "media"),
        ],
        "dashboard_power": [
            IconButton("dp_back", "Back", 8, 8, 70, 40, "nav", "dashboard"),
            IconButton("dp_wifi", "WiFi", 84, 8, 70, 40, "nav", "wifi"),
            IconButton("dp_web", "WebUI", 160, 8, 70, 40, "webui_start"),
            IconButton("dp_pomo", "Pomo", 236, 8, 70, 40, "pomo_toggle"),
            IconButton("dp_cfg_save", "SaveCfg", 8, 54, 74, 40, "cfg_save"),
            IconButton("dp_cfg_load", "LoadCfg", 88, 54, 74, 40, "cfg_load"),
        ],
        "wifi": [
            IconButton("wf_back", "Back", 8, 8, 70, 40, "nav", "dashboard_power"),
            IconButton("wf_refresh", "Refresh", 84, 8, 70, 40, "wifi_refresh"),
            IconButton("wf_web", "Start Web", 160, 8, 90, 40, "webui_start"),
            IconButton("wf_link", f"Link:{'UP' if state.wifi_connected else 'DOWN'}", 8, 54, 148, 36, "wifi_refresh"),
            IconButton("wf_ssid", f"SSID:{state.wifi_ssid or '-'}", 160, 54, 152, 36, "wifi_refresh"),
            IconButton("wf_ip", f"IP:{state.wifi_ip or '-'}", 8, 96, 304, 32, "wifi_refresh"),
            IconButton("wf_subnet", f"Subnet:{state.wifi_subnet or '-'}", 8, 134, 304, 32, "wifi_refresh"),
            IconButton("wf_gateway", f"Gateway:{state.wifi_gateway or '-'}", 8, 172, 304, 32, "wifi_refresh"),
        ],
        "macro_prog_pick": [
            IconButton("mp_cancel", "Cancel", 8, 8, 70, 36, "macro_prog_cancel"),
            IconButton("mp_l1", "L1", 84, 8, 44, 36, "macro_prog_layer_pick", BASE),
            IconButton("mp_l2", "L2", 132, 8, 44, 36, "macro_prog_layer_pick", FN),
            IconButton("mp_l3", "L3", 180, 8, 44, 36, "macro_prog_layer_pick", MEDIA),
            IconButton("mp_wait", "Press Key", 228, 8, 84, 36, "macro_prog_wait_key"),
        ],
        "macro_prog_grid": [
            IconButton("mpg_back", "Back", 8, 8, 70, 36, "nav", "macro_prog_pick"),
            IconButton("mpg_wait", "Press Key", 84, 8, 86, 36, "macro_prog_wait_key"),
            IconButton("mpg_cancel", "Cancel", 176, 8, 70, 36, "macro_prog_cancel"),
        ] + _macro_grid_icons(),
        "macro_prog_overwrite": [
            IconButton("mow_no", "No", 8, 8, 70, 40, "nav", "macro_prog_pick"),
            IconButton("mow_yes", "Yes", 84, 8, 70, 40, "macro_prog_edit_begin"),
            IconButton("mow_del", "Delete", 160, 8, 80, 40, "macro_prog_delete", None, True),
        ],
        "macro_prog_keys": [
            IconButton("mpk_cancel", "Cancel", 8, 8, 70, 40, "macro_prog_cancel"),
            IconButton("mpk_fn", "Fn", 84, 8, 50, 40, "macro_prog_mode_fn"),
            IconButton("mpk_string", "String", 140, 8, 70, 40, "macro_prog_mode_string"),
            IconButton("mpk_bk", "Backsp", 216, 8, 70, 40, "macro_prog_backspace"),
            IconButton("mpk_save", "Save", 292, 8, 20, 40, "macro_prog_save"),
        ] + _macro_qwerty_icons(),
        "macro_prog_fn": [
            IconButton("mpf_back", "ABC", 8, 8, 60, 40, "macro_prog_mode_keys"),
            IconButton("mpf_string", "String", 74, 8, 70, 40, "macro_prog_mode_string"),
            IconButton("mpf_bk", "Backsp", 150, 8, 70, 40, "macro_prog_backspace"),
            IconButton("mpf_save", "Save", 226, 8, 70, 40, "macro_prog_save"),
        ] + _macro_fn_icons(),
        "macro_prog_string": [
            IconButton("mps_back", "ABC", 8, 8, 60, 40, "macro_prog_mode_keys"),
            IconButton("mps_clear", "Clear", 74, 8, 60, 40, "macro_prog_clear"),
            IconButton("mps_save", "Save", 140, 8, 60, 40, "macro_prog_save"),
            IconButton("mps_delete", "Delete", 206, 8, 70, 40, "macro_prog_delete", None, True),
        ] + _macro_string_icons() + _macro_string_char_icons(),
        "ui_edit": [
            IconButton("uie_back", "Back", 8, 8, 60, 40, "nav", "home"),
            IconButton("uie_pg_prev", "Pg<", 74, 8, 52, 40, "ui_edit_page", -1),
            IconButton("uie_pg_next", "Pg>", 132, 8, 52, 40, "ui_edit_page", 1),
            IconButton("uie_add", "+Btn", 190, 8, 56, 40, "ui_edit_add"),
            IconButton("uie_del", "Delete", 252, 8, 60, 40, "ui_edit_del", None, True),
            IconButton("uie_sel_prev", "Sel<", 8, 54, 68, 40, "ui_edit_sel", -1),
            IconButton("uie_sel_next", "Sel>", 82, 54, 68, 40, "ui_edit_sel", 1),
            IconButton("uie_act_prev", "Act<", 156, 54, 68, 40, "ui_edit_preset", -1),
            IconButton("uie_act_next", "Act>", 230, 54, 68, 40, "ui_edit_preset", 1),
        ],
    }
    merged: dict[str, list[IconButton]] = {}
    for page_name, base_icons in static_pages.items():
        merged[page_name] = list(base_icons) + list(lcd_ui.custom_buttons.get(page_name, []))
    lcd_ui.pages = merged


def _ui_icon_at(x: int, y: int) -> IconButton | None:
    for icon in lcd_ui.pages.get(state.ui_page, []):
        if icon.x <= x <= (icon.x + icon.w) and icon.y <= y <= (icon.y + icon.h):
            return icon
    return None


def _ui_run_action(icon: IconButton, held_s: float = 0.0) -> bool:
    if icon.dangerous and held_s < ICON_LONG_PRESS_S:
        state.ui_status = "hold_to_confirm"
        return False

    prev_status = state.ui_status
    a = icon.action
    p = icon.payload
    if a == "nav":
        state.ui_page = str(p)
        if state.ui_page == "keylist":
            _ui_set_keylist_status()
        elif state.ui_page == "wifi":
            update_wifi_status(force=True)
            ui_define_pages()
            state.ui_status = f"wifi:{'up' if state.wifi_connected else 'down'}"
    elif a == "macro_prog_start":
        macro_program_start()
    elif a == "macro_prog_layer_pick":
        macro_program_begin_manual_pick(int(p))
    elif a == "macro_prog_wait_key":
        macro_program.wait_for_key = True
        state.ui_page = "macro_prog_pick"
        state.ui_status = "macro:press_key"
    elif a == "macro_prog_mode_keys":
        macro_program.key_mode = "keys"
        state.ui_page = "macro_prog_keys"
        state.ui_status = "macro:mode_keys"
    elif a == "macro_prog_mode_fn":
        macro_program.key_mode = "keys"
        state.ui_page = "macro_prog_fn"
        state.ui_status = "macro:mode_fn"
    elif a == "macro_prog_mode_string":
        macro_program.key_mode = "string"
        state.ui_page = "macro_prog_string"
        state.ui_status = "macro:mode_string"
    elif a == "macro_pick_key":
        macro_program_choose_key(macro_program.selected_layer, int(p))
    elif a == "macro_prog_edit_begin":
        macro_program.key_mode = "keys"
        macro_program.key_tokens.clear()
        macro_program.string_value = ""
        state.ui_page = "macro_prog_keys"
        state.ui_status = "macro:edit"
    elif a == "macro_add_key":
        tok = str(p).upper()
        if len(macro_program.key_tokens) < 48:
            macro_program.key_mode = "keys"
            macro_program.key_tokens.append(tok)
            state.ui_status = f"macro:+{tok}"
    elif a == "macro_prog_backspace":
        if macro_program.key_mode == "string":
            macro_program.string_value = macro_program.string_value[:-1]
        elif macro_program.key_tokens:
            macro_program.key_tokens.pop()
        state.ui_status = "macro:backspace"
    elif a == "macro_prog_clear":
        macro_program.key_tokens.clear()
        macro_program.string_value = ""
        state.ui_status = "macro:clear"
    elif a == "macro_set_string":
        macro_program.key_mode = "string"
        macro_program.string_value = str(p)[:MACRO_STRING_MAX_LEN]
        state.ui_status = "macro:string_set"
    elif a == "macro_add_char":
        if len(macro_program.string_value) < MACRO_STRING_MAX_LEN:
            macro_program.key_mode = "string"
            macro_program.string_value += str(p)
        state.ui_status = "macro:string_add"
    elif a == "macro_prog_save":
        macro_program_save()
    elif a == "macro_prog_delete":
        if macro_delete_key_binding(macro_program.selected_layer, macro_program.selected_key):
            state.ui_status = (
                f"macro:deleted_l{macro_program.selected_layer + 1}k{macro_program.selected_key}"
            )
        else:
            state.ui_status = "macro:no_macro"
        _macro_program_reset()
        state.ui_page = "macro"
    elif a == "macro_prog_cancel":
        _macro_program_reset()
        state.ui_page = "macro"
        state.ui_status = "macro:cancel"
    elif a == "macro_play":
        macro_play(int(p))
    elif a == "macro_clear":
        macro_store.slots.clear()
    elif a == "ir_scene":
        ir_scene_run(str(p))
    elif a == "ir_slot":
        ir_send_slot(int(p))
    elif a == "ir_scan_toggle":
        ir_scan_toggle()
    elif a == "ir_learn_start":
        ir_learn_start()
    elif a == "ir_learn_cancel":
        ir_learn_cancel()
    elif a == "ir_learn_send":
        ir_learn_send_candidate()
    elif a == "ir_learn_next":
        ir_learn_next_candidate(1)
    elif a == "ir_learn_prev":
        ir_learn_next_candidate(-1)
    elif a == "ir_learn_confirm":
        ir_learn_mark_working()
    elif a == "ir_learn_layer_pick":
        ir_wizard.selected_layer = int(p)
        state.ui_page = "ir_learn_grid"
        state.ui_status = f"ir:pick_l{ir_wizard.selected_layer + 1}"
    elif a == "ir_learn_wait_key":
        ir_wizard.wait_for_key = True
        state.ui_page = "ir_learn_pick"
        state.ui_status = "ir:press_key"
    elif a == "ir_pick_key":
        ir_learn_choose_key(ir_wizard.selected_layer, int(p))
    elif a == "ir_learn_overwrite_no":
        ir_wizard.selected_key = -1
        ir_wizard.wait_for_key = True
        state.ui_page = "ir_learn_pick"
        state.ui_status = "ir:pick_again"
    elif a == "ir_learn_overwrite_yes":
        ir_learn_commit()
    elif a == "hall_mode":
        state.hall_mode = str(p)
    elif a == "wifi_refresh":
        update_wifi_status(force=True)
        ui_define_pages()
        state.ui_status = (
            f"wifi:{'up' if state.wifi_connected else 'down'} "
            f"ip:{state.wifi_ip or '-'}"
        )
    elif a == "toggle":
        name = str(p)
        if hasattr(toggles, name):
            setattr(toggles, name, not getattr(toggles, name))
    elif a == "webui_start":
        webui_start()
    elif a == "pomo_toggle":
        if pomodoro.running:
            pomodoro_stop()
        else:
            pomodoro_start()
    elif a == "context_set":
        state.context_app = str(p)
        state.context_auto = False
    elif a == "context_auto":
        state.context_auto = True
    elif a == "cfg_save":
        config_save_to_sd()
    elif a == "cfg_load":
        config_load_from_sd()
    elif a == "send_key":
        kc = _macro_token_to_keycode(str(p))
        if kc is None:
            state.ui_status = "ui:key_invalid"
            return False
        _emit_key(kc)
        state.ui_status = f"ui:key_{p}"
    elif a == "ui_edit_page":
        ui_editor_set_page(int(p))
    elif a == "ui_edit_add":
        ui_editor_add_button()
    elif a == "ui_edit_sel":
        ui_editor_select(int(p))
    elif a == "ui_edit_preset":
        ui_editor_apply_preset(int(p))
    elif a == "ui_edit_del":
        ui_editor_delete_selected()
    elif a == "keylist_layer":
        lcd_ui.keylist_layer = int(p)
        lcd_ui.keylist_offset = 0
        state.ui_page = "keylist"
        _ui_set_keylist_status()
    elif a == "keylist_scroll":
        lcd_ui.keylist_offset += int(p)
        state.ui_page = "keylist"
        _ui_set_keylist_status()
    else:
        return False
    if state.ui_status == prev_status:
        state.ui_status = f"ok:{icon.id}"
    return True


def ui_touch_down(x: int, y: int) -> bool:
    lcd_ui.touch_down_xy = (x, y)
    lcd_ui.touch_active = True
    icon = _ui_icon_at(x, y)
    if icon is None:
        return False
    lcd_ui.pressed_icon_id = icon.id
    lcd_ui.press_started_at = time.monotonic()
    state.ui_status = f"press:{icon.id}"
    return True


def ui_touch_up(x: int, y: int) -> bool:
    sx, sy = lcd_ui.touch_down_xy
    lcd_ui.touch_active = False
    dx = x - sx
    dy = y - sy
    if abs(dx) >= SWIPE_MIN_X and abs(dy) <= SWIPE_MAX_Y_DRIFT:
        lcd_ui.pressed_icon_id = ""
        if dx < 0:
            return ui_swipe_page(1)
        return ui_swipe_page(-1)
    icon = _ui_icon_at(x, y)
    pressed = lcd_ui.pressed_icon_id
    lcd_ui.pressed_icon_id = ""
    if icon is None or icon.id != pressed:
        return False
    held = time.monotonic() - lcd_ui.press_started_at
    return _ui_run_action(icon, held_s=held)


def ui_touch_tap(x: int, y: int) -> bool:
    # Convenience path for simple touch stacks that only emit tap.
    icon = _ui_icon_at(x, y)
    if icon is None:
        return False
    return _ui_run_action(icon, held_s=0.0)


def ui_set_context_app(app_name: str) -> None:
    state.context_app = app_name


def ui_apply_context_page() -> None:
    if not state.context_auto:
        return
    # Context-aware icon page switching.
    if state.context_app in {"code", "game", "media"}:
        state.ui_page = "context"


def ui_update_overlay_state() -> None:
    lcd_ui.battery_badge_pct = max(0, min(100, int(round(state.battery_percent))))
    if state.ui_page == "keylist":
        _ui_set_keylist_status()
    elif state.ui_page == "wifi":
        state.ui_status = (
            f"wifi:{'up' if state.wifi_connected else 'down'} "
            f"ip:{state.wifi_ip or '-'}"
        )


def lcd_render_model() -> dict:
    # UI model the LCD drawing backend can render:
    # - battery_badge_pct for a persistent top-right overlay
    # - keylist_rows for the scrollable "All Keys" page
    return {
        "page": state.ui_page,
        "status": state.ui_status,
        "battery_badge_pct": lcd_ui.battery_badge_pct,
        "icons": [icon.__dict__.copy() for icon in lcd_ui.pages.get(state.ui_page, [])],
        "nav_order": list(lcd_ui.nav_order),
        "edit_page": lcd_ui.edit_page,
        "edit_selected_idx": lcd_ui.edit_selected_idx,
        "edit_preset_idx": lcd_ui.edit_preset_idx,
        "keylist_layer": lcd_ui.keylist_layer,
        "keylist_offset": lcd_ui.keylist_offset,
        "keylist_rows": ui_keylist_visible_rows() if state.ui_page == "keylist" else [],
        "wifi_connected": state.wifi_connected,
        "wifi_ssid": state.wifi_ssid,
        "wifi_ip": state.wifi_ip,
        "wifi_subnet": state.wifi_subnet,
        "wifi_gateway": state.wifi_gateway,
    }


# Encoder profile table by active layer
ENCODER_PROFILES = {
    BASE: {"cw": KC.VOLU, "ccw": KC.VOLD, "press": KC.MUTE},
    FN: {"cw": KC.BRIU, "ccw": KC.BRID, "press": KC.PSCR},
    MEDIA: {"cw": KC.MNXT, "ccw": KC.MPRV, "press": KC.MPLY},
}


def encoder_action(direction: int, layer: int):
    profile = ENCODER_PROFILES.get(layer, ENCODER_PROFILES[BASE])
    return profile["cw"] if direction > 0 else profile["ccw"]


# Hall input framework.
def read_hall_raw() -> int:
    # TODO: Replace with ADC read from HALL_OUT GPIO.
    return 0


def update_hall_filter(raw: int) -> float:
    state.hall_raw = raw
    state.hall_filtered = (1.0 - HALL_SMOOTH_ALPHA) * state.hall_filtered + HALL_SMOOTH_ALPHA * raw
    return state.hall_filtered


def hall_delta(filtered_value: float, center: float = 512.0) -> int:
    delta = int(filtered_value - center)
    if abs(delta) < HALL_DEADZONE:
        return 0
    return delta


def hall_to_action(delta: int):
    if state.hall_mode == "volume":
        return KC.VOLU if delta > 0 else KC.VOLD
    if state.hall_mode == "scrub":
        return KC.RIGHT if delta > 0 else KC.LEFT
    return KC.PGDN if delta > 0 else KC.PGUP


def cycle_hall_mode():
    order = ["volume", "scrub", "zoom"]
    idx = order.index(state.hall_mode)
    state.hall_mode = order[(idx + 1) % len(order)]


def hall_gesture_update(filtered_value: float):
    # Gesture mode:
    # - Magnet NEAR toggles BASE<->MEDIA
    # - Magnet FAR cycles hall mode
    now = time.monotonic()
    if now - state.hall_last_gesture_at < HALL_GESTURE_COOLDOWN_S:
        return
    if filtered_value >= HALL_GESTURE_NEAR and state.hall_gesture_armed:
        state.hall_last_gesture_at = now
        state.hall_gesture_armed = False
        set_active_layer(MEDIA if state.layer == BASE else BASE)
        return
    if filtered_value <= HALL_GESTURE_FAR and not state.hall_gesture_armed:
        state.hall_last_gesture_at = now
        state.hall_gesture_armed = True
        cycle_hall_mode()


def ir_begin_learn():
    state.ir_learning = True
    ir_store.capture_buf.clear()


def ir_cancel_learn():
    state.ir_learning = False
    ir_store.capture_buf.clear()


def ir_ingest_pulse_us(pulse_us: int):
    # Call from your IR RX interrupt/decoder with pulse widths in microseconds.
    if state.ir_learning:
        ir_store.capture_buf.append(pulse_us)


def ir_commit_learn(slot: int, freq_hz: int = 38000) -> bool:
    if slot < 0 or slot >= IR_MAX_SLOTS or not ir_store.capture_buf:
        return False
    ir_store.slots[slot] = IRCode(freq_hz=freq_hz, pulses_us=list(ir_store.capture_buf))
    state.ir_last_slot = slot
    state.ir_learning = False
    ir_store.capture_buf.clear()
    return True


def ir_send_slot(slot: int) -> bool:
    code = ir_store.slots.get(slot)
    if code is None:
        return False
    return ir_send_code(code)


def ir_send_code(code: IRCode) -> bool:
    global _ir_tx_init_attempted, _ir_pulseout
    if not code.pulses_us:
        return False
    if _ir_pulseout is None and not _ir_tx_init_attempted:
        _ir_tx_init_attempted = True
        try:
            import pwmio  # type: ignore
            import pulseio  # type: ignore

            pin = _optional_pin(IR_TX_PAD)
            if pin is None:
                _log("IR TX init skipped: unresolved IR_TX pin")
                return False
            pwm = pwmio.PWMOut(
                pin,
                duty_cycle=2 ** 15,
                frequency=max(1000, int(code.freq_hz or IR_TX_DEFAULT_FREQ_HZ)),
                variable_frequency=True,
            )
            _ir_pulseout = pulseio.PulseOut(pwm)
            _log("IR TX initialized")
        except Exception as e:
            _log(f"IR TX init failed: {e}")
            _ir_pulseout = None
            return False
    if _ir_pulseout is None:
        return False
    try:
        _ir_pulseout.frequency = max(1000, int(code.freq_hz or IR_TX_DEFAULT_FREQ_HZ))
        pulses: list[int] = []
        for p in code.pulses_us:
            v = int(p)
            if v <= 0:
                continue
            pulses.append(min(65535, v))
        if not pulses:
            return False
        _ir_pulseout.send(pulses)
        return True
    except Exception as e:
        _log(f"IR TX send failed: {e}")
        return False


def ir_scan_prepare_candidates_from_known_codes() -> None:
    # Candidate pool for scan mode. Replace with a real TV power code db as needed.
    if ir_scan.candidates:
        return
    if ir_store.slots:
        ir_scan.candidates = [ir_store.slots[k] for k in sorted(ir_store.slots.keys())]
        return
    ir_scan.candidates = [
        IRCode(freq_hz=38000, pulses_us=[9000, 4500, 560, 560, 560, 1690]),
        IRCode(freq_hz=38000, pulses_us=[9000, 4500, 560, 1690, 560, 560]),
        IRCode(freq_hz=38000, pulses_us=[4500, 4500, 560, 560, 560, 560]),
    ]


def ir_scan_start() -> bool:
    ir_scan_prepare_candidates_from_known_codes()
    if not ir_scan.candidates:
        state.ir_scan_status = "no_candidates"
        return False
    ir_scan.active = True
    ir_scan.candidate_idx = 0
    ir_scan.current = ir_scan.candidates[0]
    ir_scan.last_send_at = 0.0
    state.ir_scan_status = "running"
    return True


def ir_scan_stop() -> None:
    ir_scan.active = False
    state.ir_scan_status = "idle"


def ir_scan_toggle() -> bool:
    if ir_scan.active:
        ir_scan_stop()
        return True
    return ir_scan_start()


def ir_scan_tick() -> None:
    if not ir_scan.active or not ir_scan.candidates:
        return
    now = time.monotonic()
    if now - ir_scan.last_send_at < ir_scan.period_s:
        return
    ir_scan.last_send_at = now
    ir_scan.current = ir_scan.candidates[ir_scan.candidate_idx]
    ir_send_code(ir_scan.current)
    ir_scan.candidate_idx = (ir_scan.candidate_idx + 1) % len(ir_scan.candidates)


def _ir_wizard_reset() -> None:
    ir_wizard.active = False
    ir_wizard.candidate_idx = 0
    ir_wizard.learned_code = None
    ir_wizard.selected_layer = MEDIA
    ir_wizard.selected_key = -1
    ir_wizard.wait_for_key = False


def _ir_next_slot() -> int:
    for i in range(IR_MAX_SLOTS):
        if i not in ir_store.slots:
            return i
    return 0


def _ir_key_has_assignment(layer: int, key_index: int) -> bool:
    if macro_get_key_binding(layer, key_index):
        return True
    if layer == MEDIA and key_index in ir_scan.l3_key_slots:
        return True
    return False


def ir_learn_start() -> bool:
    ir_scan_prepare_candidates_from_known_codes()
    if not ir_scan.candidates:
        state.ui_status = "ir_learn:no_candidates"
        return False
    _ir_wizard_reset()
    ir_wizard.active = True
    ir_wizard.candidate_idx = 0
    state.ui_page = "ir_learn_scan"
    state.ui_status = f"ir_learn:ready_1_of_{len(ir_scan.candidates)}"
    return True


def ir_learn_cancel() -> None:
    _ir_wizard_reset()
    state.ui_page = "ir"
    state.ui_status = "ir_learn:esc"


def ir_learn_send_candidate() -> bool:
    if not ir_wizard.active or not ir_scan.candidates:
        state.ui_status = "ir_learn:not_active"
        return False
    code = ir_scan.candidates[ir_wizard.candidate_idx]
    if not ir_send_code(code):
        state.ui_status = "ir_learn:send_fail"
        return False
    state.ui_status = (
        f"ir_learn:sent_{ir_wizard.candidate_idx + 1}_of_{len(ir_scan.candidates)}"
    )
    return True


def ir_learn_next_candidate(delta: int) -> bool:
    if not ir_wizard.active or not ir_scan.candidates:
        state.ui_status = "ir_learn:not_active"
        return False
    n = len(ir_scan.candidates)
    ir_wizard.candidate_idx = (ir_wizard.candidate_idx + int(delta)) % n
    state.ui_status = f"ir_learn:candidate_{ir_wizard.candidate_idx + 1}_of_{n}"
    return True


def ir_learn_mark_working() -> bool:
    if not ir_wizard.active or not ir_scan.candidates:
        state.ui_status = "ir_learn:not_active"
        return False
    code = ir_scan.candidates[ir_wizard.candidate_idx]
    ir_wizard.learned_code = IRCode(freq_hz=code.freq_hz, pulses_us=list(code.pulses_us))
    ir_wizard.wait_for_key = True
    state.ui_page = "ir_learn_pick"
    state.ui_status = "ir_learn:code_ok_press_key"
    return True


def ir_learn_choose_key(layer: int, key_index: int) -> bool:
    if not ir_wizard.active or ir_wizard.learned_code is None:
        state.ui_status = "ir_learn:not_ready"
        return False
    if key_index == 1:
        state.ui_status = "ir_learn:sw1_locked"
        return False
    ir_wizard.selected_layer = int(layer)
    ir_wizard.selected_key = int(key_index)
    ir_wizard.wait_for_key = False
    if _ir_key_has_assignment(layer, key_index):
        state.ui_page = "ir_learn_overwrite"
        state.ui_status = f"ir_learn:key_l{layer + 1}k{key_index}_occupied"
        return True
    return ir_learn_commit()


def ir_learn_handle_physical_key(key_index: int) -> bool:
    if not (ir_wizard.active and ir_wizard.wait_for_key):
        return False
    return ir_learn_choose_key(ir_wizard.selected_layer, key_index)


def ir_learn_commit() -> bool:
    if not ir_wizard.active or ir_wizard.learned_code is None:
        state.ui_status = "ir_learn:not_ready"
        return False
    layer = ir_wizard.selected_layer
    key_index = ir_wizard.selected_key
    if key_index <= 0 or key_index == 1:
        state.ui_status = "ir_learn:no_target"
        return False

    slot = _ir_next_slot()
    ir_store.slots[slot] = IRCode(
        freq_hz=ir_wizard.learned_code.freq_hz,
        pulses_us=list(ir_wizard.learned_code.pulses_us),
    )
    ir_scan.l3_key_slots[key_index] = slot
    macro_set_key_binding(layer, key_index, [MacroEvent(keycode=("IR_SLOT", slot), delay_ms=0)])

    state.ui_status = f"ir_learn:saved_slot{slot}_l{layer + 1}k{key_index}"
    _ir_wizard_reset()
    state.ui_page = "ir"
    return True


def l3_ir_key_press(key_index: int) -> bool:
    # Layer 3 behavior:
    # - Key 26: start/stop scan
    # - While scanning, keys 2..25 save current candidate to that key
    # - When not scanning, keys 2..25 send saved code
    if key_index == 26:
        return ir_scan_toggle()
    if key_index in (1, 26):
        return False

    if ir_scan.active:
        if ir_scan.current is None:
            return False
        slot = key_index
        ir_store.slots[slot] = IRCode(
            freq_hz=ir_scan.current.freq_hz,
            pulses_us=list(ir_scan.current.pulses_us),
        )
        ir_scan.l3_key_slots[key_index] = slot
        state.ir_last_slot = slot
        state.ir_scan_status = f"saved_k{key_index}"
        ir_scan_stop()
        return True

    slot = ir_scan.l3_key_slots.get(key_index)
    if slot is None:
        state.ir_scan_status = f"empty_k{key_index}"
        return False
    return ir_send_slot(slot)


def sw1_tap_press() -> None:
    sw1_tap.tap_count += 1
    sw1_tap.last_tap_at = time.monotonic()
    sw1_tap.pending = True


def sw1_tap_tick() -> None:
    if not sw1_tap.pending:
        return
    if time.monotonic() - sw1_tap.last_tap_at < SW1_TAP_WINDOW_S:
        return
    taps = sw1_tap.tap_count
    sw1_tap.tap_count = 0
    sw1_tap.pending = False
    if taps == 1:
        set_active_layer(BASE)
    elif taps == 2:
        set_active_layer(FN)
    else:
        set_active_layer(MEDIA)


def _mk_sw1_key():
    def _press(*_args, **_kwargs):
        sw1_tap_press()
    return make_key(names=("SW1TD",), on_press=_press)


def _handle_matrix_key_press(key_index: int, default_keycode, media_key: bool = False):
    # Hard lock: SW1 is tap-dance only.
    if key_index == 1:
        return
    # If IR learn wizard is waiting for key assignment, consume press.
    if ir_learn_handle_physical_key(key_index):
        return
    # If macro programmer is waiting for physical key selection, consume press.
    if macro_program_handle_physical_key(key_index):
        return
    # Assigned macro takes priority over default key behavior.
    if macro_play_key_binding(state.layer, key_index):
        return
    if media_key:
        l3_ir_key_press(key_index)
        return
    macro_record_key(default_keycode)
    _emit_key(default_keycode)


def _mk_base_key(index: int, default_keycode):
    def _press(*_args, **_kwargs):
        _handle_matrix_key_press(index, default_keycode, media_key=False)
    return make_key(names=(f"BKEY{index}",), on_press=_press)


def _mk_fn_key(index: int):
    def _press(*_args, **_kwargs):
        _handle_matrix_key_press(index, KC.TRNS, media_key=False)
    return make_key(names=(f"FKEY{index}",), on_press=_press)


def _mk_l3_key(index: int):
    def _press(*_args, **_kwargs):
        _handle_matrix_key_press(index, KC.TRNS, media_key=True)
    return make_key(names=(f"MKEY{index}",), on_press=_press)


SW1_TD = _mk_sw1_key()
BASE_FALLBACKS = {
    2: KC.N2, 3: KC.N3, 4: KC.N4, 5: KC.N5, 6: KC.N6,
    7: KC.N7, 8: KC.N8, 9: KC.N9, 10: KC.N0, 11: KC.MINS, 12: KC.EQL,
    13: KC.Q, 14: KC.W, 15: KC.E, 16: KC.R, 17: KC.T, 18: KC.Y,
    19: KC.A, 20: KC.S, 21: KC.D, 22: KC.F, 23: KC.G, 24: KC.H,
    25: KC.Z, 26: KC.X,
}
BASE_KEYS = {i: _mk_base_key(i, BASE_FALLBACKS[i]) for i in range(2, 27)}
FN_KEYS = {i: _mk_fn_key(i) for i in range(2, 27)}
MEDIA_KEYS = {i: _mk_l3_key(i) for i in range(2, 27)}
L3_IR_KEYS = MEDIA_KEYS


def _emit_key(keycode):
    tap = getattr(keyboard, "tap_key", None)
    if isinstance(keycode, tuple) and len(keycode) == 2 and keycode[0] == "STRING":
        text = str(keycode[1])
        if callable(tap) and send_string is not None:
            try:
                tap(send_string(text))
            except Exception:
                pass
        return
    if isinstance(keycode, tuple) and len(keycode) == 2 and keycode[0] == "IR_SLOT":
        ir_send_slot(int(keycode[1]))
        return
    if callable(tap):
        try:
            tap(keycode)
            return
        except Exception:
            pass
    press = getattr(keyboard, "press_key", None)
    release = getattr(keyboard, "release_key", None)
    if callable(press) and callable(release):
        try:
            press(keycode)
            release(keycode)
        except Exception:
            pass


def _macro_binding_id(layer: int, key_index: int) -> str:
    return f"L{int(layer)}:K{int(key_index)}"


def _keycode_to_doc(keycode):
    if isinstance(keycode, tuple) and len(keycode) == 2:
        return {"tuple": [str(keycode[0]), keycode[1]]}
    text = str(keycode)
    if text.startswith("KC."):
        return {"kc": text[3:]}
    return {"raw": text}


def _doc_to_keycode(doc):
    if not isinstance(doc, dict):
        return None
    if "tuple" in doc and isinstance(doc["tuple"], list) and len(doc["tuple"]) == 2:
        return (doc["tuple"][0], doc["tuple"][1])
    if "kc" in doc:
        name = str(doc["kc"])
        return getattr(KC, name, None)
    return None


def _events_to_doc(events: list[MacroEvent]) -> list[dict]:
    out: list[dict] = []
    for ev in events:
        out.append({"keycode": _keycode_to_doc(ev.keycode), "delay_ms": int(ev.delay_ms)})
    return out


def _events_from_doc(doc_items: list) -> list[MacroEvent]:
    out: list[MacroEvent] = []
    for item in doc_items:
        if not isinstance(item, dict):
            continue
        kc = _doc_to_keycode(item.get("keycode"))
        if kc is None:
            continue
        out.append(MacroEvent(keycode=kc, delay_ms=int(item.get("delay_ms", 0))))
    return out


def _icon_to_doc(icon: IconButton) -> dict:
    return {
        "id": str(icon.id),
        "label": str(icon.label),
        "x": int(icon.x),
        "y": int(icon.y),
        "w": int(icon.w),
        "h": int(icon.h),
        "action": str(icon.action),
        "payload": icon.payload,
        "dangerous": bool(icon.dangerous),
        "user_defined": bool(icon.user_defined),
    }


def _icon_from_doc(doc: dict) -> IconButton | None:
    if not isinstance(doc, dict):
        return None
    try:
        return IconButton(
            id=str(doc.get("id", "usr_btn")),
            label=str(doc.get("label", "Btn")),
            x=int(doc.get("x", 8)),
            y=int(doc.get("y", 114)),
            w=int(doc.get("w", 70)),
            h=int(doc.get("h", 26)),
            action=str(doc.get("action", "nav")),
            payload=doc.get("payload"),
            dangerous=bool(doc.get("dangerous", False)),
            user_defined=bool(doc.get("user_defined", True)),
        )
    except Exception:
        return None


def _build_config_doc() -> dict:
    macro_doc: dict[str, list[dict]] = {}
    for key, events in macro_store.key_bindings.items():
        macro_doc[key] = _events_to_doc(events)
    ir_doc: dict[str, dict] = {}
    for slot, code in ir_store.slots.items():
        ir_doc[str(slot)] = {"freq_hz": int(code.freq_hz), "pulses_us": list(code.pulses_us)}
    ui_custom_doc: dict[str, list[dict]] = {}
    for page, icons in lcd_ui.custom_buttons.items():
        ui_custom_doc[page] = [_icon_to_doc(i) for i in icons]
    return {
        "version": 1,
        "layer": int(state.layer),
        "hall_mode": state.hall_mode,
        "context_app": state.context_app,
        "context_auto": bool(state.context_auto),
        "toggles": {
            "rgb_dynamic": bool(toggles.rgb_dynamic),
            "haptics_enabled": bool(toggles.haptics_enabled),
            "focus_lock": bool(toggles.focus_lock),
            "safe_mode": bool(toggles.safe_mode),
        },
        "macro_key_bindings": macro_doc,
        "ir_slots": ir_doc,
        "l3_key_slots": {str(k): int(v) for k, v in ir_scan.l3_key_slots.items()},
        "ui": {
            "custom_buttons": ui_custom_doc,
            "edit_page": lcd_ui.edit_page,
            "edit_selected_idx": int(lcd_ui.edit_selected_idx),
            "edit_preset_idx": int(lcd_ui.edit_preset_idx),
        },
    }


def _apply_config_doc(doc: dict) -> bool:
    if not isinstance(doc, dict):
        return False
    if "layer" in doc:
        set_active_layer(int(doc["layer"]))
    if "hall_mode" in doc:
        state.hall_mode = str(doc["hall_mode"])
    if "context_app" in doc:
        state.context_app = str(doc["context_app"])
    if "context_auto" in doc:
        state.context_auto = bool(doc["context_auto"])
    td = doc.get("toggles", {})
    if isinstance(td, dict):
        for name in ("rgb_dynamic", "haptics_enabled", "focus_lock", "safe_mode"):
            if name in td and hasattr(toggles, name):
                setattr(toggles, name, bool(td[name]))

    mkd = doc.get("macro_key_bindings", {})
    if isinstance(mkd, dict):
        macro_store.key_bindings.clear()
        for key, items in mkd.items():
            if not isinstance(items, list):
                continue
            events = _events_from_doc(items)
            if events:
                macro_store.key_bindings[str(key)] = events

    ird = doc.get("ir_slots", {})
    if isinstance(ird, dict):
        ir_store.slots.clear()
        for slot_s, cdoc in ird.items():
            if not isinstance(cdoc, dict):
                continue
            try:
                slot = int(slot_s)
                freq = int(cdoc.get("freq_hz", 38000))
                pulses = [int(x) for x in cdoc.get("pulses_us", [])]
            except Exception:
                continue
            ir_store.slots[slot] = IRCode(freq_hz=freq, pulses_us=pulses)

    l3d = doc.get("l3_key_slots", {})
    if isinstance(l3d, dict):
        ir_scan.l3_key_slots.clear()
        for k, v in l3d.items():
            try:
                ir_scan.l3_key_slots[int(k)] = int(v)
            except Exception:
                continue

    uid = doc.get("ui", {})
    if isinstance(uid, dict):
        cbd = uid.get("custom_buttons", {})
        if isinstance(cbd, dict):
            lcd_ui.custom_buttons.clear()
            for page, items in cbd.items():
                if not isinstance(items, list):
                    continue
                out: list[IconButton] = []
                for item in items:
                    icon = _icon_from_doc(item)
                    if icon is not None:
                        out.append(icon)
                if out:
                    lcd_ui.custom_buttons[str(page)] = out
        if "edit_page" in uid:
            lcd_ui.edit_page = str(uid.get("edit_page"))
        if "edit_selected_idx" in uid:
            lcd_ui.edit_selected_idx = int(uid.get("edit_selected_idx", -1))
        if "edit_preset_idx" in uid:
            lcd_ui.edit_preset_idx = int(uid.get("edit_preset_idx", 0))
        ui_define_pages()
    return True


def _ensure_sd_mounted() -> bool:
    try:
        if SD_MOUNT_POINT.strip("/").split("/")[-1] in os.listdir("/"):
            return True
    except Exception:
        pass
    try:
        import busio  # type: ignore
        import digitalio  # type: ignore
        import sdcardio  # type: ignore
        import storage  # type: ignore
    except Exception:
        return False
    try:
        sck = _must_pin(LCD_SPI_SCLK_PAD, "LCD_SCLK")
        mosi = _must_pin(LCD_SPI_MOSI_PAD, "LCD_MOSI")
        miso = _must_pin(SPI_MISO_PAD, "SPI_MISO")
        cs = digitalio.DigitalInOut(_must_pin(SD_CS_PAD, "SD_CS"))
        spi = busio.SPI(sck, MOSI=mosi, MISO=miso)
        sd = sdcardio.SDCard(spi, cs)
        storage.mount(storage.VfsFat(sd), SD_MOUNT_POINT)
        return True
    except Exception:
        return False


def config_save_to_sd() -> bool:
    if not _ensure_sd_mounted():
        state.ui_status = "cfg:sd_mount_fail"
        return False
    try:
        with open(SD_CONFIG_FILE, "w") as f:
            json.dump(_build_config_doc(), f)
        state.ui_status = "cfg:saved"
        return True
    except Exception:
        state.ui_status = "cfg:save_fail"
        return False


def config_load_from_sd() -> bool:
    if not _ensure_sd_mounted():
        state.ui_status = "cfg:sd_mount_fail"
        return False
    try:
        with open(SD_CONFIG_FILE, "r") as f:
            doc = json.load(f)
    except Exception:
        state.ui_status = "cfg:load_fail"
        return False
    if not _apply_config_doc(doc):
        state.ui_status = "cfg:invalid"
        return False
    state.ui_status = "cfg:loaded"
    return True


def macro_get_key_binding(layer: int, key_index: int) -> list[MacroEvent] | None:
    return macro_store.key_bindings.get(_macro_binding_id(layer, key_index))


def macro_set_key_binding(layer: int, key_index: int, events: list[MacroEvent]) -> None:
    macro_store.key_bindings[_macro_binding_id(layer, key_index)] = list(events)


def macro_delete_key_binding(layer: int, key_index: int) -> bool:
    k = _macro_binding_id(layer, key_index)
    if k not in macro_store.key_bindings:
        return False
    del macro_store.key_bindings[k]
    return True


def macro_play_key_binding(layer: int, key_index: int) -> bool:
    events = macro_get_key_binding(layer, key_index)
    if not events:
        return False
    for ev in events:
        if ev.delay_ms > 0:
            time.sleep(ev.delay_ms / 1000.0)
        _emit_key(ev.keycode)
    return True


def _macro_token_to_keycode(token: str):
    t = token.strip().upper()
    special = {
        "SPC": KC.SPC,
        "TAB": KC.TAB,
        "ENT": KC.ENT,
        "ESC": KC.ESC,
        "BSPC": KC.BSPC,
        "DEL": KC.DEL,
        "LEFT": KC.LEFT,
        "RIGHT": KC.RIGHT,
        "UP": KC.UP,
        "DOWN": KC.DOWN,
        "CTRL": KC.LCTL,
        "ALT": KC.LALT,
        "GUI": KC.LGUI,
        "SHIFT": KC.LSFT,
        "FN": KC.TRNS,
    }
    if t in special:
        return special[t]
    if len(t) == 1 and "A" <= t <= "Z":
        return getattr(KC, t, None)
    if len(t) == 1 and "0" <= t <= "9":
        return getattr(KC, f"N{t}", None)
    if t.startswith("F") and t[1:].isdigit():
        return getattr(KC, t, None)
    return None


def _macro_program_reset() -> None:
    macro_program.active = False
    macro_program.selected_layer = state.layer
    macro_program.selected_key = -1
    macro_program.wait_for_key = False
    macro_program.key_mode = "keys"
    macro_program.key_tokens.clear()
    macro_program.string_value = ""


def macro_program_start() -> None:
    _macro_program_reset()
    macro_program.active = True
    macro_program.wait_for_key = True
    state.ui_page = "macro_prog_pick"
    state.ui_status = "macro:press_key"


def macro_program_begin_manual_pick(layer: int) -> None:
    macro_program.wait_for_key = False
    macro_program.selected_layer = layer
    state.ui_page = "macro_prog_grid"
    state.ui_status = f"macro:pick_l{layer + 1}"


def macro_program_choose_key(layer: int, key_index: int) -> bool:
    if key_index == 1:
        state.ui_status = "macro:sw1_locked"
        return False
    macro_program.selected_layer = int(layer)
    macro_program.selected_key = int(key_index)
    existing = macro_get_key_binding(layer, key_index)
    if existing:
        state.ui_page = "macro_prog_overwrite"
        state.ui_status = f"macro:exists_l{layer + 1}k{key_index}"
        return True
    macro_program.key_mode = "keys"
    macro_program.key_tokens.clear()
    macro_program.string_value = ""
    state.ui_page = "macro_prog_keys"
    state.ui_status = f"macro:new_l{layer + 1}k{key_index}"
    return True


def macro_program_handle_physical_key(key_index: int) -> bool:
    if not (macro_program.active and macro_program.wait_for_key):
        return False
    return macro_program_choose_key(state.layer, key_index)


def macro_program_save() -> bool:
    layer = macro_program.selected_layer
    key = macro_program.selected_key
    if key == 1 or key < 1:
        state.ui_status = "macro:no_target"
        return False
    events: list[MacroEvent] = []
    if macro_program.key_mode == "string":
        text = macro_program.string_value.strip()
        if not text:
            state.ui_status = "macro:string_empty"
            return False
        events.append(MacroEvent(keycode=("STRING", text), delay_ms=0))
    else:
        for token in macro_program.key_tokens:
            kc = _macro_token_to_keycode(token)
            if kc is None:
                continue
            events.append(MacroEvent(keycode=kc, delay_ms=0))
        if not events:
            state.ui_status = "macro:keys_empty"
            return False
    macro_set_key_binding(layer, key, events)
    state.ui_status = f"macro:saved_l{layer + 1}k{key}"
    _macro_program_reset()
    state.ui_page = "macro"
    return True


def macro_start_record(slot: int) -> bool:
    if slot < 0 or slot >= MACRO_MAX_SLOTS:
        return False
    state.macro_recording = True
    state.macro_record_slot = slot
    macro_store._record_buf.clear()
    macro_store._record_last_at = time.monotonic()
    return True


def macro_record_key(keycode) -> None:
    if not state.macro_recording:
        return
    now = time.monotonic()
    delay_ms = int((now - macro_store._record_last_at) * 1000)
    macro_store._record_last_at = now
    macro_store._record_buf.append(MacroEvent(keycode=keycode, delay_ms=max(0, delay_ms)))


def macro_stop_record() -> bool:
    slot = state.macro_record_slot
    if not state.macro_recording or slot < 0:
        return False
    macro_store.slots[slot] = list(macro_store._record_buf)
    state.macro_recording = False
    state.macro_record_slot = -1
    macro_store._record_buf.clear()
    return True


def macro_play(slot: int) -> bool:
    events = macro_store.slots.get(slot)
    if not events:
        return False
    for ev in events:
        if ev.delay_ms > 0:
            time.sleep(ev.delay_ms / 1000.0)
        _emit_key(ev.keycode)
    return True


def pomodoro_start(work_min: int = POMODORO_WORK_MIN, break_min: int = POMODORO_BREAK_MIN):
    pomodoro.work_s = int(work_min * 60)
    pomodoro.break_s = int(break_min * 60)
    pomodoro.remaining_s = pomodoro.work_s
    pomodoro.on_break = False
    pomodoro.running = True
    pomodoro._last_tick = time.monotonic()


def pomodoro_stop():
    pomodoro.running = False


def pomodoro_tick():
    if not pomodoro.running:
        return
    now = time.monotonic()
    elapsed = int(now - pomodoro._last_tick)
    if elapsed <= 0:
        return
    pomodoro._last_tick = now
    pomodoro.remaining_s -= elapsed
    if pomodoro.remaining_s > 0:
        return
    pomodoro.on_break = not pomodoro.on_break
    pomodoro.remaining_s = pomodoro.break_s if pomodoro.on_break else pomodoro.work_s
    # Transition cue: use power pixel as a quick notifier.
    r, g, b = capped_rgb(70, 70, 0 if pomodoro.on_break else 70)
    rgb.set_rgb(0, r, g, b)


def _ir_scene_run_step(step: IRSceneStep) -> None:
    if step.kind == "ir":
        ir_send_slot(int(step.value))
    elif step.kind == "delay_ms":
        time.sleep(float(step.value) / 1000.0)
    elif step.kind == "macro":
        macro_play(int(step.value))
    elif step.kind == "layer":
        set_active_layer(int(step.value))


def ir_scene_define(name: str, steps: list[IRSceneStep]) -> None:
    ir_scenes.scenes[name] = steps


def ir_scene_run(name: str) -> bool:
    steps = ir_scenes.scenes.get(name)
    if not steps:
        return False
    for step in steps:
        _ir_scene_run_step(step)
    return True


def _webui_snapshot() -> dict:
    ui_custom: dict[str, list[dict]] = {}
    for page, icons in lcd_ui.custom_buttons.items():
        ui_custom[page] = [_icon_to_doc(i) for i in icons]
    return {
        "layer": state.layer,
        "hall_mode": state.hall_mode,
        "power_status": state.power_status,
        "battery_percent": round(state.battery_percent, 1),
        "battery_voltage": round(state.battery_voltage, 3),
        "macro_slots": sorted(list(macro_store.slots.keys())),
        "ir_slots": sorted(list(ir_store.slots.keys())),
        "ir_scenes": sorted(list(ir_scenes.scenes.keys())),
        "ui_page": state.ui_page,
        "ui_status": state.ui_status,
        "ui_battery_badge_pct": lcd_ui.battery_badge_pct,
        "ui_keylist_layer": lcd_ui.keylist_layer,
        "ui_keylist_offset": lcd_ui.keylist_offset,
        "ui_keylist_rows": ui_keylist_visible_rows(),
        "ui_nav_order": list(lcd_ui.nav_order),
        "ui_edit_page": lcd_ui.edit_page,
        "ui_edit_selected_idx": lcd_ui.edit_selected_idx,
        "ui_edit_preset_idx": lcd_ui.edit_preset_idx,
        "ui_custom_buttons": ui_custom,
        "context_app": state.context_app,
        "context_auto": state.context_auto,
        "wifi": {
            "connected": state.wifi_connected,
            "ssid": state.wifi_ssid,
            "ip": state.wifi_ip,
            "subnet": state.wifi_subnet,
            "gateway": state.wifi_gateway,
        },
        "toggles": {
            "rgb_dynamic": toggles.rgb_dynamic,
            "haptics_enabled": toggles.haptics_enabled,
            "focus_lock": toggles.focus_lock,
            "safe_mode": toggles.safe_mode,
        },
        "pomodoro": {
            "running": pomodoro.running,
            "on_break": pomodoro.on_break,
            "remaining_s": pomodoro.remaining_s,
        },
    }


def webui_export_json() -> str:
    return json.dumps(_webui_snapshot())


def webui_apply_json(payload: str) -> bool:
    # Minimal config loader for local web tooling.
    try:
        doc = json.loads(payload)
    except Exception:
        return False
    if "layer" in doc:
        set_active_layer(int(doc["layer"]))
    if "hall_mode" in doc:
        state.hall_mode = str(doc["hall_mode"])
    if "context_app" in doc:
        state.context_app = str(doc["context_app"])
    if "context_auto" in doc:
        state.context_auto = bool(doc["context_auto"])
    if "ui_custom_buttons" in doc and isinstance(doc["ui_custom_buttons"], dict):
        lcd_ui.custom_buttons.clear()
        for page, items in doc["ui_custom_buttons"].items():
            if not isinstance(items, list):
                continue
            icons: list[IconButton] = []
            for item in items:
                icon = _icon_from_doc(item)
                if icon is not None:
                    icons.append(icon)
            if icons:
                lcd_ui.custom_buttons[str(page)] = icons
        ui_define_pages()
    if "ui_edit_page" in doc:
        lcd_ui.edit_page = str(doc["ui_edit_page"])
    if "ui_edit_selected_idx" in doc:
        lcd_ui.edit_selected_idx = int(doc["ui_edit_selected_idx"])
    if "ui_edit_preset_idx" in doc:
        lcd_ui.edit_preset_idx = int(doc["ui_edit_preset_idx"])
    if "toggles" in doc and isinstance(doc["toggles"], dict):
        td = doc["toggles"]
        for name in ("rgb_dynamic", "haptics_enabled", "focus_lock", "safe_mode"):
            if name in td and hasattr(toggles, name):
                setattr(toggles, name, bool(td[name]))
    if "pomodoro" in doc and isinstance(doc["pomodoro"], dict):
        p = doc["pomodoro"]
        if p.get("running"):
            pomodoro_start(
                work_min=int(p.get("work_min", POMODORO_WORK_MIN)),
                break_min=int(p.get("break_min", POMODORO_BREAK_MIN)),
            )
        else:
            pomodoro_stop()
    return True


def webui_start() -> bool:
    global _web_server, _web_server_poll
    state.webui_enabled = True
    if _web_server is not None:
        return True
    try:
        import wifi  # type: ignore
        import socketpool  # type: ignore
        from adafruit_httpserver import (  # type: ignore
            GET,
            POST,
            JSONResponse,
            Request,
            Response,
            Server,
        )
    except Exception as e:
        state.ui_status = "webui:libs_missing"
        _log(f"WebUI init failed: {e}")
        return False

    ssid = os.getenv("CIRCUITPY_WIFI_SSID") or os.getenv("WIFI_SSID") or os.getenv("CP_WIFI_SSID")
    password = (
        os.getenv("CIRCUITPY_WIFI_PASSWORD")
        or os.getenv("WIFI_PASSWORD")
        or os.getenv("CP_WIFI_PASSWORD")
    )
    if not ssid or not password:
        state.ui_status = "webui:wifi_cfg_missing"
        return False
    try:
        if not getattr(wifi.radio, "ipv4_address", None):
            wifi.radio.connect(ssid, password)
        pool = socketpool.SocketPool(wifi.radio)
        server = Server(pool, debug=False)
        ip = str(wifi.radio.ipv4_address)

        @server.route("/", GET)
        def _root(request: Request):
            return Response(
                request,
                body=(
                    "Macropad WebUI\n"
                    "GET /snapshot -> runtime json\n"
                    "POST /apply -> apply json config"
                ),
                content_type="text/plain",
            )

        @server.route("/snapshot", GET)
        def _snapshot(request: Request):
            return JSONResponse(request, _webui_snapshot())

        @server.route("/apply", POST)
        def _apply(request: Request):
            try:
                payload = request.body.decode("utf-8")
            except Exception:
                payload = ""
            ok = webui_apply_json(payload)
            return JSONResponse(request, {"ok": bool(ok), "status": state.ui_status})

        server.start(ip)
        _web_server = server
        _web_server_poll = server.poll
        state.webui_url = f"http://{ip}/"
        update_wifi_status(force=True)
        ui_define_pages()
        state.ui_status = "webui:ready"
        _log(f"WebUI started at {state.webui_url}")
        return True
    except Exception as e:
        _web_server = None
        _web_server_poll = None
        state.ui_status = "webui:start_fail"
        _log(f"WebUI start failed: {e}")
        return False


def apply_runtime():
    # Call this periodically from your board loop hook.
    _run_deferred_init()
    update_power_status()
    update_wifi_status()
    ui_update_overlay_state()
    pomodoro_tick()
    sw1_tap_tick()
    ir_scan_tick()
    if callable(_web_server_poll):
        try:
            _web_server_poll()
        except Exception as e:
            state.ui_status = "webui:poll_fail"
            _log(f"WebUI poll failed: {e}")
    ui_apply_context_page()
    set_layer_color(state.layer)
    apply_power_overlay(state.power_status)
    _ = oled_render()
    filtered = update_hall_filter(read_hall_raw())
    hall_gesture_update(filtered)
    delta = hall_delta(filtered)
    if delta != 0:
        now = time.monotonic()
        if (now - state.hall_last_emit_at) >= HALL_KEY_REPEAT_S:
            state.hall_last_emit_at = now
            _emit_key(hall_to_action(delta))


def set_active_layer(layer: int):
    state.layer = layer
    set_layer_color(layer)


def init_default_ir_scenes():
    # Example "movie" automation scene.
    ir_scene_define(
        "movie",
        [
            IRSceneStep("ir", 0),          # TV power
            IRSceneStep("delay_ms", 600),
            IRSceneStep("ir", 1),          # Input select
            IRSceneStep("delay_ms", 250),
            IRSceneStep("ir", 2),          # AVR power/input
            IRSceneStep("delay_ms", 250),
            IRSceneStep("macro", 0),       # optional macro slot
            IRSceneStep("layer", MEDIA),
        ],
    )


keyboard.keymap = [
    # BASE
    [
        SW1_TD, KC.N2, KC.N3, KC.N4, KC.N5, KC.N6,
        KC.N7, KC.N8, KC.N9, KC.N0, KC.MINS, KC.EQL,
        KC.Q, KC.W, KC.E, KC.R, KC.T, KC.Y,
        KC.A, KC.S, KC.D, KC.F, KC.G, KC.H,
        KC.Z, KC.X,
    ],
    # FN
    [SW1_TD] + [KC.TRNS] * 25,
    # MEDIA
    [
        SW1_TD, L3_IR_KEYS[2], L3_IR_KEYS[3], L3_IR_KEYS[4], L3_IR_KEYS[5], L3_IR_KEYS[6],
        L3_IR_KEYS[7], L3_IR_KEYS[8], L3_IR_KEYS[9], L3_IR_KEYS[10], L3_IR_KEYS[11], L3_IR_KEYS[12],
        L3_IR_KEYS[13], L3_IR_KEYS[14], L3_IR_KEYS[15], L3_IR_KEYS[16], L3_IR_KEYS[17], L3_IR_KEYS[18],
        L3_IR_KEYS[19], L3_IR_KEYS[20], L3_IR_KEYS[21], L3_IR_KEYS[22], L3_IR_KEYS[23], L3_IR_KEYS[24],
        L3_IR_KEYS[25], L3_IR_KEYS[26],
    ],
]


if __name__ == "__main__":
    # Initial visual state.
    set_layer_color(BASE)
    apply_power_overlay("ok")
    init_default_ir_scenes()
    ui_define_pages()
    keyboard.go()
