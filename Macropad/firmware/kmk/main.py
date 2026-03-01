# Rev-B integrated firmware template (KMK-style) for Macropad.
# Update board-specific pin mappings before flashing.

from dataclasses import dataclass

from kmk.kmk_keyboard import KMKKeyboard
from kmk.keys import KC
from kmk.modules.layers import Layers
from kmk.modules.encoder import EncoderHandler
from kmk.extensions.rgb import RGB
from kmk.extensions.oled import Oled, OledData


keyboard = KMKKeyboard()
layers = Layers()
keyboard.modules.append(layers)

# Placeholder extension setup (replace with your board pin mapping).
rgb = RGB(
    pixel_pin=None,          # TODO: map to RGB_DATA GPIO
    num_pixels=26,
    val_default=28,          # conservative default brightness
)
keyboard.extensions.append(rgb)

oled = Oled(
    OledData(
        pin_sda=None,        # TODO: map to LCD_SDA GPIO
        pin_scl=None,        # TODO: map to LCD_SCL GPIO
        i2c_addr=0x3C,
    )
)
keyboard.extensions.append(oled)

encoder_handler = EncoderHandler()
keyboard.modules.append(encoder_handler)


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


@dataclass
class RuntimeState:
    layer: int = BASE
    power_status: str = "ok"        # ok | low | charging
    oled_page: int = 0
    hall_raw: int = 0
    hall_filtered: float = 0.0
    hall_mode: str = "volume"       # volume | scrub | zoom


state = RuntimeState()


def set_layer_color(layer):
    color = LAYER_COLORS.get(layer, (20, 20, 20))
    rgb.set_hsv_fill(0, 0, 0)  # reset
    rgb.set_rgb_fill(color[0], color[1], color[2])


def apply_power_overlay(status: str):
    # Keep this simple: tint key 0 as status indicator.
    if status == "ok":
        c = POWER_OK
    elif status == "low":
        c = POWER_LOW
    else:
        c = POWER_CHARGING
    rgb.set_rgb(0, c[0], c[1], c[2])


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
        return f"POWER:{state.power_status} HALL:{int(state.hall_filtered)}"
    return f"PROFILE:{state.layer} MODE:{state.hall_mode}"


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


def apply_runtime():
    # Call this periodically from your board loop hook.
    set_layer_color(state.layer)
    apply_power_overlay(state.power_status)
    _ = oled_render()
    filtered = update_hall_filter(read_hall_raw())
    delta = hall_delta(filtered)
    if delta != 0:
        _ = hall_to_action(delta)
        # TODO: send key based on action using keyboard tap API for your KMK version.


def set_active_layer(layer: int):
    state.layer = layer
    set_layer_color(layer)


keyboard.keymap = [
    # BASE
    [
        KC.N1, KC.N2, KC.N3, KC.N4, KC.N5, KC.N6,
        KC.N7, KC.N8, KC.N9, KC.N0, KC.MINS, KC.EQL,
        KC.Q, KC.W, KC.E, KC.R, KC.T, KC.Y,
        KC.A, KC.S, KC.D, KC.F, KC.G, KC.H,
        KC.Z, KC.X,
    ],
    # FN
    [KC.TRNS] * 26,
    # MEDIA
    [KC.TRNS] * 26,
]


if __name__ == "__main__":
    # Initial visual state.
    set_layer_color(BASE)
    apply_power_overlay("ok")
    keyboard.go()
