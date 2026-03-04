#!/usr/bin/env python3
from PIL import Image, ImageDraw
import numpy as np


SRC = "Macropad/2090-15.jpg"
DST = "Macropad/pins_to_holes_2090.png"


# Board pin centers in left-to-right order shown on 2090-15 (full-res 2342x1496).
board_x = [570, 627, 683, 740, 797, 853, 908, 962, 1046, 1103, 1159, 1214, 1270]
board_y = 943

# OLED left header hole centers, top-to-bottom (full-res 2342x1496).
oled_x = 681
oled_y = [278, 305, 331, 358, 385, 411, 438, 465, 492, 518, 545, 572, 598]


def cleanup_old_lines(img: Image.Image) -> Image.Image:
    arr = np.array(img.convert("RGB"))
    r = arr[:, :, 0]
    g = arr[:, :, 1]
    b = arr[:, :, 2]

    # Remove old user-drawn bright lines from the left/OLED/board-guide area.
    roi = np.zeros(r.shape, dtype=bool)
    roi[:1100, :1500] = True

    red_like = (r > 165) & (g < 120) & (b < 120)
    blue_like = (b > 150) & (r < 130) & (g < 170)
    green_like = (g > 165) & (r < 140) & (b < 140)
    yellow_like = (r > 170) & (g > 170) & (b < 140)
    orange_like = (r > 180) & (g > 90) & (g < 190) & (b < 120)
    white_like = (r > 220) & (g > 220) & (b > 220)
    color_mask = red_like | blue_like | green_like | yellow_like | orange_like | white_like
    mask = roi & color_mask

    out = arr.copy()
    h, w, _ = out.shape
    ys, xs = np.where(mask)
    for y, x in zip(ys.tolist(), xs.tolist()):
        y0 = max(0, y - 4)
        y1 = min(h, y + 5)
        x0 = max(0, x - 4)
        x1 = min(w, x + 5)
        patch = arr[y0:y1, x0:x1]
        patch_mask = mask[y0:y1, x0:x1]
        keep = patch[~patch_mask]
        if keep.size:
            out[y, x] = np.median(keep, axis=0).astype(np.uint8)
    return Image.fromarray(out, mode="RGB")


def main() -> None:
    base = Image.open(SRC)
    img = base.convert("RGBA")
    draw = ImageDraw.Draw(img, "RGBA")

    palette = [
        (0, 225, 255, 255),
        (255, 80, 80, 255),
        (80, 255, 120, 255),
        (255, 210, 60, 255),
        (190, 130, 255, 255),
        (255, 140, 40, 255),
        (80, 170, 255, 255),
        (255, 80, 190, 255),
        (120, 255, 255, 255),
        (255, 120, 120, 255),
        (120, 255, 160, 255),
        (255, 220, 120, 255),
        (170, 170, 255, 255),
    ]
    shadow = (0, 0, 0, 120)

    # One line = one board pin to one OLED hole.
    for i, (x0, y1) in enumerate(zip(board_x, oled_y)):
        color = palette[i % len(palette)]
        # subtle shadow for visibility on mixed background
        draw.line([(x0 + 1, board_y + 1), (oled_x + 1, y1 + 1)], fill=shadow, width=4)
        draw.line([(x0, board_y), (oled_x, y1)], fill=color, width=3)
        draw.ellipse((x0 - 2, board_y - 2, x0 + 2, board_y + 2), fill=color)
        draw.ellipse((oled_x - 2, y1 - 2, oled_x + 2, y1 + 2), fill=color)

    img.convert("RGB").save(DST, quality=95)


if __name__ == "__main__":
    main()
