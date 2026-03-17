#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


BLOCK_RE = re.compile(r'^\t\((segment|via)\n(?:.*\n)*?\t\)\n?', re.MULTILINE)
NET_RE = re.compile(r'^\s*\(net\s+(\d+)\s+"([^"]+)"\)$', re.MULTILINE)
FLOAT_RE = r'(-?\d+(?:\.\d+)?)'
START_RE = re.compile(rf'^\s*\(start {FLOAT_RE} {FLOAT_RE}\)$', re.MULTILINE)
END_RE = re.compile(rf'^\s*\(end {FLOAT_RE} {FLOAT_RE}\)$', re.MULTILINE)
WIDTH_RE = re.compile(rf'^\s*\(width {FLOAT_RE}\)$', re.MULTILINE)
LAYER_RE = re.compile(r'^\s*\(layer "([^"]+)"\)$', re.MULTILINE)
LAYERS_RE = re.compile(r'^\s*\(layers "([^"]+)" "([^"]+)"\)$', re.MULTILINE)
AT_RE = re.compile(rf'^\s*\(at {FLOAT_RE} {FLOAT_RE}\)$', re.MULTILINE)
SIZE_RE = re.compile(rf'^\s*\(size {FLOAT_RE}\)$', re.MULTILINE)
NET_ID_RE = re.compile(r'^\s*\(net (\d+)\)$', re.MULTILINE)
UUID_RE = re.compile(r'^\s*\(uuid "([^"]+)"\)$', re.MULTILINE)


@dataclass
class Segment:
    block: str
    uuid: str
    net: int
    layer: str
    width: float
    start: tuple[float, float]
    end: tuple[float, float]
    line: int

    @property
    def diagonal(self) -> bool:
        return not (
            math.isclose(self.start[0], self.end[0], abs_tol=1e-6)
            or math.isclose(self.start[1], self.end[1], abs_tol=1e-6)
        )

    @property
    def length(self) -> float:
        return math.hypot(self.start[0] - self.end[0], self.start[1] - self.end[1])


@dataclass
class Via:
    block: str
    uuid: str
    net: int
    layers: tuple[str, str]
    at: tuple[float, float]
    size: float
    line: int


def load_blocks(text: str) -> tuple[list[Segment], list[Via], dict[int, str]]:
    net_names = {int(m.group(1)): m.group(2) for m in NET_RE.finditer(text)}
    line_starts = [0]
    for i, ch in enumerate(text):
        if ch == "\n":
            line_starts.append(i + 1)

    def line_of(offset: int) -> int:
        lo, hi = 0, len(line_starts) - 1
        while lo <= hi:
            mid = (lo + hi) // 2
            if line_starts[mid] <= offset:
                lo = mid + 1
            else:
                hi = mid - 1
        return hi + 1

    segments: list[Segment] = []
    vias: list[Via] = []
    for m in BLOCK_RE.finditer(text):
        block = m.group(0)
        kind = m.group(1)
        net = int(NET_ID_RE.search(block).group(1))
        uuid = UUID_RE.search(block).group(1)
        line = line_of(m.start())
        if kind == "segment":
            start = tuple(map(float, START_RE.search(block).groups()))
            end = tuple(map(float, END_RE.search(block).groups()))
            width = float(WIDTH_RE.search(block).group(1))
            layer = LAYER_RE.search(block).group(1)
            segments.append(
                Segment(
                    block=block,
                    uuid=uuid,
                    net=net,
                    layer=layer,
                    width=width,
                    start=start,
                    end=end,
                    line=line,
                )
            )
        else:
            at = tuple(map(float, AT_RE.search(block).groups()))
            size = float(SIZE_RE.search(block).group(1))
            layers = LAYERS_RE.search(block).groups()
            vias.append(
                Via(
                    block=block,
                    uuid=uuid,
                    net=net,
                    layers=layers,
                    at=at,
                    size=size,
                    line=line,
                )
            )
    return segments, vias, net_names


def orient(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a: tuple[float, float], b: tuple[float, float], p: tuple[float, float]) -> bool:
    return (
        min(a[0], b[0]) - 1e-6 <= p[0] <= max(a[0], b[0]) + 1e-6
        and min(a[1], b[1]) - 1e-6 <= p[1] <= max(a[1], b[1]) + 1e-6
    )


def segment_intersection(
    a1: tuple[float, float],
    a2: tuple[float, float],
    b1: tuple[float, float],
    b2: tuple[float, float],
) -> tuple[float, float] | None:
    x1, y1 = a1
    x2, y2 = a2
    x3, y3 = b1
    x4, y4 = b2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-9:
        if not (
            math.isclose(orient(a1, a2, b1), 0.0, abs_tol=1e-6)
            and math.isclose(orient(a1, a2, b2), 0.0, abs_tol=1e-6)
        ):
            return None
        points = [a1, a2, b1, b2]
        for p in points:
            if on_segment(a1, a2, p) and on_segment(b1, b2, p):
                return p
        return None
    det1 = x1 * y2 - y1 * x2
    det2 = x3 * y4 - y3 * x4
    px = (det1 * (x3 - x4) - (x1 - x2) * det2) / denom
    py = (det1 * (y3 - y4) - (y1 - y2) * det2) / denom
    p = (px, py)
    if on_segment(a1, a2, p) and on_segment(b1, b2, p):
        return p
    return None


def shared_endpoint(sa: Segment, sb: Segment) -> bool:
    pts_a = {sa.start, sa.end}
    pts_b = {sb.start, sb.end}
    for pa in pts_a:
        for pb in pts_b:
            if math.isclose(pa[0], pb[0], abs_tol=1e-6) and math.isclose(pa[1], pb[1], abs_tol=1e-6):
                return True
    return False


def bbox(seg: Segment) -> tuple[float, float, float, float]:
    return (
        min(seg.start[0], seg.end[0]),
        min(seg.start[1], seg.end[1]),
        max(seg.start[0], seg.end[0]),
        max(seg.start[1], seg.end[1]),
    )


def boxes_overlap(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> bool:
    return not (a[2] < b[0] - 1e-6 or b[2] < a[0] - 1e-6 or a[3] < b[1] - 1e-6 or b[3] < a[1] - 1e-6)


def find_crossings(segments: list[Segment]) -> list[dict[str, object]]:
    by_layer: dict[str, list[Segment]] = defaultdict(list)
    for seg in segments:
        by_layer[seg.layer].append(seg)

    crossings: list[dict[str, object]] = []
    for layer, layer_segments in by_layer.items():
        boxes = [bbox(s) for s in layer_segments]
        for i, sa in enumerate(layer_segments):
            for j in range(i + 1, len(layer_segments)):
                sb = layer_segments[j]
                if not boxes_overlap(boxes[i], boxes[j]):
                    continue
                p = segment_intersection(sa.start, sa.end, sb.start, sb.end)
                if p is None:
                    continue
                if shared_endpoint(sa, sb):
                    if (
                        math.isclose(p[0], sa.start[0], abs_tol=1e-6)
                        and math.isclose(p[1], sa.start[1], abs_tol=1e-6)
                    ) or (
                        math.isclose(p[0], sa.end[0], abs_tol=1e-6)
                        and math.isclose(p[1], sa.end[1], abs_tol=1e-6)
                    ):
                        continue
                crossings.append(
                    {
                        "layer": layer,
                        "a_uuid": sa.uuid,
                        "b_uuid": sb.uuid,
                        "a_line": sa.line,
                        "b_line": sb.line,
                        "a_net": sa.net,
                        "b_net": sb.net,
                        "point": [round(p[0], 4), round(p[1], 4)],
                    }
                )
    return crossings


def region_filter(segments: list[Segment], x0: float, x1: float, y0: float, y1: float) -> list[Segment]:
    out = []
    for seg in segments:
        sx0, sy0, sx1, sy1 = bbox(seg)
        if sx1 < x0 or sx0 > x1 or sy1 < y0 or sy0 > y1:
            continue
        out.append(seg)
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("board", nargs="?", default="Macropad.kicad_pcb")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--region", nargs=4, type=float, metavar=("X0", "X1", "Y0", "Y1"))
    parser.add_argument("--list-diagonals", action="store_true")
    parser.add_argument("--list-crossings", action="store_true")
    args = parser.parse_args()

    text = Path(args.board).read_text()
    segments, vias, net_names = load_blocks(text)
    chosen = segments
    if args.region:
        x0, x1, y0, y1 = args.region
        chosen = region_filter(segments, x0, x1, y0, y1)

    diagonals = [s for s in chosen if s.diagonal]
    crossings = find_crossings(chosen)

    if args.json:
        print(
            json.dumps(
                {
                    "segments": len(chosen),
                    "vias": len(vias),
                    "diagonals": [
                        {
                            "line": s.line,
                            "uuid": s.uuid,
                            "net": s.net,
                            "net_name": net_names.get(s.net, ""),
                            "layer": s.layer,
                            "start": s.start,
                            "end": s.end,
                            "length": round(s.length, 4),
                        }
                        for s in diagonals
                    ],
                    "crossings": crossings,
                },
                indent=2,
            )
        )
        return

    print(f"segments={len(chosen)} vias={len(vias)} diagonals={len(diagonals)} crossings={len(crossings)}")
    if args.list_diagonals:
        for s in sorted(diagonals, key=lambda item: (item.line, item.net)):
            print(
                f"diag line={s.line} uuid={s.uuid} net={s.net} name={net_names.get(s.net, '')} "
                f"layer={s.layer} start={s.start} end={s.end} len={s.length:.3f}"
            )
    if args.list_crossings:
        for item in crossings:
            print(
                f"cross layer={item['layer']} point={tuple(item['point'])} "
                f"a=line{item['a_line']}/net{item['a_net']} b=line{item['b_line']}/net{item['b_net']}"
            )


if __name__ == "__main__":
    main()
