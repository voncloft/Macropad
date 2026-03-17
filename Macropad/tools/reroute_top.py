#!/usr/bin/env python3

from __future__ import annotations

import argparse
from collections import defaultdict

import pcbnew


F_CU = pcbnew.F_Cu
B_CU = pcbnew.B_Cu
IN1_CU = pcbnew.In1_Cu
IN2_CU = pcbnew.In2_Cu

REGION = (-5.0, 111.25, -41.0, 40.5)
RIGHT_GND_X = 106.5
RIGHT_5V_X = 108.0
RIGHT_BOOST_X = 109.5


def mm(value: float) -> int:
    return pcbnew.FromMM(value)


def pt(x: float, y: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(mm(x), mm(y))


def close(a: float, b: float, tol: float = 0.01) -> bool:
    return abs(a - b) <= tol


def bbox_intersects(item: pcbnew.BOARD_ITEM, region: tuple[float, float, float, float]) -> bool:
    bb = item.GetBoundingBox()
    left = pcbnew.ToMM(bb.GetLeft())
    right = pcbnew.ToMM(bb.GetRight())
    top = pcbnew.ToMM(bb.GetTop())
    bottom = pcbnew.ToMM(bb.GetBottom())
    return not (right < region[0] or left > region[1] or bottom < region[2] or top > region[3])


def is_track(item: pcbnew.BOARD_ITEM) -> bool:
    return item.GetClass() == "PCB_TRACK"


def is_via(item: pcbnew.BOARD_ITEM) -> bool:
    return item.GetClass() == "PCB_VIA"


def has_through_hole_pad(board: pcbnew.BOARD, net: int, x: float, y: float) -> bool:
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            if pad.GetNetCode() != net:
                continue
            if pad.GetDrillSizeX() <= 0 and pad.GetDrillSizeY() <= 0:
                continue
            cx = pcbnew.ToMM(pad.GetCenter().x)
            cy = pcbnew.ToMM(pad.GetCenter().y)
            if close(cx, x) and close(cy, y):
                return True
    return False


def has_same_net_via(board: pcbnew.BOARD, net: int, x: float, y: float) -> bool:
    for item in board.GetTracks():
        if not is_via(item) or item.GetNetCode() != net:
            continue
        pos = item.GetPosition()
        if close(pcbnew.ToMM(pos.x), x) and close(pcbnew.ToMM(pos.y), y):
            return True
    return False


def keep_net22_grid(item: pcbnew.BOARD_ITEM) -> bool:
    width = round(pcbnew.ToMM(item.GetWidth()), 3) if is_track(item) else None
    bb = item.GetBoundingBox()
    left = pcbnew.ToMM(bb.GetLeft())
    right = pcbnew.ToMM(bb.GetRight())
    top = pcbnew.ToMM(bb.GetTop())
    return left >= -1.0 and right <= 85.0 and top >= 12.5 and (width == 0.12 or is_via(item))


def keep_net23_grid(item: pcbnew.BOARD_ITEM) -> bool:
    width = round(pcbnew.ToMM(item.GetWidth()), 3) if is_track(item) else None
    bb = item.GetBoundingBox()
    left = pcbnew.ToMM(bb.GetLeft())
    right = pcbnew.ToMM(bb.GetRight())
    top = pcbnew.ToMM(bb.GetTop())
    return left >= -1.0 and right <= 88.5 and top >= 16.0 and (width == 0.12 or is_via(item))


ROW_NETS = {1, 2, 3, 4, 5, 6}


def remove_top_items(board: pcbnew.BOARD) -> None:
    reroute_nets = {
        7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
        50, 51, 52, 53, 54, 55, 56, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100,
    }
    to_remove: list[pcbnew.BOARD_ITEM] = []
    for item in list(board.GetTracks()):
        net = item.GetNetCode()
        if net not in reroute_nets:
            continue
        if net not in {7, 8, 9, 10, 11} and not bbox_intersects(item, REGION):
            continue
        if net == 22 and keep_net22_grid(item):
            continue
        if net == 23 and keep_net23_grid(item):
            continue
        to_remove.append(item)
    for item in to_remove:
        board.Remove(item)


def remove_row_items(board: pcbnew.BOARD) -> None:
    to_remove: list[pcbnew.BOARD_ITEM] = []
    for item in list(board.GetTracks()):
        if item.GetNetCode() in ROW_NETS:
            to_remove.append(item)
    for item in to_remove:
        board.Remove(item)


def add_track(board: pcbnew.BOARD, net: int, layer: int, width: float, a: tuple[float, float], b: tuple[float, float]) -> None:
    if round(a[0], 4) == round(b[0], 4) and round(a[1], 4) == round(b[1], 4):
        return
    track = pcbnew.PCB_TRACK(board)
    track.SetNetCode(net)
    track.SetLayer(layer)
    track.SetWidth(mm(width))
    track.SetStart(pt(*a))
    track.SetEnd(pt(*b))
    board.Add(track)


def add_path(board: pcbnew.BOARD, net: int, layer: int, width: float, points: list[tuple[float, float]]) -> None:
    for a, b in zip(points, points[1:]):
        add_track(board, net, layer, width, a, b)


def add_via(board: pcbnew.BOARD, net: int, x: float, y: float, size: float = 0.8, drill: float = 0.4) -> None:
    # Skip redundant drills on existing vias and plated through-hole pads.
    if has_same_net_via(board, net, x, y) or has_through_hole_pad(board, net, x, y):
        return
    via = pcbnew.PCB_VIA(board)
    via.SetNetCode(net)
    via.SetPosition(pt(x, y))
    via.SetWidth(mm(size))
    via.SetDrill(mm(drill))
    via.SetLayerPair(F_CU, B_CU)
    board.Add(via)


def route_to_vertical_trunk(
    board: pcbnew.BOARD,
    net: int,
    src: tuple[float, float],
    trunk_x: float,
    dst: tuple[float, float],
    width: float,
) -> None:
    add_via(board, net, src[0], src[1])
    add_path(board, net, IN1_CU, width, [src, (trunk_x, src[1])])
    add_via(board, net, trunk_x, src[1])
    add_path(board, net, IN2_CU, width, [(trunk_x, src[1]), (trunk_x, dst[1])])
    add_via(board, net, trunk_x, dst[1])
    add_path(board, net, F_CU, width, [(trunk_x, dst[1]), dst])


def route_escape_lane_to_pad(
    board: pcbnew.BOARD,
    net: int,
    src: tuple[float, float],
    lane_y: float,
    dst: tuple[float, float],
    width: float,
) -> None:
    src_entry = src
    if not close(src[1], lane_y):
        add_path(board, net, F_CU, width, [src, (src[0], lane_y)])
        src_entry = (src[0], lane_y)
    add_via(board, net, src_entry[0], src_entry[1])
    add_path(board, net, IN1_CU, width, [src_entry, (dst[0], lane_y)])
    add_via(board, net, dst[0], lane_y)
    add_path(board, net, IN2_CU, width, [(dst[0], lane_y), dst])


def route_escape_lane_to_fcu(
    board: pcbnew.BOARD,
    net: int,
    src: tuple[float, float],
    lane_y: float,
    dst: tuple[float, float],
    width: float,
) -> None:
    src_entry = src
    if not close(src[1], lane_y):
        add_path(board, net, F_CU, width, [src, (src[0], lane_y)])
        src_entry = (src[0], lane_y)
    add_via(board, net, src_entry[0], src_entry[1])
    add_path(board, net, IN1_CU, width, [src_entry, (dst[0], lane_y)])
    add_via(board, net, dst[0], lane_y)
    add_path(board, net, IN2_CU, width, [(dst[0], lane_y), (dst[0], dst[1])])
    add_via(board, net, dst[0], dst[1])


def remove_rgb_orphan_stubs(board: pcbnew.BOARD) -> None:
    to_remove: list[pcbnew.BOARD_ITEM] = []
    for item in list(board.GetTracks()):
        if not is_track(item):
            continue
        if item.GetNetCode() == 22:
            start = (round(pcbnew.ToMM(item.GetStartX()), 3), round(pcbnew.ToMM(item.GetStartY()), 3))
            end = (round(pcbnew.ToMM(item.GetEndX()), 3), round(pcbnew.ToMM(item.GetEndY()), 3))
            pts = {start, end}
            if pts == {(0.5, 13.0), (22.25, 13.0)}:
                to_remove.append(item)
        if item.GetNetCode() == 23:
            start = (round(pcbnew.ToMM(item.GetStartX()), 3), round(pcbnew.ToMM(item.GetStartY()), 3))
            end = (round(pcbnew.ToMM(item.GetEndX()), 3), round(pcbnew.ToMM(item.GetEndY()), 3))
            pts = {start, end}
            if pts == {(1.5, 18.0), (22.25, 18.0)}:
                to_remove.append(item)
    for item in to_remove:
        board.Remove(item)


def route_rows(board: pcbnew.BOARD) -> None:
    # ROW0 only serves the encoder switch, so it is a short local rail rather than a full matrix row.
    add_path(board, 1, F_CU, 0.3, [(1.965, 6.985), (55.5, 6.985)])
    add_track(board, 1, F_CU, 0.25, (66.5, -24.45), (55.5, -24.45))
    add_via(board, 1, 55.5, -24.45)
    add_path(board, 1, IN2_CU, 0.3, [(55.5, -24.45), (55.5, 6.985)])
    add_via(board, 1, 55.5, 6.985)

    row_defs = [
        (2, (66.5, -23.18), 54.0, 30.797),
        (3, (66.5, -21.91), 52.5, 49.848),
        (4, (66.5, -20.64), 51.0, 68.898),
        (5, (66.5, -14.29), 49.5, 87.947),
    ]
    for net, src, trunk_x, rail_y in row_defs:
        add_path(board, net, F_CU, 0.3, [(5.715, rail_y), (81.915, rail_y)])
        add_track(board, net, F_CU, 0.25, src, (trunk_x, src[1]))
        add_via(board, net, trunk_x, src[1])
        add_path(board, net, IN2_CU, 0.3, [(trunk_x, src[1]), (trunk_x, rail_y)])
        add_via(board, net, trunk_x, rail_y)

    add_path(board, 6, F_CU, 0.3, [(5.715, 106.998), (81.915, 106.998)])
    add_track(board, 6, F_CU, 0.25, (70.805, -10.5), (70.805, -11.0))
    add_via(board, 6, 70.805, -11.0)
    add_path(board, 6, IN1_CU, 0.3, [(70.805, -11.0), (47.0, -11.0)])
    add_via(board, 6, 47.0, -11.0)
    add_path(board, 6, IN2_CU, 0.3, [(47.0, -11.0), (47.0, 106.998)])
    add_via(board, 6, 47.0, 106.998)


def route_columns(board: pcbnew.BOARD) -> None:
    cols = [
        (7, 72.075, -20.0, 9.405),
        (8, 73.345, -18.0, 28.455),
        (9, 74.615, -16.0, 47.505),
        (10, 75.885, -14.0, 66.555),
        (11, 77.155, -12.0, 85.605),
    ]
    for net, sx, escape_y, trunk_x in cols:
        add_track(board, net, F_CU, 0.25, (sx, -10.5), (sx, -11.0))
        add_via(board, net, sx, -11.0)
        add_path(board, net, IN2_CU, 0.3, [(sx, -11.0), (sx, escape_y)])
        add_via(board, net, sx, escape_y)
        add_path(board, net, IN1_CU, 0.3, [(sx, escape_y), (trunk_x, escape_y)])
        add_via(board, net, trunk_x, escape_y)
        add_path(board, net, IN2_CU, 0.3, [(trunk_x, escape_y), (trunk_x, 116.038)])


def route_mcu_signals(board: pcbnew.BOARD) -> None:
    bottom_bcu = [
        (98, (69.535, -10.5), -12.5, (58.12, -6.5), 0.25),
        (88, (79.695, -10.5), -15.0, (55.58, -6.5), 0.25),
        (89, (80.965, -10.5), -16.5, (38.0, -6.5), 0.25),
        (90, (82.235, -10.5), -18.7, (53.04, -6.5), 0.25),
    ]
    for net, src, lane_y, dst, width in bottom_bcu:
        route_escape_lane_to_pad(board, net, src, lane_y, dst, width)

    vertical_routes = [
        (14, (84.0, -14.29), 91.175, (91.175, -23.0), 0.25),
        (15, (84.0, -13.02), 98.25, (98.25, -30.54), 0.25),
        (16, (84.0, -19.37), 40.42, (40.42, -6.5), 0.25),
        (17, (84.0, -20.64), 42.96, (42.96, -6.5), 0.25),
        (18, (84.0, -15.56), 5.5, (5.5, -2.5), 0.25),
        (19, (84.0, -16.83), 10.5, (10.5, -2.5), 0.25),
        (20, (84.0, -18.1), 7.0, (5.5, -17.5), 0.25),
        (84, (84.0, -23.18), 35.46, (35.46, -6.5), 0.25),
        (85, (84.0, -24.45), 32.92, (32.92, -6.5), 0.25),
    ]
    for net, src, trunk_x, dst, width in vertical_routes:
        route_to_vertical_trunk(board, net, src, trunk_x, dst, width)

    route_escape_lane_to_pad(board, 91, (84.0, -11.75), -19.5, (50.5, -6.5), 0.25)

    route_escape_lane_to_pad(board, 86, (84.0, -26.5), -26.5, (30.38, -6.5), 0.25)

    route_escape_lane_to_pad(board, 87, (84.0, -27.75), -27.75, (60.66, -6.5), 0.25)

    route_escape_lane_to_fcu(board, 12, (78.425, -10.5), -13.5, (78.123, -3.0), 0.25)

    route_to_vertical_trunk(board, 99, (66.5, -13.02), 48.6, (50.375, -37.0), 0.2)
    add_track(board, 99, F_CU, 0.2, (50.375, -37.0), (50.375, -39.08))

    route_to_vertical_trunk(board, 100, (66.5, -11.75), 53.6, (51.875, -35.8), 0.2)
    add_track(board, 100, F_CU, 0.2, (51.875, -35.8), (51.875, -39.08))

    add_path(board, 93, F_CU, 0.25, [(101.188, -24.54), (101.188, -24.0), (104.5, -24.0), (104.5, -24.54), (104.0, -24.54)])
    add_path(board, 94, F_CU, 0.25, [(97.438, -24.0), (97.438, -25.2), (103.062, -25.2), (103.062, -24.54)])


def route_led_and_ir(board: pcbnew.BOARD) -> None:
    add_via(board, 13, 79.998, -3.0)
    add_path(board, 13, IN1_CU, 0.18, [(79.998, -3.0), (2.5, -3.0)])
    add_via(board, 13, 32.0, -3.0)
    add_path(board, 13, IN2_CU, 0.18, [(32.0, -3.0), (32.0, -38.0)])
    add_via(board, 13, 2.5, -3.0)
    add_path(board, 13, IN2_CU, 0.18, [(2.5, -3.0), (2.5, 16.375)])
    add_via(board, 13, 2.5, 16.375)
    add_track(board, 13, F_CU, 0.18, (2.5, 16.375), (7.975, 16.375))

    add_track(board, 92, F_CU, 0.25, (92.825, -23.0), (92.825, -25.8))
    add_via(board, 92, 92.825, -25.8)
    add_track(board, 92, IN1_CU, 0.25, (92.825, -25.8), (96.0, -25.8))
    add_via(board, 92, 96.0, -25.8)
    add_path(board, 92, F_CU, 0.25, [(96.0, -25.8), (96.0, -25.3), (95.562, -25.3), (95.562, -24.95)])


def route_3v3(board: pcbnew.BOARD) -> None:
    add_track(board, 21, F_CU, 0.3, (18.19, -38.0), (18.19, -28.5))
    add_via(board, 21, 18.19, -28.5)

    add_track(board, 21, F_CU, 0.3, (48.04, -6.5), (48.04, -28.5))
    add_via(board, 21, 48.04, -28.5)

    add_path(board, 21, F_CU, 0.3, [(66.5, -26.99), (67.8, -26.99), (67.8, -27.825)])
    add_path(board, 21, B_CU, 0.3, [(67.5, -27.825), (67.8, -27.825)])
    add_via(board, 21, 67.8, -27.825)

    add_path(board, 21, F_CU, 0.3, [(66.5, -25.5), (67.6, -25.5), (67.6, -24.6), (69.0, -24.6)])
    add_via(board, 21, 69.0, -24.6)

    add_path(board, 21, IN1_CU, 0.3, [(18.19, -28.5), (67.8, -28.5), (93.0, -28.5)])
    add_path(board, 21, IN1_CU, 0.3, [(67.8, -27.825), (67.8, -28.5)])
    add_path(board, 21, IN2_CU, 0.3, [(69.0, -24.6), (69.0, -28.5)])
    add_via(board, 21, 69.0, -28.5)
    add_path(board, 21, IN1_CU, 0.3, [(69.0, -28.5), (67.8, -28.5)])
    add_via(board, 21, 93.0, -28.5)
    add_path(board, 21, IN2_CU, 0.3, [(93.0, -28.5), (93.0, -14.0)])
    add_via(board, 21, 93.0, -14.0)
    add_path(board, 21, F_CU, 0.3, [(93.0, -14.0), (96.1, -14.0), (102.4, -14.0)])
    add_path(board, 21, IN1_CU, 0.3, [(93.0, -28.5), (99.5, -28.5)])
    add_via(board, 21, 99.5, -28.5)
    add_path(board, 21, IN2_CU, 0.3, [(99.5, -28.5), (99.5, -25.46)])
    add_via(board, 21, 99.5, -25.46)
    add_path(board, 21, F_CU, 0.3, [(99.5, -25.46), (98.25, -25.46)])


def route_5v(board: pcbnew.BOARD) -> None:
    add_path(board, 22, F_CU, 0.3, [(0.5, 13.0), (4.0, 13.0)])
    add_via(board, 22, 4.0, 13.0)
    add_path(board, 22, IN2_CU, 0.3, [(4.0, 13.0), (4.0, -32.0)])
    add_via(board, 22, 4.0, -32.0)

    for x in (20.73, 34.54, 55.945):
        top = -39.08 if close(x, 55.945) else -38.0
        add_path(board, 22, F_CU, 0.3, [(x, top), (x, -32.0)])
        add_via(board, 22, x, -32.0)

    add_path(board, 22, IN1_CU, 0.3, [(4.0, -32.0), (RIGHT_5V_X, -32.0)])
    add_via(board, 22, RIGHT_5V_X, -32.0)
    add_path(board, 22, IN2_CU, 0.3, [(RIGHT_5V_X, -32.0), (RIGHT_5V_X, 35.2)])

    branch_points = [
        (-19.46, [(104.0, -19.46), (RIGHT_5V_X, -19.46)]),
        (-11.7, [(96.1, -11.7), (RIGHT_5V_X, -11.7)]),
        (5.5, [(99.25, 5.0), (99.25, 5.5), (RIGHT_5V_X, 5.5)]),
        (7.5, [(98.65, 7.0), (98.65, 6.6), (99.0, 6.6), (99.0, 7.5), (RIGHT_5V_X, 7.5)]),
        (23.2, [(92.175, 24.0), (92.175, 23.2), (RIGHT_5V_X, 23.2)]),
        (32.2, [(99.175, 33.0), (99.175, 32.2), (RIGHT_5V_X, 32.2)]),
        (35.2, [(95.175, 36.0), (95.175, 35.2), (RIGHT_5V_X, 35.2)]),
    ]
    for y, points in branch_points:
        add_via(board, 22, RIGHT_5V_X, y)
        add_path(board, 22, F_CU, 0.3, points)


def route_battery_and_boost(board: pcbnew.BOARD) -> None:
    # BAT+
    add_path(board, 50, F_CU, 0.3, [(4.0, -38.0), (4.0, -31.0)])
    add_via(board, 50, 4.0, -31.0)
    add_path(board, 50, F_CU, 0.3, [(14.54, -37.325), (16.5, -37.325), (16.5, -31.0)])
    add_via(board, 50, 16.5, -31.0)
    add_path(board, 50, F_CU, 0.3, [(25.81, -38.0), (25.81, -31.0)])
    add_via(board, 50, 25.81, -31.0)
    add_path(board, 50, IN1_CU, 0.3, [(4.0, -31.0), (16.5, -31.0), (25.81, -31.0), (103.5, -31.0)])
    add_via(board, 50, 103.5, -31.0)
    add_path(board, 50, IN2_CU, 0.3, [(103.5, -31.0), (103.5, 31.4)])
    add_via(board, 50, 103.5, 12.0)
    add_path(board, 50, F_CU, 0.3, [(98.138, 12.0), (103.5, 12.0)])
    add_via(board, 50, 103.5, 13.45)
    add_path(board, 50, F_CU, 0.3, [(98.138, 12.95), (98.138, 13.45), (103.5, 13.45)])
    add_via(board, 50, 103.5, 26.2)
    add_path(board, 50, F_CU, 0.3, [(93.175, 27.0), (93.175, 26.2), (103.5, 26.2)])
    add_via(board, 50, 103.5, 31.4)
    add_path(board, 50, F_CU, 0.3, [(95.175, 33.0), (95.175, 31.4), (103.5, 31.4)])

    # BOOST_SW
    add_path(board, 54, F_CU, 0.3, [(8.0, -38.0), (8.0, -30.0)])
    add_via(board, 54, 8.0, -30.0)
    add_path(board, 54, IN1_CU, 0.3, [(8.0, -30.0), (RIGHT_BOOST_X, -30.0)])
    add_via(board, 54, RIGHT_BOOST_X, -30.0)
    add_path(board, 54, IN2_CU, 0.3, [(RIGHT_BOOST_X, -30.0), (RIGHT_BOOST_X, 11.05)])
    add_via(board, 54, RIGHT_BOOST_X, 6.2)
    add_path(board, 54, F_CU, 0.3, [(95.35, 7.0), (95.35, 6.2), (RIGHT_BOOST_X, 6.2)])
    add_via(board, 54, RIGHT_BOOST_X, 10.6)
    add_path(board, 54, F_CU, 0.3, [(95.862, 11.05), (95.862, 10.6), (RIGHT_BOOST_X, 10.6)])
    add_via(board, 54, RIGHT_BOOST_X, 11.4)
    add_path(board, 54, F_CU, 0.3, [(98.138, 11.05), (98.138, 11.4), (RIGHT_BOOST_X, 11.4)])

    # BOOST_FB
    add_via(board, 55, 95.862, 12.95)
    add_path(board, 55, B_CU, 0.25, [(95.862, 12.95), (95.862, 39.0)])
    add_via(board, 55, 95.862, 36.0)
    add_path(board, 55, F_CU, 0.25, [(95.862, 36.0), (96.825, 36.0)])
    add_via(board, 55, 95.862, 39.0)
    add_path(board, 55, F_CU, 0.25, [(95.862, 39.0), (95.175, 39.0)])

    # CHG_PROG
    add_path(board, 53, F_CU, 0.25, [(91.138, 21.05), (91.138, 20.5), (96.175, 20.5), (96.175, 22.0)])


def route_usb_and_battery(board: pcbnew.BOARD) -> None:
    # BAT_RAW
    add_path(board, 95, F_CU, 0.3, [(12.0, -38.0), (12.0, -29.0)])
    add_via(board, 95, 12.0, -29.0)
    add_path(board, 95, IN1_CU, 0.3, [(12.0, -29.0), (100.0, -29.0)])
    add_via(board, 95, 100.0, -29.0)
    add_path(board, 95, IN2_CU, 0.3, [(100.0, -29.0), (100.0, 24.0)])
    add_via(board, 95, 100.0, 24.0)
    add_path(board, 95, IN1_CU, 0.3, [(100.0, 24.0), (90.0, 24.0)])
    add_via(board, 95, 90.0, 24.0)
    add_path(board, 95, F_CU, 0.3, [(90.0, 24.0), (90.0, 23.4), (88.862, 23.4), (88.862, 22.95)])

    # BAT_SW
    add_path(board, 96, F_CU, 0.3, [(9.46, -35.0), (9.46, -36.5)])
    add_via(board, 96, 9.46, -36.5)
    add_path(board, 96, IN1_CU, 0.3, [(9.46, -36.5), (15.5, -36.5)])
    add_via(board, 96, 15.5, -36.5)
    add_path(board, 96, F_CU, 0.3, [(15.5, -36.5), (15.5, -35.675), (14.54, -35.675)])

    # VBUS
    add_path(board, 97, F_CU, 0.25, [(49.605, -39.08), (49.605, -38.2)])
    add_via(board, 97, 49.605, -38.2, 0.6, 0.3)
    add_path(board, 97, F_CU, 0.25, [(52.645, -39.08), (52.645, -38.2)])
    add_via(board, 97, 52.645, -38.2, 0.6, 0.3)
    add_path(board, 97, IN1_CU, 0.25, [(49.605, -38.2), (52.645, -38.2)])
    add_via(board, 97, 52.645, -29.5, 0.6, 0.3)
    add_path(board, 97, IN2_CU, 0.25, [(52.645, -38.2), (52.645, -29.5)])
    add_path(board, 97, IN1_CU, 0.25, [(52.645, -29.5), (99.0, -29.5)])
    add_via(board, 97, 99.0, -29.5)
    add_path(board, 97, IN2_CU, 0.25, [(99.0, -29.5), (99.0, 22.95)])
    add_via(board, 97, 99.0, 22.95)
    add_path(board, 97, F_CU, 0.25, [(99.0, 22.95), (91.138, 22.95)])


def route_en(board: pcbnew.BOARD) -> None:
    add_path(board, 56, F_CU, 0.25, [(66.5, -25.72), (67.5, -25.72), (67.5, -26.175)])
    add_via(board, 56, 67.5, -25.72)
    add_path(board, 56, IN2_CU, 0.25, [(67.5, -25.72), (67.5, -3.15)])
    add_via(board, 56, 67.5, -3.15)
    add_path(board, 56, B_CU, 0.25, [(67.5, -3.15), (73.7, -3.15)])


def route_usb_cc(board: pcbnew.BOARD) -> None:
    # USB_CC1
    add_path(board, 51, F_CU, 0.25, [(50.625, -39.08), (50.625, -37.8)])
    add_via(board, 51, 50.625, -37.8, 0.6, 0.3)
    add_path(board, 51, F_CU, 0.25, [(47.675, -31.5), (47.675, -32.3)])
    add_via(board, 51, 47.675, -32.3, 0.6, 0.3)
    add_path(board, 51, IN1_CU, 0.25, [(50.625, -37.8), (47.675, -37.8)])
    add_via(board, 51, 47.675, -37.8, 0.6, 0.3)
    add_path(board, 51, IN2_CU, 0.25, [(47.675, -37.8), (47.675, -32.3)])

    # USB_CC2
    add_path(board, 52, F_CU, 0.25, [(51.625, -39.08), (51.625, -37.2)])
    add_via(board, 52, 51.625, -37.2, 0.6, 0.3)
    add_path(board, 52, F_CU, 0.25, [(52.675, -31.5), (52.675, -32.3)])
    add_via(board, 52, 52.675, -32.3, 0.6, 0.3)
    add_path(board, 52, IN1_CU, 0.25, [(51.625, -37.2), (52.675, -37.2)])
    add_via(board, 52, 52.675, -37.2, 0.6, 0.3)
    add_path(board, 52, IN2_CU, 0.25, [(52.675, -37.2), (52.675, -32.3)])


def route_ground(board: pcbnew.BOARD) -> None:
    add_path(
        board,
        23,
        B_CU,
        0.3,
        [
            (1.5, 18.0),
            (1.5, -34.0),
            (73.0, -34.0),
            (73.0, -32.0),
            (85.5, -32.0),
            (85.5, -34.0),
            (RIGHT_GND_X, -34.0),
            (RIGHT_GND_X, 39.0),
        ],
    )

    bottom_stubs = [
        [(14.54, -38.0), (17.0, -38.0), (17.0, -34.0)],
        [(23.27, -38.0), (23.27, -34.0)],
        [(29.46, -38.0), (29.46, -34.0)],
        [(46.805, -39.0), (46.805, -34.0)],
        [(46.805, -35.2), (46.805, -34.0)],
        [(48.375, -39.08), (48.375, -34.0)],
        [(49.325, -31.5), (49.325, -34.0)],
        [(53.875, -39.08), (53.875, -34.0)],
        [(54.325, -31.5), (54.325, -34.0), (53.875, -34.0)],
        [(55.445, -39.0), (55.445, -34.0)],
        [(55.445, -35.2), (55.445, -34.0)],
    ]
    for points in bottom_stubs:
        add_path(board, 23, F_CU, 0.3, points)
    for x, y in {
        (17.0, -34.0),
        (23.27, -34.0),
        (29.46, -34.0),
        (46.805, -34.0),
        (48.375, -34.0),
        (49.325, -34.0),
        (53.875, -34.0),
        (55.445, -34.0),
    }:
        add_via(board, 23, x, y)

    # Tie the encoder switch ground pads into the main bottom ground trunk explicitly.
    add_path(board, 23, F_CU, 0.3, [(8.0, -2.5), (8.0, -17.5), (10.5, -17.5), (10.5, -21.0)])
    add_via(board, 23, 10.5, -21.0)
    add_path(board, 23, B_CU, 0.3, [(10.5, -21.0), (14.54, -21.0), (14.54, -34.0)])

    # Stitch the display header ground pad into the right-side ground branch instead of relying on the zone.
    add_path(board, 23, F_CU, 0.3, [(45.5, -6.5), (45.5, -5.0), (68.5, -5.0)])
    add_via(board, 23, 68.5, -5.0)
    add_path(board, 23, B_CU, 0.3, [(68.5, -5.0), (68.5, -4.85), (70.3, -4.85)])

    add_path(board, 23, F_CU, 0.3, [(66.5, -28.26), (68.15, -28.26)])
    add_path(board, 23, F_CU, 0.3, [(68.15, -28.26), (86.0, -28.26)])
    add_path(board, 23, F_CU, 0.3, [(86.0, -28.26), (98.25, -28.26)])
    add_path(board, 23, F_CU, 0.3, [(98.25, -28.26), (RIGHT_GND_X, -28.26)])
    add_path(board, 23, F_CU, 0.3, [(68.15, -26.99), (68.15, -28.26)])
    add_path(board, 23, F_CU, 0.3, [(68.15, -25.5), (68.15, -26.99)])
    add_path(board, 23, F_CU, 0.3, [(73.75, -20.54), (86.0, -20.54), (86.0, -28.26)])
    add_path(board, 23, B_CU, 0.3, [(70.3, -4.85), (RIGHT_GND_X, -4.85)])

    branch_specs = [
        (-28.0, [(98.25, -28.0), (98.25, -28.26)]),
        (-24.95, [(93.85, -24.95), (93.85, -23.8), (106.5, -23.8), (106.5, -24.95)]),
        (-23.05, [(95.562, -23.05), (106.5, -23.05)]),
        (-22.0, [(104.0, -22.0), (106.5, -22.0)]),
        (-16.3, [(96.1, -16.3), (106.5, -16.3)]),
        (4.4, [(102.75, 5.0), (102.75, 4.4), (106.5, 4.4)]),
        (7.0, [(100.3, 7.0), (106.5, 7.0)]),
        (11.5, [(95.862, 12.0), (95.862, 11.5), (106.5, 11.5)]),
        (12.45, [(99.787, 12.95), (99.787, 12.45), (106.5, 12.45)]),
        (21.2, [(97.825, 22.0), (97.825, 21.2), (106.5, 21.2)]),
        (22.8, [(88.862, 22.0), (88.862, 22.8), (106.5, 22.8)]),
        (24.8, [(93.825, 24.0), (93.825, 24.8), (106.5, 24.8)]),
        (27.8, [(94.825, 27.0), (94.825, 27.8), (106.5, 27.8)]),
        (33.8, [(96.825, 33.0), (96.825, 33.8), (106.5, 33.8)]),
        (33.8, [(100.825, 33.0), (100.825, 33.8), (106.5, 33.8)]),
        (39.0, [(96.825, 39.0), (106.5, 39.0)]),
    ]
    for y, points in branch_specs:
        add_path(board, 23, F_CU, 0.3, points)
        if close(points[-1][0], RIGHT_GND_X) and close(points[-1][1], y):
            add_via(board, 23, RIGHT_GND_X, y)

    add_via(board, 23, RIGHT_GND_X, -28.26)


def build_routes(board: pcbnew.BOARD) -> None:
    route_rows(board)
    route_columns(board)
    route_mcu_signals(board)
    route_led_and_ir(board)
    route_3v3(board)
    route_5v(board)
    route_battery_and_boost(board)
    route_usb_and_battery(board)
    route_usb_cc(board)
    route_en(board)
    route_ground(board)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("board", nargs="?", default="Macropad.kicad_pcb")
    parser.add_argument("--output")
    args = parser.parse_args()

    board = pcbnew.LoadBoard(args.board)
    board.SetCopperLayerCount(4)
    remove_top_items(board)
    remove_row_items(board)
    remove_rgb_orphan_stubs(board)
    build_routes(board)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(args.output or args.board)


if __name__ == "__main__":
    main()
