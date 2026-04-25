#!/usr/bin/env python3
"""Generate a printable ring of ArUco markers for the Stable-Bot platform.

Outputs (into --out-dir):
  ring_full.png       Full ring at print DPI, for visual reference.
  page_NN.png         Letter-size (8.5x11") pages, each a tile of the ring,
                      with registration marks at the corners of the usable
                      area so you can align pieces when taping them together.
  pages_all.pdf       Same pages bundled into one PDF, easier to print.
  marker_layout.yaml  Per-marker (id, x_mm, y_mm, size_mm) in platform-frame
                      coordinates — feed this to cv.aruco.estimatePoseBoard
                      in the vision node.

Print at 100% / actual size (NOT 'fit to page'), cut along the registration
marks, align adjacent tiles by their overlap, and stick the assembled ring
onto the platform with the center crosshair on the disk center. Use matte
vinyl sticker paper — glossy causes specular highlights under the OAK's IR
projector and wrecks corner detection.

All markers are printed with the same orientation (local +y axis = platform
+y axis), so the vision node's board layout is: for each marker in the
YAML, its four corners are (x-s/2, y-s/2), (x+s/2, y-s/2), (x+s/2, y+s/2),
(x-s/2, y+s/2), with s = marker_size_mm.

Typical usage:
  python3 generate_aruco_ring.py \\
      --platform-diameter 300 \\
      --ring-radius 120 \\
      --num-markers 8 \\
      --marker-size 50 \\
      --out-dir ./aruco_ring

Dependencies: numpy, opencv-python (with aruco contrib), Pillow, PyYAML.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml
from PIL import Image


def _get_dictionary(name):
    dict_id = getattr(cv2.aruco, name, None)
    if dict_id is None:
        raise SystemExit(
            f"unknown dictionary '{name}'. Try DICT_4X4_50, DICT_4X4_100, "
            "DICT_5X5_50, etc.")
    # OpenCV 4.7+ uses getPredefinedDictionary; older uses Dictionary_get.
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def _draw_marker(aruco_dict, marker_id, size_px):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_px)
    return cv2.aruco.drawMarker(aruco_dict, marker_id, size_px)


def _draw_dashed_line(img, pt1, pt2, color=(0, 0, 0), thickness=1,
                      dash_px=10, gap_px=6):
    x1, y1 = pt1
    x2, y2 = pt2
    length = math.hypot(x2 - x1, y2 - y1)
    if length == 0:
        return
    dx = (x2 - x1) / length
    dy = (y2 - y1) / length
    step = dash_px + gap_px
    d = 0.0
    while d < length:
        e = min(length, d + dash_px)
        sx = int(round(x1 + dx * d))
        sy = int(round(y1 + dy * d))
        ex = int(round(x1 + dx * e))
        ey = int(round(y1 + dy * e))
        cv2.line(img, (sx, sy), (ex, ey), color, thickness, cv2.LINE_AA)
        d += step


def _draw_dashed_line_outside_circle(img, pt1, pt2, cx, cy, R, **kwargs):
    """Like _draw_dashed_line, but only the portions of the segment that
    fall outside the circle (cx, cy, R) are drawn. Only handles
    axis-aligned segments — sufficient for seam guides which run page
    edge to page edge."""
    x1, y1 = pt1
    x2, y2 = pt2
    if x1 == x2:
        x = x1
        if abs(x - cx) >= R:
            _draw_dashed_line(img, pt1, pt2, **kwargs)
            return
        dy = math.sqrt(R * R - (x - cx) ** 2)
        y_top, y_bot = cy - dy, cy + dy
        y_lo, y_hi = min(y1, y2), max(y1, y2)
        if y_lo < y_top:
            _draw_dashed_line(img, (x, int(round(y_lo))),
                              (x, int(round(min(y_top, y_hi)))), **kwargs)
        if y_hi > y_bot:
            _draw_dashed_line(img, (x, int(round(max(y_bot, y_lo)))),
                              (x, int(round(y_hi))), **kwargs)
    elif y1 == y2:
        y = y1
        if abs(y - cy) >= R:
            _draw_dashed_line(img, pt1, pt2, **kwargs)
            return
        dx = math.sqrt(R * R - (y - cy) ** 2)
        x_left, x_right = cx - dx, cx + dx
        x_lo, x_hi = min(x1, x2), max(x1, x2)
        if x_lo < x_left:
            _draw_dashed_line(img, (int(round(x_lo)), y),
                              (int(round(min(x_left, x_hi))), y), **kwargs)
        if x_hi > x_right:
            _draw_dashed_line(img, (int(round(max(x_right, x_lo))), y),
                              (int(round(x_hi)), y), **kwargs)
    else:
        # Diagonal: fall back to drawing the whole thing.
        _draw_dashed_line(img, pt1, pt2, **kwargs)


def build_ring(args):
    mm_per_in = 25.4
    px_per_mm = args.dpi / mm_per_in

    # Cut ring diameter matches the platform exactly, so the printed sheet
    # covers the platform edge-to-edge with no extra margin.
    cut_radius_mm = args.platform_diameter / 2 + args.cut_margin
    # Canvas = cut-ring bounding square + 1 mm so the cut-line's own
    # thickness isn't clipped at the canvas edge. This is the only
    # "whitespace" — everything outside the cut ring is discarded when the
    # user follows the black circle with scissors.
    canvas_mm = 2 * cut_radius_mm + 1
    canvas_px = int(round(canvas_mm * px_per_mm))
    canvas = np.ones((canvas_px, canvas_px, 3), dtype=np.uint8) * 255

    aruco_dict = _get_dictionary(args.dict)
    marker_px = int(round(args.marker_size * px_per_mm))

    layout = []
    max_corner_mm = 0.0
    for i in range(args.num_markers):
        marker_id = args.first_id + i
        # Start at the top (angle = pi/2), go counter-clockwise so marker 0 is
        # at the +y axis and subsequent markers rotate toward +x.
        angle = math.pi / 2 - 2 * math.pi * i / args.num_markers
        x_mm = args.ring_radius * math.cos(angle)
        y_mm = args.ring_radius * math.sin(angle)

        # Canvas pixel coords: origin at top-left, +y down → flip sign on y.
        cx_px = int(round(canvas_px / 2 + x_mm * px_per_mm))
        cy_px = int(round(canvas_px / 2 - y_mm * px_per_mm))

        img = _draw_marker(aruco_dict, marker_id, marker_px)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        half = marker_px // 2
        x0, y0 = cx_px - half, cy_px - half
        canvas[y0:y0 + marker_px, x0:x0 + marker_px] = img_rgb

        # Small ID label below each marker (won't be seen by the detector,
        # useful to humans when assembling).
        label_px = (cx_px - 30, cy_px + half + 18)
        cv2.putText(canvas, f"id {marker_id}", label_px,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1,
                    cv2.LINE_AA)

        # Markers are axis-aligned, so the farthest point from the platform
        # center is the outermost corner of the axis-aligned bounding box.
        max_corner_mm = max(
            max_corner_mm,
            math.hypot(abs(x_mm) + args.marker_size / 2,
                       abs(y_mm) + args.marker_size / 2))

        layout.append({
            'id': marker_id,
            'x_mm': round(x_mm, 3),
            'y_mm': round(y_mm, 3),
            'size_mm': args.marker_size,
        })

    required_cut_radius_mm = max_corner_mm + args.marker_border
    if required_cut_radius_mm > cut_radius_mm:
        print(
            f"warning: markers + {args.marker_border:.1f} mm quiet zone need "
            f"cut radius {required_cut_radius_mm:.1f} mm, but the cut ring "
            f"is {cut_radius_mm:.1f} mm (platform_diameter/2). Markers will "
            f"clip the cut line — reduce --ring-radius or --marker-size, or "
            f"increase --platform-diameter.", file=sys.stderr)

    # Center crosshair for alignment.
    c = canvas_px // 2
    tick_px = int(round(5 * px_per_mm))
    cv2.line(canvas, (c - tick_px, c), (c + tick_px, c), (150, 150, 150), 2)
    cv2.line(canvas, (c, c - tick_px), (c, c + tick_px), (150, 150, 150), 2)

    # The alignment circle (--center-circle-diameter) is intentionally NOT
    # drawn on the shared canvas — if it were, the tile boundaries (which
    # all meet at the canvas centre) would slice it into quarter-arcs and
    # no single page would show the full circle. Instead it's painted onto
    # each tile page after tiling, see tile_pages.

    # Solid cut-ring: the black circle IS the platform boundary, follow it
    # with scissors.
    cut_radius_px = int(round(cut_radius_mm * px_per_mm))
    cv2.circle(canvas, (c, c), cut_radius_px, (0, 0, 0), 2, cv2.LINE_AA)

    return canvas, canvas_mm, cut_radius_mm, layout, px_per_mm


def _plan_tiles(canvas_mm, cut_radius_mm, args):
    """Pick the page orientation that yields fewest non-empty tiles.

    Tiles whose usable area is entirely outside the cut circle are skipped —
    no content, no page. The surviving grid is centered on the canvas so the
    corners of the square bounding box (which lie outside the circle) get
    discarded symmetrically.
    """
    letter_long_mm = 11.0 * 25.4   # 279.4
    letter_short_mm = 8.5 * 25.4   # 215.9

    best = None
    for orientation, page_w_mm, page_h_mm in (
        ('portrait', letter_short_mm, letter_long_mm),
        ('landscape', letter_long_mm, letter_short_mm),
    ):
        usable_w_mm = page_w_mm - 2 * args.page_margin
        usable_h_mm = page_h_mm - 2 * args.page_margin
        stride_w_mm = usable_w_mm - args.page_overlap
        stride_h_mm = usable_h_mm - args.page_overlap

        cols = max(1, math.ceil((canvas_mm - args.page_overlap) / stride_w_mm))
        rows = max(1, math.ceil((canvas_mm - args.page_overlap) / stride_h_mm))

        # Center the tile grid on the canvas: any slack from the grid being
        # wider than the canvas (which shows up as blank margin in the tiles)
        # is distributed evenly to left/right and top/bottom.
        total_w_mm = cols * stride_w_mm + args.page_overlap
        total_h_mm = rows * stride_h_mm + args.page_overlap
        x0_global = (canvas_mm - total_w_mm) / 2
        y0_global = (canvas_mm - total_h_mm) / 2

        cx = canvas_mm / 2
        cy = canvas_mm / 2

        tiles = []
        for row in range(rows):
            for col in range(cols):
                sx0 = x0_global + col * stride_w_mm
                sy0 = y0_global + row * stride_h_mm
                sx1 = sx0 + usable_w_mm
                sy1 = sy0 + usable_h_mm
                # Closest point of the tile rectangle to the canvas centre:
                # if its distance exceeds cut_radius, the tile is entirely
                # outside the printable disc and contributes nothing.
                closest_x = max(sx0, min(cx, sx1))
                closest_y = max(sy0, min(cy, sy1))
                if math.hypot(closest_x - cx, closest_y - cy) > cut_radius_mm:
                    continue
                tiles.append({
                    'page_w_mm': page_w_mm,
                    'page_h_mm': page_h_mm,
                    'usable_w_mm': usable_w_mm,
                    'usable_h_mm': usable_h_mm,
                    'src_x0_mm': sx0,
                    'src_y0_mm': sy0,
                    'row': row + 1,
                    'col': col + 1,
                    'rows_total': rows,
                    'cols_total': cols,
                })

        key = (len(tiles), 0 if orientation == 'landscape' else 1)
        candidate = (key, orientation, tiles)
        if best is None or candidate[0] < best[0]:
            best = candidate

    return best[1], best[2]


_CORNER_ROTATION_CW_DEG = {
    ('top-left', 'top-left'): 0, ('top-left', 'top-right'): 90,
    ('top-left', 'bottom-right'): 180, ('top-left', 'bottom-left'): 270,
    ('top-right', 'top-left'): 270, ('top-right', 'top-right'): 0,
    ('top-right', 'bottom-right'): 90, ('top-right', 'bottom-left'): 180,
    ('bottom-right', 'top-left'): 180, ('bottom-right', 'top-right'): 270,
    ('bottom-right', 'bottom-right'): 0, ('bottom-right', 'bottom-left'): 90,
    ('bottom-left', 'top-left'): 90, ('bottom-left', 'top-right'): 180,
    ('bottom-left', 'bottom-right'): 270, ('bottom-left', 'bottom-left'): 0,
}


def _natural_corner(cc_x, cc_y, margin_px, usable_w_px, usable_h_px):
    """Which corner of the usable area the canvas centre lands in."""
    right = (cc_x - margin_px) > usable_w_px / 2
    bottom = (cc_y - margin_px) > usable_h_px / 2
    if right and bottom:
        return 'bottom-right'
    if right:
        return 'top-right'
    if bottom:
        return 'bottom-left'
    return 'top-left'


def _rotate_image_cw(img, deg):
    if deg == 0:
        return img
    if deg == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if deg == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    raise ValueError(f"unsupported rotation {deg}")


def _stitch_pages_at_centers(pages, cc_positions, px_per_mm):
    """Core preview stitcher: places each page so that its canvas-centre
    pixel lands at a shared origin. Accepts pages of varying shapes."""
    if not pages:
        return None

    min_x = min(-cc_x for cc_x, _ in cc_positions)
    min_y = min(-cc_y for _, cc_y in cc_positions)
    max_x = max(pages[i].shape[1] - cc_positions[i][0]
                for i in range(len(pages)))
    max_y = max(pages[i].shape[0] - cc_positions[i][1]
                for i in range(len(pages)))

    preview_w = max_x - min_x
    preview_h = max_y - min_y
    preview = np.ones((preview_h, preview_w, 3), dtype=np.uint8) * 255

    origin_x = -min_x
    origin_y = -min_y

    # Paste pages with np.minimum so dark content from any page wins in
    # overlap regions (margins are white on both pages so they stay white).
    for page, (cc_x, cc_y) in zip(pages, cc_positions):
        h, w = page.shape[:2]
        tl_x = origin_x - cc_x
        tl_y = origin_y - cc_y
        preview[tl_y:tl_y + h, tl_x:tl_x + w] = np.minimum(
            preview[tl_y:tl_y + h, tl_x:tl_x + w], page)

    palette = [
        (0, 0, 220), (0, 180, 0), (220, 0, 0), (0, 140, 220),
        (180, 0, 180), (180, 180, 0),
    ]
    border_thickness = max(2, int(round(0.3 * px_per_mm)))
    label_scale = 1.8
    label_thickness = max(3, int(round(0.4 * px_per_mm)))
    for i, (page, (cc_x, cc_y)) in enumerate(zip(pages, cc_positions)):
        h, w = page.shape[:2]
        tl_x = origin_x - cc_x
        tl_y = origin_y - cc_y
        color = palette[i % len(palette)]
        cv2.rectangle(preview, (tl_x, tl_y), (tl_x + w - 1, tl_y + h - 1),
                      color, border_thickness, cv2.LINE_AA)
        text_x = tl_x + int(round(6 * px_per_mm))
        text_y = tl_y + int(round(18 * px_per_mm))
        cv2.putText(preview, f"p{i + 1}", (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, label_scale, color,
                    label_thickness, cv2.LINE_AA)

    cross_px = int(round(20 * px_per_mm))
    cv2.line(preview, (origin_x - cross_px, origin_y),
             (origin_x + cross_px, origin_y), (50, 50, 255), 3, cv2.LINE_AA)
    cv2.line(preview, (origin_x, origin_y - cross_px),
             (origin_x, origin_y + cross_px), (50, 50, 255), 3, cv2.LINE_AA)
    return preview


def _generate_assembly_preview(pages, tiles, canvas_mm, args, px_per_mm):
    """Stitch all (unrotated) pages together with their canvas-centres
    aligned, so the user can sanity-check that the tiling does what
    they expect before committing paper + ink to the print."""
    if not pages:
        return None
    margin_px = int(round(args.page_margin * px_per_mm))
    canvas_cx_mm = canvas_mm / 2
    canvas_cy_mm = canvas_mm / 2
    cc_positions = []
    for t in tiles:
        cc_x = int(round((canvas_cx_mm - t['src_x0_mm']) * px_per_mm)) + margin_px
        cc_y = int(round((canvas_cy_mm - t['src_y0_mm']) * px_per_mm)) + margin_px
        cc_positions.append((cc_x, cc_y))
    return _stitch_pages_at_centers(pages, cc_positions, px_per_mm)


def _generate_uniform_preview(pages, pages_meta, px_per_mm):
    """Uniform-mode preview: reverse the per-page print rotation (as the
    user does physically during assembly), then stitch at canvas-centres.
    The reversed pages show every seam dashed line the real PDF shows."""
    if not pages:
        return None
    reversed_pages = []
    reversed_cc = []
    for page, meta in zip(pages, pages_meta):
        rev_deg = (360 - meta['rot_deg']) % 360
        h_orig = meta['page_h_px']
        w_orig = meta['page_w_px']
        cc_x, cc_y = meta['cc_x'], meta['cc_y']
        if rev_deg == 0:
            rp = page
            new_cc = (cc_x, cc_y)
        elif rev_deg == 90:
            rp = cv2.rotate(page, cv2.ROTATE_90_CLOCKWISE)
            new_cc = (h_orig - 1 - cc_y, cc_x)
        elif rev_deg == 180:
            rp = cv2.rotate(page, cv2.ROTATE_180)
            new_cc = (w_orig - 1 - cc_x, h_orig - 1 - cc_y)
        elif rev_deg == 270:
            rp = cv2.rotate(page, cv2.ROTATE_90_COUNTERCLOCKWISE)
            new_cc = (cc_y, w_orig - 1 - cc_x)
        else:
            rp = page
            new_cc = (cc_x, cc_y)
        reversed_pages.append(rp)
        reversed_cc.append(new_cc)
    return _stitch_pages_at_centers(reversed_pages, reversed_cc, px_per_mm)


def _build_uniform_pages(canvas, canvas_mm, cut_radius_mm, args, px_per_mm):
    """Build 4 landscape pages using SQUARE tiles (canvas/2 wide), with
    each page's tile rotated so the canvas-centre always lands in the
    same corner of the usable area. Square tiles stay square under 90°
    rotation, so every page keeps the same landscape orientation.

    The user physically rotates each page around the M3 bolt when
    assembling; the net rotation (print + assembly) cancels to zero, so
    ArUco markers end up at their natural platform-frame orientation.
    """
    page_w_mm = 11.0 * 25.4
    page_h_mm = 8.5 * 25.4
    margin_mm = args.page_margin
    usable_w_mm = page_w_mm - 2 * margin_mm
    usable_h_mm = page_h_mm - 2 * margin_mm

    tile_size_mm = canvas_mm / 2
    if tile_size_mm > min(usable_w_mm, usable_h_mm):
        raise SystemExit(
            f"canvas is {canvas_mm:.1f} mm so each quarter tile is "
            f"{tile_size_mm:.1f} mm, which doesn't fit on a letter "
            f"landscape page with {margin_mm:.1f} mm margin "
            f"(usable {usable_w_mm:.1f} x {usable_h_mm:.1f} mm). "
            f"--canvas-center-corner requires the quarter tile to fit; "
            f"reduce --page-margin or --platform-diameter.")

    page_w_px = int(round(page_w_mm * px_per_mm))
    page_h_px = int(round(page_h_mm * px_per_mm))
    usable_w_px = int(round(usable_w_mm * px_per_mm))
    usable_h_px = int(round(usable_h_mm * px_per_mm))
    margin_px = int(round(margin_mm * px_per_mm))
    tile_size_px = int(round(tile_size_mm * px_per_mm))

    canvas_px = canvas.shape[0]
    half = canvas_px // 2

    # Each quadrant of the canvas + its natural corner for the canvas
    # centre within that quadrant + the platform quadrant its content
    # belongs in (so we can label the page for the assembler).
    natural_tiles = [
        {'row': 1, 'col': 1, 'natural': 'bottom-right',
         'platform_quadrant': 'upper-left',
         'tile': canvas[0:half, 0:half].copy()},
        {'row': 1, 'col': 2, 'natural': 'bottom-left',
         'platform_quadrant': 'upper-right',
         'tile': canvas[0:half, half:canvas_px].copy()},
        {'row': 2, 'col': 1, 'natural': 'top-right',
         'platform_quadrant': 'lower-left',
         'tile': canvas[half:canvas_px, 0:half].copy()},
        {'row': 2, 'col': 2, 'natural': 'top-left',
         'platform_quadrant': 'lower-right',
         'tile': canvas[half:canvas_px, half:canvas_px].copy()},
    ]

    target = args.canvas_center_corner
    if target == 'bottom-left':
        paste_x = margin_px
        paste_y = margin_px + usable_h_px - tile_size_px
        cc_x = paste_x
        cc_y = paste_y + tile_size_px - 1
    elif target == 'bottom-right':
        paste_x = margin_px + usable_w_px - tile_size_px
        paste_y = margin_px + usable_h_px - tile_size_px
        cc_x = paste_x + tile_size_px - 1
        cc_y = paste_y + tile_size_px - 1
    elif target == 'top-left':
        paste_x = margin_px
        paste_y = margin_px
        cc_x = paste_x
        cc_y = paste_y
    elif target == 'top-right':
        paste_x = margin_px + usable_w_px - tile_size_px
        paste_y = margin_px
        cc_x = paste_x + tile_size_px - 1
        cc_y = paste_y
    else:
        raise ValueError(f"unknown target corner {target}")

    pages = []
    pages_meta = []
    for i, nt in enumerate(natural_tiles, start=1):
        rot_deg = _CORNER_ROTATION_CW_DEG[(nt['natural'], target)]
        tile_rotated = _rotate_image_cw(nt['tile'], rot_deg)

        page = np.ones((page_h_px, page_w_px, 3), dtype=np.uint8) * 255
        th, tw = tile_rotated.shape[:2]
        page[paste_y:paste_y + th, paste_x:paste_x + tw] = tile_rotated
        pages_meta.append({
            'rot_deg': rot_deg,
            'cc_x': cc_x, 'cc_y': cc_y,
            'page_w_px': page_w_px, 'page_h_px': page_h_px,
        })

        if args.center_circle_diameter > 0:
            cc_r_px = int(round(args.center_circle_diameter / 2
                                * px_per_mm))
            cv2.circle(page, (cc_x, cc_y), cc_r_px, (150, 150, 150),
                       2, cv2.LINE_AA)

        # Seam lines on the two tile edges that meet at the canvas-centre
        # corner — those are the seams between this quadrant and its
        # neighbours on the assembled platform. Other tile edges are free
        # (no adjacent piece), so no cut guides there.
        if args.seam_bleed > 0:
            bleed_px = int(round(args.seam_bleed * px_per_mm))
            inner_off_px = int(round((margin_mm - args.seam_bleed)
                                     * px_per_mm))
            cut_R_px = cut_radius_mm * px_per_mm
            seam_dirs = {
                'bottom-left': ('bottom', 'left'),
                'bottom-right': ('bottom', 'right'),
                'top-left': ('top', 'left'),
                'top-right': ('top', 'right'),
            }[target]
            for direction in seam_dirs:
                if direction == 'left':
                    x_outer = max(0, margin_px - bleed_px)
                    _draw_dashed_line(page, (x_outer, 0),
                                      (x_outer, page_h_px))
                    if args.seam_alignment_line and inner_off_px > 0:
                        x_in = x_outer + inner_off_px
                        if 0 <= x_in < page_w_px:
                            _draw_dashed_line_outside_circle(
                                page, (x_in, 0), (x_in, page_h_px),
                                cc_x, cc_y, cut_R_px,
                                dash_px=4, gap_px=6)
                elif direction == 'right':
                    x_outer = min(page_w_px - 1,
                                  margin_px + usable_w_px + bleed_px)
                    _draw_dashed_line(page, (x_outer, 0),
                                      (x_outer, page_h_px))
                    if args.seam_alignment_line and inner_off_px > 0:
                        x_in = x_outer - inner_off_px
                        if 0 <= x_in < page_w_px:
                            _draw_dashed_line_outside_circle(
                                page, (x_in, 0), (x_in, page_h_px),
                                cc_x, cc_y, cut_R_px,
                                dash_px=4, gap_px=6)
                elif direction == 'top':
                    y_outer = max(0, margin_px - bleed_px)
                    _draw_dashed_line(page, (0, y_outer),
                                      (page_w_px, y_outer))
                    if args.seam_alignment_line and inner_off_px > 0:
                        y_in = y_outer + inner_off_px
                        if 0 <= y_in < page_h_px:
                            _draw_dashed_line_outside_circle(
                                page, (0, y_in), (page_w_px, y_in),
                                cc_x, cc_y, cut_R_px,
                                dash_px=4, gap_px=6)
                elif direction == 'bottom':
                    y_outer = min(page_h_px - 1,
                                  margin_px + usable_h_px + bleed_px)
                    _draw_dashed_line(page, (0, y_outer),
                                      (page_w_px, y_outer))
                    if args.seam_alignment_line and inner_off_px > 0:
                        y_in = y_outer - inner_off_px
                        if 0 <= y_in < page_h_px:
                            _draw_dashed_line_outside_circle(
                                page, (0, y_in), (page_w_px, y_in),
                                cc_x, cc_y, cut_R_px,
                                dash_px=4, gap_px=6)

        label_bits = [
            f"page {i} of 4",
            f"natural tile (row {nt['row']}, col {nt['col']}) -> "
            f"{nt['platform_quadrant']} quadrant of platform",
        ]
        if rot_deg != 0:
            back = (360 - rot_deg) % 360
            label_bits.append(
                f"PRINTED ROTATED {rot_deg} deg CW -- rotate "
                f"{back} deg CW back when assembling")
        label_bits.append("print at 100%, NOT 'fit to page'")
        label = "   |   ".join(label_bits)
        label_y = max(18, margin_px // 3)
        cv2.putText(page, label, (max(10, margin_px // 4), label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1,
                    cv2.LINE_AA)
        pages.append(page)

    return pages, pages_meta


def tile_pages(canvas, canvas_mm, cut_radius_mm, args, px_per_mm):
    """Split canvas into letter-size pages, preferring the fewest-pages
    orientation and skipping tiles that would print blank. Also builds an
    assembly-preview image (pages stitched at their canvas-centres) and
    optionally rotates each page so the canvas-centre lands in a uniform
    corner of the usable area."""
    if args.canvas_center_corner != 'natural':
        pages, pages_meta = _build_uniform_pages(
            canvas, canvas_mm, cut_radius_mm, args, px_per_mm)
        # Reverse-rotate each page (as the user does physically when
        # assembling) and stitch at the canvas-centres, so the preview
        # shows the seam dashed lines exactly as they'll appear on the
        # finished platform.
        preview = _generate_uniform_preview(pages, pages_meta, px_per_mm)
        return pages, 2, 2, 'landscape (uniform)', preview

    orientation, tiles = _plan_tiles(canvas_mm, cut_radius_mm, args)

    margin_px = int(round(args.page_margin * px_per_mm))
    canvas_px = canvas.shape[0]

    pages = []
    page_meta = []
    for i, t in enumerate(tiles, start=1):
        page_w_px = int(round(t['page_w_mm'] * px_per_mm))
        page_h_px = int(round(t['page_h_mm'] * px_per_mm))
        usable_w_px = int(round(t['usable_w_mm'] * px_per_mm))
        usable_h_px = int(round(t['usable_h_mm'] * px_per_mm))
        page = np.ones((page_h_px, page_w_px, 3), dtype=np.uint8) * 255

        src_x0 = int(round(t['src_x0_mm'] * px_per_mm))
        src_y0 = int(round(t['src_y0_mm'] * px_per_mm))
        src_x1 = src_x0 + usable_w_px
        src_y1 = src_y0 + usable_h_px
        # Clip the source region to the canvas (tiles near the edge extend
        # past the canvas once the grid is centred).
        csx0 = max(0, src_x0)
        csy0 = max(0, src_y0)
        csx1 = min(canvas_px, src_x1)
        csy1 = min(canvas_px, src_y1)
        dx0 = margin_px + (csx0 - src_x0)
        dy0 = margin_px + (csy0 - src_y0)
        cw = csx1 - csx0
        ch = csy1 - csy0
        if cw > 0 and ch > 0:
            page[dy0:dy0 + ch, dx0:dx0 + cw] = canvas[csy0:csy1, csx0:csx1]

        # Cut-ring centre and radius in this page's pixel coords. Used by
        # both the alignment circle and the inner seam line clipping.
        canvas_cx_mm = canvas_mm / 2
        canvas_cy_mm = canvas_mm / 2
        cc_x = int(round((canvas_cx_mm - t['src_x0_mm']) * px_per_mm)
                   + margin_px)
        cc_y = int(round((canvas_cy_mm - t['src_y0_mm']) * px_per_mm)
                   + margin_px)
        cut_ring_R_px = cut_radius_mm * px_per_mm

        # Alignment circle drawn directly onto the page (not the canvas)
        # so the full circle survives even though the canvas centre lands
        # on a tile corner where four pages meet. Each page gets its own
        # complete circle around the centre mark; the small portion that
        # falls into the printer margin will be trimmed off with the rest
        # of the margin, leaving an arc on each piece that joins into a
        # full circle once the four pieces are taped together.
        if args.center_circle_diameter > 0:
            cc_radius_px = int(round(args.center_circle_diameter / 2
                                     * px_per_mm))
            cv2.circle(page, (cc_x, cc_y), cc_radius_px, (150, 150, 150),
                       2, cv2.LINE_AA)

        # Dashed cut lines on seam edges (shared with another tile). The
        # line is outset by --seam-bleed mm from the tile's content edge so
        # cutting on the dashes keeps a small strip of paper past the
        # content — when adjacent pages are placed together those strips
        # give a small overlap for taping/alignment.
        if args.seam_bleed > 0:
            bleed_px = int(round(args.seam_bleed * px_per_mm))
            inner_offset_px = int(round(
                (args.page_margin - args.seam_bleed) * px_per_mm))
            left_seam = t['col'] > 1
            right_seam = t['col'] < t['cols_total']
            top_seam = t['row'] > 1
            bottom_seam = t['row'] < t['rows_total']
            # Outer cut line (cut here with a scalpel after assembly).
            if left_seam:
                x = max(0, margin_px - bleed_px)
                _draw_dashed_line(page, (x, 0), (x, page_h_px))
            if right_seam:
                x = min(page_w_px - 1, margin_px + usable_w_px + bleed_px)
                _draw_dashed_line(page, (x, 0), (x, page_h_px))
            if top_seam:
                y = max(0, margin_px - bleed_px)
                _draw_dashed_line(page, (0, y), (page_w_px, y))
            if bottom_seam:
                y = min(page_h_px - 1, margin_px + usable_h_px + bleed_px)
                _draw_dashed_line(page, (0, y), (page_w_px, y))
            # Inner alignment line, mirrored across the outer line at the
            # same distance as the outer line is from the page edge. Lay an
            # adjacent page's factory edge on this line during assembly to
            # land its overlap consistently. Drawn shorter-dashed so it's
            # visually distinct from the outer cut line. Clipped so it
            # only appears OUTSIDE the cut-ring circle — inside the ring
            # is where markers live, and a stray dashed line crossing the
            # quiet zone could confuse the ArUco detector.
            if args.seam_alignment_line and inner_offset_px > 0:
                if left_seam:
                    x = max(0, margin_px - bleed_px) + inner_offset_px
                    if 0 <= x < page_w_px:
                        _draw_dashed_line_outside_circle(
                            page, (x, 0), (x, page_h_px),
                            cc_x, cc_y, cut_ring_R_px,
                            dash_px=4, gap_px=6)
                if right_seam:
                    x = min(page_w_px - 1,
                            margin_px + usable_w_px + bleed_px) - inner_offset_px
                    if 0 <= x < page_w_px:
                        _draw_dashed_line_outside_circle(
                            page, (x, 0), (x, page_h_px),
                            cc_x, cc_y, cut_ring_R_px,
                            dash_px=4, gap_px=6)
                if top_seam:
                    y = max(0, margin_px - bleed_px) + inner_offset_px
                    if 0 <= y < page_h_px:
                        _draw_dashed_line_outside_circle(
                            page, (0, y), (page_w_px, y),
                            cc_x, cc_y, cut_ring_R_px,
                            dash_px=4, gap_px=6)
                if bottom_seam:
                    y = min(page_h_px - 1,
                            margin_px + usable_h_px + bleed_px) - inner_offset_px
                    if 0 <= y < page_h_px:
                        _draw_dashed_line_outside_circle(
                            page, (0, y), (page_w_px, y),
                            cc_x, cc_y, cut_ring_R_px,
                            dash_px=4, gap_px=6)

        pages.append(page)
        # Remember the natural corner + usable area in px for rotation.
        natural = _natural_corner(cc_x, cc_y, margin_px,
                                  usable_w_px, usable_h_px)
        page_meta.append({
            'natural_corner': natural,
            'usable_w_px': usable_w_px,
            'usable_h_px': usable_h_px,
            'row': t['row'], 'col': t['col'],
            'rows_total': t['rows_total'], 'cols_total': t['cols_total'],
        })

    # Build the assembly preview BEFORE applying any per-page rotation so
    # it always shows the logical "natural" layout, which is what the user
    # actually wants to eyeball.
    preview = _generate_assembly_preview(pages, tiles, canvas_mm, args,
                                          px_per_mm)

    # Now optionally rotate each page so the canvas-centre lands in a
    # uniform corner, and stamp the top-margin label onto the final page.
    final_pages = []
    for i, (page, meta) in enumerate(zip(pages, page_meta), start=1):
        rot_deg = 0
        if args.canvas_center_corner != 'natural':
            rot_deg = _CORNER_ROTATION_CW_DEG[
                (meta['natural_corner'], args.canvas_center_corner)]
        page = _rotate_image_cw(page, rot_deg)

        label_bits = [
            f"page {i} of {len(tiles)}",
            f"(row {meta['row']}/{meta['rows_total']}, "
            f"col {meta['col']}/{meta['cols_total']})",
        ]
        if rot_deg != 0:
            back = (360 - rot_deg) % 360
            label_bits.append(
                f"PRINTED ROTATED {rot_deg} deg CW -- rotate "
                f"{back} deg CW back for assembly")
        label_bits.append("print at 100%, NOT 'fit to page'")
        label = "   |   ".join(label_bits)

        label_y = max(18, margin_px // 3)
        cv2.putText(page, label, (max(10, margin_px // 4), label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1,
                    cv2.LINE_AA)
        final_pages.append(page)

    rows_total = tiles[0]['rows_total'] if tiles else 0
    cols_total = tiles[0]['cols_total'] if tiles else 0
    return final_pages, cols_total, rows_total, orientation, preview


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--platform-diameter', type=float, default=300.0,
                   help='platform diameter in mm (for the reference circle)')
    p.add_argument('--ring-radius', type=float, default=120.0,
                   help='radius of the marker ring in mm from platform center')
    p.add_argument('--num-markers', type=int, default=8,
                   help='number of markers around the ring')
    p.add_argument('--marker-size', type=float, default=50.0,
                   help='outer size of each marker square in mm')
    p.add_argument('--marker-border', type=float, default=3.0,
                   help='extra white quiet-zone margin beyond the marker in mm')
    p.add_argument('--dict', default='DICT_4X4_50',
                   help='ArUco dictionary name')
    p.add_argument('--first-id', type=int, default=0,
                   help='starting marker ID (subsequent markers are +1)')
    p.add_argument('--dpi', type=int, default=300,
                   help='output DPI')
    p.add_argument('--page-margin', type=float, default=5.0,
                   help='printer safe-area margin per side in mm. Default '
                        '5 mm works on most modern printers; raise to 10 '
                        'if yours needs more safe area (each extra mm of '
                        'margin can push up the page count).')
    p.add_argument('--page-overlap', type=float, default=0.0,
                   help='overlap between adjacent pages in mm. Default 0 so '
                        'no content is printed twice; raise to e.g. 10 if '
                        'you want redundant content along the seams for '
                        'visual alignment.')
    p.add_argument('--seam-bleed', type=float, default=0.0,
                   help='if > 0, draw a dashed cut line on each page '
                        'SEAM edge (where two pages meet), outset by this '
                        'many mm from the tile boundary. Cutting on the '
                        'dashes keeps a small bleed strip past the content '
                        'so adjacent pieces overlap slightly for taping. '
                        'Default 0 (no dashed lines).')
    p.add_argument('--seam-alignment-line', action='store_true',
                   help='only meaningful with --seam-bleed > 0: also draw '
                        'a second, short-dashed line on each seam edge '
                        'mirrored across the outer dashed line by the same '
                        'distance as the outer line is from the page edge. '
                        'During assembly, lay an adjacent page so its '
                        'factory edge covers this inner line — that '
                        'positions the overlap consistently. NOTE: this '
                        'creates a content gap of 2 * --seam-bleed mm at '
                        'each seam, see the README for details.')
    p.add_argument('--cut-margin', type=float, default=0.0,
                   help='extra mm beyond platform_diameter/2 for the cut '
                        'ring. Default 0 — the cut ring exactly matches '
                        'the platform.')
    p.add_argument('--center-circle-diameter', type=float, default=0.0,
                   help='if > 0, draw a grey alignment circle of this '
                        'diameter (mm) around the centre crosshair, to '
                        'help line up the printed pieces during assembly. '
                        'Clamped to the cut ring so the circle always '
                        'fits inside the printable area.')
    p.add_argument('--canvas-center-corner',
                   choices=['natural', 'top-left', 'top-right',
                            'bottom-left', 'bottom-right'],
                   default='natural',
                   help='make every output page land the platform centre '
                        'in the same corner of the usable area, so '
                        'printer margin asymmetry affects every page '
                        'identically. In uniform mode the tiles are '
                        'forced to be SQUARE (canvas/2 per side), so '
                        'rotating them does not change the page '
                        'orientation — all four pages stay landscape. '
                        'Each page label says how many degrees CW to '
                        'rotate it back when assembling so markers end '
                        'up at their natural platform-frame orientation. '
                        'Default "natural" (rectangular tiles, no '
                        'rotation). Requires canvas/2 to fit the '
                        'landscape usable area; errors out otherwise. '
                        '--seam-bleed and --seam-alignment-line still '
                        'apply: the dashed lines are drawn on the two '
                        'tile edges that meet at the canvas-centre '
                        'corner (the seams between adjacent quadrants '
                        'on the assembled platform).')
    p.add_argument('--out-dir', default='aruco_ring',
                   help='output directory')
    args = p.parse_args()

    # Users on WSL sometimes pass Windows-style paths like
    # "\home\sorak\foo bar" — on Linux those backslashes are not path
    # separators, so os.makedirs would silently create one directory with
    # a name full of backslashes. Translate them, and expand ~.
    out_dir = args.out_dir
    if '\\' in out_dir:
        fixed = out_dir.replace('\\', '/')
        print(f"note: --out-dir contained backslashes; interpreting "
              f"'{out_dir}' as '{fixed}'", file=sys.stderr)
        out_dir = fixed
    args.out_dir = os.path.expanduser(out_dir)

    os.makedirs(args.out_dir, exist_ok=True)

    canvas, canvas_mm, cut_radius_mm, layout, px_per_mm = build_ring(args)

    full_png = os.path.join(args.out_dir, 'ring_full.png')
    Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)).save(
        full_png, dpi=(args.dpi, args.dpi))

    pages, cols, rows, orientation, preview = tile_pages(
        canvas, canvas_mm, cut_radius_mm, args, px_per_mm)
    if preview is not None:
        preview_path = os.path.join(args.out_dir, 'preview_assembled.png')
        preview_im = Image.fromarray(cv2.cvtColor(preview, cv2.COLOR_BGR2RGB))
        # Downsample so the preview stays a few MB, not tens.
        preview_im.thumbnail((3000, 3000))
        preview_im.save(preview_path, dpi=(args.dpi, args.dpi))
    pil_pages = []
    for i, page in enumerate(pages, start=1):
        path = os.path.join(args.out_dir, f'page_{i:02d}.png')
        im = Image.fromarray(cv2.cvtColor(page, cv2.COLOR_BGR2RGB))
        im.save(path, dpi=(args.dpi, args.dpi))
        pil_pages.append(im)

    pdf_path = os.path.join(args.out_dir, 'pages_all.pdf')
    if pil_pages:
        pil_pages[0].save(
            pdf_path, 'PDF', resolution=float(args.dpi),
            save_all=True, append_images=pil_pages[1:])

    layout_yaml = {
        'platform_diameter_mm': args.platform_diameter,
        'ring_radius_mm': args.ring_radius,
        'dictionary': args.dict,
        'marker_size_mm': args.marker_size,
        'marker_count': args.num_markers,
        'orientation_note': (
            'All markers share the platform-frame orientation. For marker m '
            'with center (x, y), its four corners in platform-frame mm are '
            '(x-s/2, y-s/2), (x+s/2, y-s/2), (x+s/2, y+s/2), (x-s/2, y+s/2) '
            'where s = marker_size_mm. Z=0 in platform frame for all corners.'
        ),
        'markers': layout,
    }
    with open(os.path.join(args.out_dir, 'marker_layout.yaml'), 'w') as f:
        yaml.safe_dump(layout_yaml, f, sort_keys=False)

    skipped = cols * rows - len(pages) if cols and rows else 0
    print(f"generated {args.num_markers} markers "
          f"(IDs {args.first_id}..{args.first_id + args.num_markers - 1})")
    print(f"cut ring: {2 * cut_radius_mm:.1f} mm diameter "
          f"(platform = {args.platform_diameter:.1f} mm)")
    print(f"ring canvas: {canvas_mm:.1f} mm square at {args.dpi} DPI")
    skip_note = f", {skipped} empty tile(s) skipped" if skipped else ""
    print(f"tiled into {len(pages)} {orientation} letter page(s) "
          f"({cols} cols x {rows} rows{skip_note})")

    # Per-page space analysis: ink = actual printed pixels (markers + cut
    # ring line + labels); kept = area inside the cut ring, which is the
    # part that survives the scissors. "Kept" is the honest measure of how
    # much of each page is used after the user cuts — the cut ring is
    # mostly a thin line so ink % is always small.
    c_canvas_mm = canvas_mm / 2
    print("per-page space analysis:")
    for i, (page, t) in enumerate(zip(pages, _plan_tiles(
            canvas_mm, cut_radius_mm, args)[1]), start=1):
        h_px, w_px = page.shape[:2]
        total = h_px * w_px
        ink = int(np.sum(np.any(page < 250, axis=2)))
        # Circle center in page pixel coords:
        cx_page = int(round((args.page_margin + (c_canvas_mm - t['src_x0_mm']))
                            * px_per_mm))
        cy_page = int(round((args.page_margin + (c_canvas_mm - t['src_y0_mm']))
                            * px_per_mm))
        r_px = int(round(cut_radius_mm * px_per_mm))
        mask = np.zeros((h_px, w_px), dtype=np.uint8)
        cv2.circle(mask, (cx_page, cy_page), r_px, 255, -1)
        kept = int(np.sum(mask > 0))
        print(f"  page {i}: {100*ink/total:.1f}% ink, "
              f"{100*kept/total:.1f}% kept after cutting the circle "
              f"({100*(total-kept)/total:.1f}% will be trimmed off)")

    print(f"outputs in {args.out_dir}/:")
    print(f"  ring_full.png            (reference)")
    print(f"  page_NN.png              (print each at 100% / actual size)")
    print(f"  pages_all.pdf            (or print this PDF at 100%)")
    print(f"  preview_assembled.png    (all pages stitched at canvas centre)")
    print(f"  marker_layout.yaml       (feed to the vision node)")
    print("")
    print("assembly: trim the outer white margin off each page so the "
          "printed content reaches the edge, butt adjacent pages together "
          "so the solid circle and markers flow smoothly across the seam "
          "and tape the back, then cut around the solid black circle with "
          "scissors and apply to the platform with the center crosshair on "
          "the disk center.")


if __name__ == '__main__':
    sys.exit(main() or 0)
