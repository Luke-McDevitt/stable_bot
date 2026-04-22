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


def build_ring(args):
    mm_per_in = 25.4
    px_per_mm = args.dpi / mm_per_in

    # Canvas sized to fit the outermost edge of the markers plus a small buffer.
    outer_mm = args.ring_radius + args.marker_size / 2 + args.marker_border
    canvas_mm = 2 * (outer_mm + 5)
    canvas_px = int(round(canvas_mm * px_per_mm))
    canvas = np.ones((canvas_px, canvas_px, 3), dtype=np.uint8) * 255

    aruco_dict = _get_dictionary(args.dict)
    marker_px = int(round(args.marker_size * px_per_mm))

    layout = []
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

        layout.append({
            'id': marker_id,
            'x_mm': round(x_mm, 3),
            'y_mm': round(y_mm, 3),
            'size_mm': args.marker_size,
        })

    # Center crosshair + platform boundary for alignment.
    c = canvas_px // 2
    tick_px = int(round(5 * px_per_mm))
    cv2.line(canvas, (c - tick_px, c), (c + tick_px, c), (150, 150, 150), 2)
    cv2.line(canvas, (c, c - tick_px), (c, c + tick_px), (150, 150, 150), 2)
    cv2.circle(canvas, (c, c),
               int(round(args.platform_diameter / 2 * px_per_mm)),
               (200, 200, 200), 1)

    return canvas, canvas_mm, layout, px_per_mm


def tile_pages(canvas, canvas_mm, args, px_per_mm):
    """Split canvas into letter-size pages with overlap + registration marks."""
    page_w_mm = 8.5 * 25.4    # 215.9
    page_h_mm = 11.0 * 25.4   # 279.4
    margin_mm = args.page_margin
    usable_w_mm = page_w_mm - 2 * margin_mm
    usable_h_mm = page_h_mm - 2 * margin_mm
    stride_w_mm = usable_w_mm - args.page_overlap
    stride_h_mm = usable_h_mm - args.page_overlap

    cols = max(1, math.ceil((canvas_mm - args.page_overlap) / stride_w_mm))
    rows = max(1, math.ceil((canvas_mm - args.page_overlap) / stride_h_mm))

    page_w_px = int(round(page_w_mm * px_per_mm))
    page_h_px = int(round(page_h_mm * px_per_mm))
    margin_px = int(round(margin_mm * px_per_mm))
    usable_w_px = int(round(usable_w_mm * px_per_mm))
    usable_h_px = int(round(usable_h_mm * px_per_mm))
    tick_px = int(round(3 * px_per_mm))

    canvas_px = canvas.shape[0]
    pages = []
    page_num = 0
    for row in range(rows):
        for col in range(cols):
            page_num += 1
            page = np.ones((page_h_px, page_w_px, 3), dtype=np.uint8) * 255

            src_x0 = int(round(col * stride_w_mm * px_per_mm))
            src_y0 = int(round(row * stride_h_mm * px_per_mm))
            src_x1 = min(src_x0 + usable_w_px, canvas_px)
            src_y1 = min(src_y0 + usable_h_px, canvas_px)
            cw = max(0, src_x1 - src_x0)
            ch = max(0, src_y1 - src_y0)
            if cw > 0 and ch > 0 and src_x0 < canvas_px and src_y0 < canvas_px:
                page[margin_px:margin_px + ch,
                     margin_px:margin_px + cw] = canvas[src_y0:src_y1,
                                                        src_x0:src_x1]

            # Registration ticks at the four corners of the usable area so
            # adjacent pages can be aligned after cutting.
            rx0, ry0 = margin_px, margin_px
            rx1, ry1 = margin_px + usable_w_px, margin_px + usable_h_px
            for (x, y) in [(rx0, ry0), (rx1, ry0), (rx0, ry1), (rx1, ry1)]:
                cv2.line(page, (x - tick_px, y), (x + tick_px, y),
                         (0, 0, 0), 1)
                cv2.line(page, (x, y - tick_px), (x, y + tick_px),
                         (0, 0, 0), 1)

            label = (f"page {page_num}  (row {row + 1}/{rows}, "
                     f"col {col + 1}/{cols})   |   print at 100%, "
                     "NOT 'fit to page'")
            cv2.putText(page, label, (margin_px, max(20, margin_px - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1,
                        cv2.LINE_AA)
            pages.append(page)

    return pages, cols, rows


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
    p.add_argument('--page-margin', type=float, default=10.0,
                   help='printer safe-area margin per side in mm')
    p.add_argument('--page-overlap', type=float, default=10.0,
                   help='overlap between adjacent pages in mm for alignment')
    p.add_argument('--out-dir', default='aruco_ring',
                   help='output directory')
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    canvas, canvas_mm, layout, px_per_mm = build_ring(args)

    full_png = os.path.join(args.out_dir, 'ring_full.png')
    Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)).save(
        full_png, dpi=(args.dpi, args.dpi))

    pages, cols, rows = tile_pages(canvas, canvas_mm, args, px_per_mm)
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

    print(f"generated {args.num_markers} markers "
          f"(IDs {args.first_id}..{args.first_id + args.num_markers - 1})")
    print(f"ring canvas: {canvas_mm:.1f} mm square at {args.dpi} DPI")
    print(f"tiled into {len(pages)} letter page(s) ({cols} cols x {rows} rows)")
    print(f"outputs in {args.out_dir}/:")
    print(f"  ring_full.png       (reference)")
    print(f"  page_NN.png         (print each at 100% / actual size)")
    print(f"  pages_all.pdf       (or print this PDF at 100%)")
    print(f"  marker_layout.yaml  (feed to the vision node)")
    print("")
    print("assembly: cut along the registration ticks, align adjacent pages "
          "by their overlap, tape/stick, apply to platform with the center "
          "crosshair on the disk center.")


if __name__ == '__main__':
    sys.exit(main() or 0)
