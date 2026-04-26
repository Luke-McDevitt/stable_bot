#!/usr/bin/env python3
"""generate_chessboard.py — print-ready chessboard for OAK-D calibration.

Default produces a 10x7-square chessboard (9x6 INTERNAL corners, the
shape that `cv2.findChessboardCorners(..., (9, 6))` expects) with
25 mm squares — the size assumed by `calibrate_oak.py` Stage A.

The output is sized to fit on US letter LANDSCAPE at 100 % print scale.
For the carbon-fiber + matte-vinyl sticker workflow:

  1. Print the PDF at 100 % (do NOT "fit to page").
  2. Verify with a ruler: a square should measure exactly 25.0 mm.
  3. Stick the vinyl onto the carbon-fiber backer; smooth out bubbles
     working from the center outward.
  4. Trim to the registration crop marks.

Usage:
    python3 generate_chessboard.py
    python3 generate_chessboard.py --cols-squares 10 --rows-squares 7
    python3 generate_chessboard.py --square-mm 30 --out-dir ./bigger_board

Outputs (in --out-dir):
    chessboard.png       high-res render at --dpi
    chessboard.pdf       printable, dpi-correct
    chessboard.yaml      metadata for calibrate_oak.py

Spec: ../../stewart_bringup/docs/closed_loop_ball_demos.md §6 Stage A.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml

try:
    import cv2
except ImportError:
    sys.exit("opencv-python required. `pip install opencv-python` "
             "(or apt: ros-kilted-cv-bridge already pulls it in).")


# US letter dimensions (mm)
LETTER_SHORT_MM = 215.9
LETTER_LONG_MM = 279.4


def render_chessboard(cols_sq: int, rows_sq: int, square_mm: float,
                      dpi: int, margin_mm: float, crop_marks: bool):
    """Render the chessboard onto a white canvas of size
    (cols_sq * square_mm + 2*margin) x (rows_sq * square_mm + 2*margin) mm.
    Returns (img_bgr, sheet_w_mm, sheet_h_mm).
    """
    px_per_mm = dpi / 25.4
    sheet_w_mm = cols_sq * square_mm + 2 * margin_mm
    sheet_h_mm = rows_sq * square_mm + 2 * margin_mm

    w_px = int(round(sheet_w_mm * px_per_mm))
    h_px = int(round(sheet_h_mm * px_per_mm))
    canvas = np.full((h_px, w_px, 3), 255, dtype=np.uint8)

    sq_px = int(round(square_mm * px_per_mm))
    m_px = int(round(margin_mm * px_per_mm))

    for r in range(rows_sq):
        for c in range(cols_sq):
            if (r + c) % 2 == 0:
                x0 = m_px + c * sq_px
                y0 = m_px + r * sq_px
                x1 = x0 + sq_px
                y1 = y0 + sq_px
                canvas[y0:y1, x0:x1] = (0, 0, 0)

    if crop_marks:
        # Tiny L-shaped crop marks at each corner of the chessboard so
        # the user knows exactly where to cut after sticking the vinyl
        # to the carbon-fiber backer.
        mark_len_px = int(round(5 * px_per_mm))   # 5 mm tick
        mark_thick_px = max(1, int(round(0.3 * px_per_mm)))
        for cx, cy in [(m_px, m_px),
                       (m_px + cols_sq * sq_px, m_px),
                       (m_px, m_px + rows_sq * sq_px),
                       (m_px + cols_sq * sq_px, m_px + rows_sq * sq_px)]:
            cv2.line(canvas, (cx - mark_len_px, cy), (cx, cy),
                     (0, 0, 0), mark_thick_px)
            cv2.line(canvas, (cx, cy - mark_len_px), (cx, cy),
                     (0, 0, 0), mark_thick_px)

    return canvas, sheet_w_mm, sheet_h_mm


def add_caption(img, cols_sq, rows_sq, square_mm, dpi, sheet_w_mm,
                sheet_h_mm, dpi_per_mm):
    text_lines = [
        f"chessboard {cols_sq}x{rows_sq} squares  |  "
        f"{int(cols_sq) - 1}x{int(rows_sq) - 1} internal corners  |  "
        f"square = {square_mm:.1f} mm  |  {dpi} dpi",
        f"sheet = {sheet_w_mm:.1f} x {sheet_h_mm:.1f} mm  |  "
        f"PRINT AT 100% (do not fit-to-page) — verify a square = "
        f"{square_mm:.1f} mm with a ruler before sticking",
    ]
    y = img.shape[0] - int(round(8 * dpi_per_mm))
    for line in reversed(text_lines):
        cv2.putText(img, line, (int(round(5 * dpi_per_mm)), y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (90, 90, 90), 1, cv2.LINE_AA)
        y -= int(round(4 * dpi_per_mm))
    return img


def fits_on_letter(w_mm: float, h_mm: float):
    """Return ('portrait'|'landscape'|'no', w_fit, h_fit)."""
    if w_mm <= LETTER_SHORT_MM and h_mm <= LETTER_LONG_MM:
        return 'portrait', LETTER_SHORT_MM, LETTER_LONG_MM
    if w_mm <= LETTER_LONG_MM and h_mm <= LETTER_SHORT_MM:
        return 'landscape', LETTER_LONG_MM, LETTER_SHORT_MM
    return 'no', None, None


def write_pdf(img_bgr, pdf_path: str, dpi: int):
    """Save the chessboard as a single-page PDF at the requested DPI so
    that "Print at 100 %" produces the physical size set by --dpi.
    """
    try:
        from PIL import Image
    except ImportError:
        print("[warn] Pillow not installed; skipping PDF "
              "(install with `pip install Pillow`).")
        return False
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    pim = Image.fromarray(rgb)
    pim.save(pdf_path, 'PDF', resolution=float(dpi))
    return True


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--cols-squares', type=int, default=10,
                   help='Chessboard width in SQUARES (default: 10 → 9 internal corners).')
    p.add_argument('--rows-squares', type=int, default=7,
                   help='Chessboard height in SQUARES (default: 7 → 6 internal corners).')
    p.add_argument('--square-mm', type=float, default=25.0,
                   help='Edge length of a single square, mm (default: 25.0).')
    p.add_argument('--dpi', type=int, default=300,
                   help='Output rasterization DPI (default: 300).')
    p.add_argument('--margin-mm', type=float, default=10.0,
                   help='White quiet zone outside the chessboard, mm (default: 10).')
    p.add_argument('--no-crop-marks', action='store_true',
                   help='Suppress the corner L-marks.')
    p.add_argument('--out-dir', default='chessboard_out',
                   help='Output directory (default: chessboard_out).')
    args = p.parse_args()

    if args.cols_squares < 4 or args.rows_squares < 4:
        sys.exit("error: cols_squares and rows_squares must each be >= 4.")
    if args.cols_squares == args.rows_squares:
        print("[warn] Square boards are ambiguous to OpenCV's chessboard "
              "detector (no unique orientation). Recommend asymmetric.")

    os.makedirs(args.out_dir, exist_ok=True)

    img, sheet_w_mm, sheet_h_mm = render_chessboard(
        args.cols_squares, args.rows_squares,
        args.square_mm, args.dpi, args.margin_mm,
        crop_marks=not args.no_crop_marks)

    fit, _, _ = fits_on_letter(sheet_w_mm, sheet_h_mm)
    if fit == 'no':
        print(f"[warn] Sheet is {sheet_w_mm:.1f} x {sheet_h_mm:.1f} mm — "
              f"exceeds letter (8.5\"x11\"). Print on legal/tabloid, or "
              f"reduce --square-mm or square count.")
    elif fit == 'landscape':
        print(f"[info] Sheet fits letter LANDSCAPE only "
              f"({sheet_w_mm:.1f} x {sheet_h_mm:.1f} mm).")

    add_caption(img, args.cols_squares, args.rows_squares,
                args.square_mm, args.dpi, sheet_w_mm, sheet_h_mm,
                args.dpi / 25.4)

    png_path = os.path.join(args.out_dir, 'chessboard.png')
    pdf_path = os.path.join(args.out_dir, 'chessboard.pdf')
    yaml_path = os.path.join(args.out_dir, 'chessboard.yaml')

    cv2.imwrite(png_path, img)
    pdf_ok = write_pdf(img, pdf_path, args.dpi)

    meta = {
        'cols_squares': int(args.cols_squares),
        'rows_squares': int(args.rows_squares),
        'cols_internal_corners': int(args.cols_squares) - 1,
        'rows_internal_corners': int(args.rows_squares) - 1,
        'square_mm': float(args.square_mm),
        'dpi': int(args.dpi),
        'margin_mm': float(args.margin_mm),
        'sheet_size_mm': [float(sheet_w_mm), float(sheet_h_mm)],
        'opencv_pattern_size_for_findChessboardCorners':
            [int(args.cols_squares) - 1, int(args.rows_squares) - 1],
    }
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)

    print(f"wrote {png_path}")
    if pdf_ok:
        print(f"wrote {pdf_path}")
    print(f"wrote {yaml_path}")
    print(f"\nsheet: {sheet_w_mm:.1f} x {sheet_h_mm:.1f} mm  |  "
          f"internal corners: "
          f"{args.cols_squares - 1} x {args.rows_squares - 1}  |  "
          f"square: {args.square_mm:.1f} mm")
    print("Print at 100% scale; verify with a ruler before sticking.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
