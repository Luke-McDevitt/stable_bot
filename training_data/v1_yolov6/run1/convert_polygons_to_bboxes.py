#!/usr/bin/env python3
"""Convert Roboflow polygon-format labels to YOLO bounding-box format.

Roboflow's Smart Polygon labeling exports labels as:
    class_id  x0 y0  x1 y1  x2 y2  ...  xN yN

(N polygon vertices, all normalized [0, 1].) Standard YOLOv6 detection
training expects the bbox-only format:
    class_id  cx cy  w h

(5 values per line, all normalized.) Converts in-place — overwrites
the original `labels/` directory. Run once before training.

Usage:
    python3 convert_polygons_to_bboxes.py [--dry-run]

The conversion takes the axis-aligned bounding box of each polygon
(min/max of vertex x and y), which matches what a detection model
would predict regardless of how the label was drawn. The polygon
detail is discarded.
"""
from __future__ import annotations

import argparse
from pathlib import Path


def polygon_to_bbox(coords: list[float]) -> tuple[float, float, float, float]:
    xs = coords[0::2]
    ys = coords[1::2]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    cx = (x_min + x_max) / 2.0
    cy = (y_min + y_max) / 2.0
    w = x_max - x_min
    h = y_max - y_min
    return cx, cy, w, h


def convert_one(path: Path, dry_run: bool) -> tuple[int, int]:
    """Returns (n_polygon_lines, n_already_bbox_lines)."""
    out_lines = []
    n_poly = 0
    n_bbox = 0
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        cls = parts[0]
        rest = parts[1:]
        if len(rest) == 4:
            # Already in bbox format — keep as-is.
            out_lines.append(line)
            n_bbox += 1
        elif len(rest) >= 6 and len(rest) % 2 == 0:
            # Polygon: class_id followed by an even number of coords.
            coords = [float(v) for v in rest]
            cx, cy, w, h = polygon_to_bbox(coords)
            out_lines.append(
                f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
            n_poly += 1
        else:
            print(f"  ! skipping malformed line in {path.name}: {line[:60]}…")
    if not dry_run and n_poly > 0:
        path.write_text("\n".join(out_lines) + "\n")
    return n_poly, n_bbox


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true",
                    help="Don't write — just report what would change.")
    args = ap.parse_args()

    here = Path(__file__).parent
    labels_dir = here / "labels"
    if not labels_dir.is_dir():
        raise SystemExit(f"labels/ not found at {labels_dir}")

    total_poly = 0
    total_bbox = 0
    n_files = 0
    for split_dir in sorted(labels_dir.iterdir()):
        if not split_dir.is_dir():
            continue
        for txt in sorted(split_dir.glob("*.txt")):
            n_files += 1
            poly, bbox = convert_one(txt, args.dry_run)
            total_poly += poly
            total_bbox += bbox

    action = "would convert" if args.dry_run else "converted"
    print(f"\nProcessed {n_files} label files.")
    print(f"  polygon lines {action}: {total_poly}")
    print(f"  bbox lines preserved:    {total_bbox}")


if __name__ == "__main__":
    main()
