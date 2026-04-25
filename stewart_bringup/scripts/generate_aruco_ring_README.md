# generate_aruco_ring.py

Generates the Stable-Bot platform's ArUco-marker ring as a printable
sticker design: full ring + tiled letter pages + per-marker layout YAML.
The assembled print sticks onto the platform with markers at known
positions for the vision node.

## Quick start

```
python3 generate_aruco_ring.py \
  --platform-diameter 400 \
  --ring-radius 120 \
  --num-markers 8 \
  --marker-size 50 \
  --seam-bleed 3 \
  --seam-alignment-line \
  --center-circle-diameter 3 \
  --out-dir ./ring_out
```

## Outputs

| File | Purpose |
|---|---|
| `ring_full.png` | Full design at print DPI for visual reference. |
| `page_NN.png`   | Each tile on its own letter page. |
| `pages_all.pdf` | All pages bundled into one PDF. |
| `preview_assembled.png` | All pages stitched together with their canvas-centres aligned — a quick sanity check that the tiling and rotations make logical sense before you commit to printing. Each page has a coloured outline + `pN` label so you can see which piece is which. |
| `marker_layout.yaml` | Per-marker `(id, x_mm, y_mm, size_mm)` for the vision node. |

## Flags

### Geometry

- `--platform-diameter MM` — diameter of the platform the ring sticks
  onto. The cut-ring (the solid black circle to follow with scissors)
  matches this exactly so the print covers the platform edge-to-edge.
- `--ring-radius MM` — distance from platform centre to each marker's
  centre.
- `--num-markers N` — markers around the ring.
- `--marker-size MM` — outer side length of each ArUco square.
- `--marker-border MM` — minimum quiet-zone margin between any marker
  corner and the cut ring. The script warns if a configuration would
  violate this.
- `--first-id N` — starting ArUco ID; subsequent markers increment.
- `--dict NAME` — ArUco dictionary (default `DICT_4X4_50`).

### Page tiling

- `--page-margin MM` — printer safe-area margin per page side
  (default 5). Larger margins can push the design onto more pages.
- `--page-overlap MM` — overlap between adjacent pages (default 0,
  no content duplication). Raise for redundant content along the seams
  if you prefer eyeballing alignment by content rather than by guides.
- `--cut-margin MM` — extra mm beyond `platform_diameter / 2` for the
  cut ring (default 0).
- `--seam-bleed MM` — if > 0, draws a long-dashed outer cut line on
  each page seam edge, outset by this many mm from the tile boundary
  into the printer margin. Cutting on the dashes leaves a small bleed
  strip past the content for taping/overlap.
- `--seam-alignment-line` — only meaningful with `--seam-bleed > 0`:
  also draws a short-dashed inner alignment line, mirrored across the
  outer line by the same distance the outer line sits from the page
  edge. The inner line is **clipped to the area outside the cut-ring**
  so it never crosses the marker quiet zone — that way it can't
  interfere with ArUco detection on the kept piece. See *Sticker
  assembly procedure* below for the intended use.
- `--center-circle-diameter MM` — if > 0, draws a faint grey circle
  on every page around where the platform centre will land. Drawn
  per-page (not on the shared canvas) so the full circle survives even
  though the canvas centre lands on a tile corner. Pair with `3` to
  fit a 3 mm hole punch + M3 bolt during assembly.
- `--canvas-center-corner {natural, top-left, top-right, bottom-left,
  bottom-right}` — land the platform centre in the same corner of
  every output page so printer margin asymmetry affects each page
  identically. In uniform mode each tile is **square** (canvas/2 per
  side), and a square tile stays square under 90° rotation, so every
  page ends up in the same landscape orientation. Each page's top
  label states the CW rotation you need to apply when assembling
  (around the M3 bolt), which cancels the print rotation and puts
  every marker back at its natural platform-frame orientation.
  Default `natural` (rectangular tiles, no rotation). Requires
  canvas/2 to fit a letter-landscape usable area — if canvas/2 is
  larger, the script errors out and you need a smaller
  `--platform-diameter` or `--page-margin`. `--seam-bleed` and
  `--seam-alignment-line` still apply in uniform mode; the dashed
  lines are drawn on the two tile edges that meet at the canvas-
  centre corner (i.e. the seams between adjacent quadrants once the
  four pages are assembled on the platform).

### Output

- `--dpi N` — output DPI (default 300).
- `--out-dir PATH` — output directory. Auto-fixes Windows-style
  backslashes and expands `~`.

## Sticker assembly procedure (M3-bolt method)

This is one workflow that uses the alignment features end-to-end. Run
with `--center-circle-diameter 3 --seam-bleed 3 --seam-alignment-line`
and:

1. Print the pages at 100% (NOT "fit to page").
2. Use a 3 mm hole punch to punch out the small grey circle at the
   centre of each page.
3. Pass an M3 bolt through the punched holes of all four pages so they
   share a common centre.
4. Line up the factory edge of one page so it completely covers the
   inner short-dashed line on the adjacent page.
5. Clamp both pages down to the platform once aligned.
6. Cut the excess paper around the cut-ring (the solid black circle)
   with scissors.
7. Peel off some of the sticker backing.
8. Stick each page down partially.
9. Repeat steps 4–8 with the other two pages.
10. Once all four pages are partially stuck around the edges, remove
    the M3 bolt and peel back the rest of the sticker backing except
    for the seam-overlap regions.
11. Use a scalpel or utility knife to cut along the outer (long-dashed)
    line on each page; repeat for both seam sides of every page.
12. Remove the remaining backing and stick the cut edges down for the
    final layup.

## Geometric note

If you use `--seam-alignment-line` and follow step 4 verbatim, the
adjacent page's factory edge ends up at the inner-dashed line, which
sits `(page-margin − seam-bleed)` mm inside the outer cut line. The
M3 bolt at the platform centre simultaneously fixes each page's
centre — and those two constraints can't both be satisfied snugly:
the natural edge position with a snug bolt is `page-margin` mm from
the bolt, while the inner-dashed line lands `(2 × seam-bleed −
page-margin)` mm past the seam. The difference is `2 × seam-bleed`
mm (e.g. 6 mm with `--seam-bleed 3`).

Practical implications:

- The cut-ring (solid black circle) ends up with `2 × seam-bleed` mm
  breaks at each of the four seams. For sticker placement that's
  cosmetic — the scissors cut takes a smooth path anyway.
- Any marker whose bounding box straddles a seam splits by the same
  shift, which destroys the marker pattern. For
  `--num-markers 8` with the default starting angle the four axial
  markers (`id 0, 2, 4, 6`) land on the cardinal axes and are at
  risk; the four diagonals (`id 1, 3, 5, 7`) are safe.

Two ways to avoid the marker hit:

1. Use a marker count where no marker lands on a cardinal axis (e.g.
   any `--num-markers` value where `360 / N` is not a divisor of 90,
   such as 6, 10, 12, 14). The diagonals-only 4-marker case also
   works.
2. Drop `--seam-alignment-line` and rely on the M3 bolt alone for
   centring; pages naturally butt at the seams (10 mm of margin
   overlap, no content shift) and the cut-ring + every marker stay
   continuous.
