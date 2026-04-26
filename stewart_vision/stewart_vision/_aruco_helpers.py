"""Shared helpers for loading the ArUco marker layout and constructing
an `cv2.aruco.Board` ready for `solvePnP` / `estimatePoseBoard`.

The marker_layout.yaml is produced by stewart_bringup/scripts/generate_aruco_ring.py.
Schema (see that script for source of truth):

    platform_diameter_mm: 400.0
    ring_radius_mm: 120.0
    dictionary: DICT_4X4_50
    marker_size_mm: 50.0
    marker_count: 8
    markers:
      - id: 0
        x_mm: 0.0
        y_mm: 120.0
        size_mm: 50.0
      ...
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import yaml


_DICT_NAME_TO_ATTR = {
    'DICT_4X4_50':   'DICT_4X4_50',
    'DICT_4X4_100':  'DICT_4X4_100',
    'DICT_4X4_250':  'DICT_4X4_250',
    'DICT_5X5_50':   'DICT_5X5_50',
    'DICT_5X5_100':  'DICT_5X5_100',
    'DICT_6X6_50':   'DICT_6X6_50',
    'DICT_6X6_250':  'DICT_6X6_250',
}


@dataclass
class MarkerLayout:
    platform_diameter_mm: float
    ring_radius_mm: float
    marker_size_mm: float
    dictionary_name: str
    ids: np.ndarray         # shape (N,)
    corners_m: np.ndarray   # shape (N, 4, 3) — corners in PLATFORM frame, METERS

    @property
    def num_markers(self) -> int:
        return int(self.ids.shape[0])


def load_marker_layout(path: str) -> MarkerLayout:
    """Parse marker_layout.yaml into a MarkerLayout with corners in METERS
    in the platform frame. Z = 0 for all corners (markers are coplanar
    on the top plate).
    """
    with open(path, 'r') as f:
        d = yaml.safe_load(f)

    s_mm = float(d['marker_size_mm'])
    half = s_mm / 2.0

    ids: List[int] = []
    corners_mm: List[np.ndarray] = []
    for m in d['markers']:
        x = float(m['x_mm'])
        y = float(m['y_mm'])
        # Order: TL, TR, BR, BL (cv2.aruco convention is the marker's own
        # corner-0 at top-left when viewed in image, then clockwise).
        # In platform frame with +y "up" we use:
        c = np.array([
            [x - half, y + half, 0.0],
            [x + half, y + half, 0.0],
            [x + half, y - half, 0.0],
            [x - half, y - half, 0.0],
        ], dtype=np.float32)
        ids.append(int(m['id']))
        corners_mm.append(c)

    corners_m = (np.stack(corners_mm, axis=0) * 1e-3).astype(np.float32)
    return MarkerLayout(
        platform_diameter_mm=float(d['platform_diameter_mm']),
        ring_radius_mm=float(d['ring_radius_mm']),
        marker_size_mm=s_mm,
        dictionary_name=str(d['dictionary']),
        ids=np.asarray(ids, dtype=np.int32),
        corners_m=corners_m,
    )


def get_aruco_dictionary(layout: MarkerLayout):
    """Resolve the layout's dictionary string to a cv2.aruco predefined
    dictionary, raising a clear error if cv2 lacks aruco support.
    """
    try:
        import cv2
    except ImportError as e:
        raise ImportError(
            "cv2 not installed; install opencv-contrib-python") from e
    if not hasattr(cv2, 'aruco'):
        raise ImportError(
            "cv2.aruco not available; install opencv-contrib-python "
            "(NOT plain opencv-python — aruco is in contrib)")
    attr = _DICT_NAME_TO_ATTR.get(layout.dictionary_name)
    if attr is None:
        raise ValueError(f"Unsupported aruco dictionary: {layout.dictionary_name}")
    # Newer API: getPredefinedDictionary; older API: Dictionary_get.
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, attr))
    return cv2.aruco.Dictionary_get(getattr(cv2.aruco, attr))


def build_board(layout: MarkerLayout):
    """Build a cv2.aruco.Board from the layout."""
    import cv2
    aruco_dict = get_aruco_dictionary(layout)
    obj_points = [layout.corners_m[i] for i in range(layout.num_markers)]
    if hasattr(cv2.aruco, 'Board') and hasattr(cv2.aruco.Board, 'create'):
        return cv2.aruco.Board.create(obj_points, aruco_dict, layout.ids)
    # OpenCV 4.7+ uses constructor directly.
    return cv2.aruco.Board(obj_points, aruco_dict, layout.ids)
