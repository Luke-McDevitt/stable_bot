"""Tests for stewart_vision._aruco_helpers — the YAML loader + dictionary
resolver shared by platform_pose_node and calibration_node.
"""
import os

import numpy as np
import pytest

from stewart_vision._aruco_helpers import (
    MarkerLayout,
    _DICT_NAME_TO_ATTR,
    build_board,
    get_aruco_dictionary,
    load_marker_layout,
)


def _layout_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), 'config', 'marker_layout.yaml')


@pytest.fixture(scope='module')
def layout() -> MarkerLayout:
    p = _layout_path()
    if not os.path.isfile(p):
        pytest.skip(f"marker_layout.yaml not present at {p}")
    return load_marker_layout(p)


def test_load_returns_marker_layout_dataclass(layout):
    assert isinstance(layout, MarkerLayout)
    assert layout.ids.dtype == np.int32
    assert layout.corners_m.dtype == np.float32


def test_corners_shape_is_n_by_4_by_3(layout):
    assert layout.corners_m.shape == (layout.num_markers, 4, 3)


def test_known_dictionaries_resolve_to_attrs():
    cv2 = pytest.importorskip('cv2')
    if not hasattr(cv2, 'aruco'):
        pytest.skip("opencv-contrib-python not installed (no cv2.aruco)")
    for name in _DICT_NAME_TO_ATTR:
        layout = MarkerLayout(
            platform_diameter_mm=400.0,
            ring_radius_mm=120.0,
            marker_size_mm=50.0,
            dictionary_name=name,
            ids=np.zeros(1, dtype=np.int32),
            corners_m=np.zeros((1, 4, 3), dtype=np.float32),
        )
        d = get_aruco_dictionary(layout)
        assert d is not None


def test_unknown_dictionary_raises_value_error():
    cv2 = pytest.importorskip('cv2')
    if not hasattr(cv2, 'aruco'):
        pytest.skip("opencv-contrib-python not installed (no cv2.aruco)")
    bogus = MarkerLayout(
        platform_diameter_mm=400.0,
        ring_radius_mm=120.0,
        marker_size_mm=50.0,
        dictionary_name='DICT_NOT_REAL',
        ids=np.zeros(1, dtype=np.int32),
        corners_m=np.zeros((1, 4, 3), dtype=np.float32),
    )
    with pytest.raises(ValueError):
        get_aruco_dictionary(bogus)


def test_build_board_runs_without_error(layout):
    cv2 = pytest.importorskip('cv2')
    if not hasattr(cv2, 'aruco'):
        pytest.skip("opencv-contrib-python not installed (no cv2.aruco)")
    board = build_board(layout)
    assert board is not None


def test_marker_corners_box_size_matches_marker_size(layout):
    """For each marker, the bounding box of its 4 corners should be
    marker_size_mm × marker_size_mm. Catches errors in the half-side
    arithmetic in load_marker_layout()."""
    s_m = layout.marker_size_mm * 1e-3
    for i in range(layout.num_markers):
        c = layout.corners_m[i]   # (4, 3)
        bbox_x = c[:, 0].max() - c[:, 0].min()
        bbox_y = c[:, 1].max() - c[:, 1].min()
        np.testing.assert_allclose(bbox_x, s_m, rtol=1e-6)
        np.testing.assert_allclose(bbox_y, s_m, rtol=1e-6)
