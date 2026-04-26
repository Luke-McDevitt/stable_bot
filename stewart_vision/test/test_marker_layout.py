"""Schema + geometry tests for the bundled marker_layout.yaml.

If the printed ArUco ring is regenerated with different parameters,
these tests need to be updated to match. Catching mismatches between
the YAML and the controller's assumptions before deploying to the Pi.
"""
import os

import numpy as np
import pytest

from stewart_vision._aruco_helpers import MarkerLayout, load_marker_layout


def _layout_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), 'config', 'marker_layout.yaml')


@pytest.fixture(scope='module')
def layout() -> MarkerLayout:
    p = _layout_path()
    if not os.path.isfile(p):
        pytest.skip(f"marker_layout.yaml not present at {p}")
    return load_marker_layout(p)


def test_loads_as_dataclass(layout):
    assert isinstance(layout, MarkerLayout)


def test_marker_count_matches_spec(layout):
    """Spec §2 / generator default: 8 markers."""
    assert layout.num_markers == 8


def test_dictionary_is_4x4_50(layout):
    """Spec choice: DICT_4X4_50 — bigger inner cells for oblique views."""
    assert layout.dictionary_name == 'DICT_4X4_50'


def test_ring_radius_120mm(layout):
    assert layout.ring_radius_mm == pytest.approx(120.0)


def test_marker_size_50mm(layout):
    assert layout.marker_size_mm == pytest.approx(50.0)


def test_corners_are_in_meters_not_mm(layout):
    """The detector consumes meters; if anything is mm-sized it'll
    silently produce a 1000x scale error in solvePnP."""
    assert layout.corners_m.max() < 0.5      # < 0.5 m, not 500 mm


def test_all_corners_are_coplanar_at_z_zero(layout):
    """ArUco ring is on the platform top surface, z = 0 in platform frame."""
    np.testing.assert_array_equal(layout.corners_m[:, :, 2], 0.0)


def test_marker_centers_lie_on_120mm_ring(layout):
    """Each marker's centroid (mean of 4 corners) sits on the ring."""
    centers_m = layout.corners_m.mean(axis=1)
    radii_mm = np.hypot(centers_m[:, 0], centers_m[:, 1]) * 1000.0
    np.testing.assert_allclose(radii_mm, 120.0, atol=0.1)


def test_marker_corners_have_consistent_orientation(layout):
    """All marker quads should have the same handedness (cv2.aruco
    convention). Mixing CW and CCW orderings produces sign-flipped
    pose estimates."""
    xy = layout.corners_m[:, :, :2]

    def signed_area(quad):
        x = quad[:, 0]
        y = quad[:, 1]
        return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)

    signs = {np.sign(signed_area(xy[i])) for i in range(layout.num_markers)}
    assert len(signs) == 1, f"corner orderings inconsistent: signs={signs}"


def test_ids_are_zero_through_seven(layout):
    """Generator emits IDs 0..N-1 by default; many places assume this."""
    assert sorted(layout.ids.tolist()) == list(range(layout.num_markers))
