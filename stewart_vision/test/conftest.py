"""Pytest fixtures + sys.path setup for stewart_vision tests.

Tests live at stewart_vision/test/ and import from stewart_vision/stewart_vision/.
Adding the package root to sys.path makes `import stewart_vision._aruco_helpers`
work when running `pytest` from anywhere in the workspace.
"""
import os
import sys

# stewart_vision/ — parent of stewart_vision/stewart_vision/
PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)
