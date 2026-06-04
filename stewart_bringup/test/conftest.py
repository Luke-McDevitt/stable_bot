"""Pytest sys.path setup for stewart_bringup tests.

Tests live at stewart_bringup/test/ and import from
stewart_bringup/stewart_bringup/. Adding the package root to sys.path
makes `import stewart_bringup._latency` work when running `pytest`
from anywhere in the workspace.
"""
import os
import sys

# stewart_bringup/ — parent of stewart_bringup/stewart_bringup/
PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)
