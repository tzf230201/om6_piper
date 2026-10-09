"""Compatibility CLI. Canonical trajectory code is now src/trajectory_test.py.

Importing this adapter does not import or execute the hardware trajectory.
"""
from pathlib import Path
import runpy
import sys

if __name__ == '__main__':
    source = Path(__file__).resolve().parent / 'src'
    sys.path.insert(0, str(source))
    runpy.run_path(str(source / 'trajectory_test.py'), run_name='__main__')
