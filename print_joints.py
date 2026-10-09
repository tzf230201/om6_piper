"""Compatibility entry; implementation lives in src/print_joints.py."""
from pathlib import Path
import sys

_source = Path(__file__).resolve().parent / 'src'
sys.path.insert(0, str(_source))
__file__ = str(_source / 'print_joints.py')
exec(compile(Path(__file__).read_bytes(), __file__, 'exec'), globals(), globals())
