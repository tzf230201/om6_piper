"""Only import search paths; never discover or connect hardware."""
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
for path in (REPO / 'src', REPO, REPO.parent / 'piper_readonly'):
    if str(path) not in sys.path:
        sys.path.append(str(path))
