"""Application paths; shared archives and control state have ONE legacy owner.

Importing this module only resolves paths. It does not copy state, grant a
permit, create a directory, connect a device, or start a process.
"""
from pathlib import Path
import sys

APP_ROOT = Path(__file__).resolve().parent
PIPER_ROOT = APP_ROOT.parent
WORKSPACE_ROOT = PIPER_ROOT.parent
LEGACY_ROOT = WORKSPACE_ROOT / 'piper_readonly'
INTERLOCK_PATH = LEGACY_ROOT / 'commissioning_interlock.json'

# Mapping algorithms/tests/SDK environment remain shared during stage 1.
# Prefer canonical application modules; consult the legacy tree only afterward.
if str(LEGACY_ROOT) not in sys.path:
    sys.path.append(str(LEGACY_ROOT))
