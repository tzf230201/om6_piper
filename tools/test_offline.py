"""Run explicit offline suites. Not a hardware commissioning or motion command."""
from datetime import datetime
from pathlib import Path
import sys
import verify_app_migration

if __name__ == '__main__':
    if len(sys.argv) > 1:
        raise SystemExit('Usage: piper.cmd test (no hardware options accepted)')
    reports = Path(__file__).resolve().parents[1] / '.local/test_runs' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    sys.argv += ['--mode', 'post', '--integration', '--report-dir', str(reports)]
    print('Offline report directory:', reports, flush=True)
    raise SystemExit(verify_app_migration.main())
