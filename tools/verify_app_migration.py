"""Offline application migration checks. No hardware or real robot state polling.

Only explicitly listed offline tests are loaded, never test discovery over the
hardware scripts. USB device discovery and RealSense acquisition are blocked.
Loopback HTTP tests use synthetic/saved snapshots, never the running viewer.
"""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import difflib
import hashlib
import importlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
from unittest.mock import patch

PIPER = Path(__file__).resolve().parents[1]
WORKSPACE = PIPER.parent
APP = PIPER / 'piper_readonly'
LEGACY = WORKSPACE / 'piper_readonly'
AUDIT = PIPER / '.local/migration_app_20261009'
TESTS = [
    'test_web_monitor', 'test_tailnet_gateway', 'test_camera_pose', 'test_fitted_mount',
    'test_monitor', 'test_feedback_snapshot', 'test_feedback_timing', 'test_usb_receive',
    'test_runtime_resources', 'test_motion_supervisor', 'test_commission_piper',
    'test_commission_revalidation', 'test_gripper_manual', 'test_clear_piper_fault_once',
    'test_recover_piper_state_once', 'test_reset_piper_disabled_once',
    'test_disable_executor', 'test_disable_once_policy', 'test_disable_sdk_encoding',
    'test_disable_verification', 'test_probe_piper_stop_once', 'test_read_piper_joint_limits',
    'test_read_piper_identity_raw', 'test_check_piper_power', 'test_piper_launcher',
]
INTEGRATION_TESTS = [
    'test_table_live_mapping', 'test_robot_mapping_replay', 'test_vision_capture',
    'test_vision_motion_profile', 'test_print_joints',
    'test_archive_study', 'test_block_archive', 'test_delta_archive',
    'test_live_scan_review', 'test_regional_reader', 'test_repository_layout',
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def model_fingerprint(module):
    result = {}
    for firmware in ('S-V1.5-7', 'S-V1.6-3', None):
        model, assets = module.load_model(firmware)
        result[str(firmware)] = dict(model=model, asset_sha256={url: digest(p) for url, p in assets.items()})
    return result


def verify_layout(mode):
    import app_paths
    import commission_piper
    import gripper_manual
    import read_robot
    import robot_model
    import tailnet_gateway
    import web_monitor
    import wrist_fgng
    assert app_paths.WORKSPACE_ROOT == WORKSPACE
    assert commission_piper.INTERLOCK == gripper_manual.GATE == LEGACY / 'commissioning_interlock.json'
    assert not (APP / 'commissioning_interlock.json').exists(), 'Duplicate gate prohibited'
    assert read_robot.ROOT == robot_model.ROOT == wrist_fgng.ROOT == LEGACY
    assert tailnet_gateway.DELIVERY_ROOT == WORKSPACE / 'output'
    assert web_monitor.APP_ROOT == APP
    plan = json.loads((AUDIT / 'plan.json').read_text())
    for name in plan['modules']:
        module = importlib.import_module(Path(name).stem)
        assert Path(module.__file__).resolve() == APP / name, (name, module.__file__)
    for name, expected in json.loads((AUDIT / 'protected_hashes.json').read_text()).items():
        assert digest(LEGACY / name).lower() == expected.lower(), f'Protected file changed: {name}'
    # Compare the entire model, camera extrinsics and every mesh with the legacy
    # implementation BEFORE cutover; afterward use that preserved fingerprint.
    baseline = AUDIT / 'model_before.json'
    if mode == 'stage' and not baseline.exists():
        spec = importlib.util.spec_from_file_location('_model_before', LEGACY / 'robot_model.py')
        old = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(old)
        baseline.write_text(json.dumps(model_fingerprint(old), indent=2), encoding='utf-8')
    assert model_fingerprint(robot_model) == json.loads(baseline.read_text())
    if mode == 'post':
        for name in ('web', 'assets'):
            assert (LEGACY / name).resolve() == APP / name
    diffs = []
    for name in plan['modules']:
        before = (AUDIT / 'original_sources' / name).read_text(encoding='utf-8').splitlines(True)
        after = (APP / name).read_text(encoding='utf-8').splitlines(True)
        diffs.extend(difflib.unified_diff(before, after, fromfile='before/' + name, tofile='after/' + name))
    (AUDIT / 'application_path_changes.diff').write_text(''.join(diffs), encoding='utf-8')
    return dict(modules=len(plan['modules']), model_and_meshes_unchanged=True,
                authoritative_gate=str(commission_piper.INTERLOCK), shared_data_root=str(LEGACY))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('stage', 'post'), required=True)
    parser.add_argument('--integration', action='store_true', help='Also test offline capture, table plan and native mapping interfaces')
    parser.add_argument('--report-dir', type=Path, help='Write new test logs separately from the migration record')
    args = parser.parse_args()
    reports = args.report_dir.resolve() if args.report_dir else AUDIT
    reports.mkdir(parents=True, exist_ok=True)
    sys.path[:0] = [str(APP), str(LEGACY), str(PIPER)] if args.mode == 'stage' else [str(LEGACY), str(APP), str(PIPER)]
    sys.path[:0] = [str(PIPER / 'src'), str(PIPER / 'tests')]
    name = 'offline_staging_verified' if args.mode == 'stage' else 'offline_post_verified'
    stream = io.StringIO()
    report = dict(mode=args.mode, hardware_access=False, success=False,
                  checked_at=datetime.now(timezone.utc).isoformat())
    with ExitStack() as guards:
        guards.enter_context(patch('usb.core.find', side_effect=AssertionError('Offline tests may not discover USB hardware')))
        guards.enter_context(patch('pyrealsense2.pipeline', side_effect=AssertionError('Offline tests may not open RealSense')))
        report['layout'] = verify_layout(args.mode)
        suite = unittest.defaultTestLoader.loadTestsFromNames(TESTS + (INTEGRATION_TESTS if args.integration else []))
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
        # Check gate and path invariants again after all mocked maintenance tests.
        verify_layout(args.mode)
        report.update(success=result.wasSuccessful(), tests_run=result.testsRun,
                      errors=len(result.errors), failures=len(result.failures), skipped=len(result.skipped),
                      integration_included=args.integration)
    node = shutil.which('node')
    js = subprocess.run([node, '--test', *[f'web/{n}.test.js' for n in
        ('model', 'fgng', 'joint_monitor', 'trajectory', 'scan_trajectory', 'cad_preview')]],
        cwd=APP, env={**os.environ, 'PIPER_PYTHON': sys.executable},
        capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=60)
    (reports / (name + '_javascript.log')).write_text(js.stdout + js.stderr, encoding='utf-8')
    report['javascript_exit_code'] = js.returncode
    report['success'] = report['success'] and js.returncode == 0
    paths = list(APP.glob('*.py')) + list((APP / 'web').glob('*')) + list((APP / 'assets').rglob('*'))
    report['canonical_hashes'] = {str(p.relative_to(APP)): digest(p) for p in paths if p.is_file()}
    (reports / (name + '.log')).write_text(stream.getvalue(), encoding='utf-8')
    (reports / (name + '.json')).write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'canonical_hashes'}, indent=2))
    if not report['success']:
        print(stream.getvalue()[-18000:])
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
