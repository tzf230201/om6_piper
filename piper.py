"""Entry point for the consolidated Piper workspace; default is an offline check.

Commands delegate to the existing programs. No automatic enable, reset, clear,
retry, interlock edit, USB connection or camera acquisition occurs on import.
"""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

PIPER_ROOT=Path(__file__).resolve().parent
WORKSPACE_ROOT=PIPER_ROOT.parent
PROGRAMS={
    'gui':('piper_readonly/web_monitor.py','Robot mesh viewer and joint monitor; opens hardware unless --snapshot is supplied.'),
    'state':('piper_readonly/read_robot.py','Read joint/controller/driver/gripper state; sends firmware queries, no motor commands.'),
    'joints':('src/print_joints.py','Print joints at nominal 50 Hz; default reads an existing viewer.'),
    'commission':('piper_readonly/commission_piper.py','Existing commissioning program; default preflight, guarded explicit execution only.'),
    'clear-error':('piper_readonly/clear_piper_fault_once.py','Existing one-shot fault-clear policy; no automatic enable or motion.'),
    'recover':('piper_readonly/recover_piper_state_once.py','Existing guarded reset/clear policy; retain its checks and explicit flags.'),
    'disable':('piper_readonly/disable_piper_once.py','Existing disable policy; preserve explicit execution and support confirmation.'),
    'scan':('src/trajectory_test.py','PHYSICAL MOTION after confirmation; existing trajectory/vision program.'),
    'preview-table':('src/table_scan.py','Offline tabletop trajectory planning; no collision certification.'),
    'gateway':('piper_readonly/tailnet_gateway.py','Existing read-only gateway; does not configure Tailscale or connect to CAN.'),
    'test':('tools/test_offline.py','Explicit offline regression suites; USB/camera access blocked.'),
}


def resource(relative):
    """Explicit transitional lookup; never fabricate or change a robot state file."""
    local=PIPER_ROOT/relative
    if local.exists():return local
    if relative.startswith('tools/video_tools/'):
        return WORKSPACE_ROOT/'tmp/video_tools'/Path(relative).name
    return WORKSPACE_ROOT/relative


def doctor():
    required=['piper_readonly/.venv_mapping/Scripts/python.exe',
        'piper_readonly/.venv/Scripts/python.exe','piper_readonly/web/index.html',
        'piper_readonly/web/viewer.bundle.js','piper_readonly/assets/wrist_cad/fitted_mount.json',
        'piper_readonly/pyAgxArm/pyAgxArm/__init__.py','piper_readonly/commissioning_interlock.json',
        'cad/source/arm_with_wirst_mount_d405.stp','cad/inventor/arm_with_wirst_mount_d405.iam',
        'cad/reference/official_d405.urdf.xacro','cpp_surface/build_robot_replay_20261007/Release/camera_local_api.dll',
        'cpp_surface/build_region_read_20261009_v2/Release/regional_read_api.dll',
        'fgng_realsense/build/Release/geometry_native.dll','tools/video_tools/ffmpeg.exe',
        'output/table_scan_20261007_v1/plan.json']
    files={name:resource(name).is_file() for name in required}
    locations={name:str(resource(name)) for name in required}
    modules={name:importlib.util.find_spec(name) is not None for name in ('numpy','scipy','cv2','pyAgxArm','pyrealsense2','can','usb','gs_usb')}
    report=dict(piper_root=str(PIPER_ROOT),python=sys.executable,files=files,locations=locations,modules=modules,
        consolidation='Application/GUI in om6_piper; SDK, environments, state/interlock and research archives remain shared at legacy paths.',
        hardware_access=False,meaning='Path/dependency check only; no claim about current robot state or readiness.')
    print(json.dumps(report,indent=2))
    return 0 if all(files.values()) and all(modules.values()) else 2


def main(argv=None):
    args=list(sys.argv[1:] if argv is None else argv)
    if not args or args[0]=='doctor':
        if len(args)>1:raise SystemExit('doctor takes no arguments')
        return doctor()
    if args[0] in ('-h','--help','help'):
        print('Usage: piper.cmd <command> [existing program arguments]\nDefault: doctor (offline).\n')
        for name,(_,description) in PROGRAMS.items():print(f'{name:16s} {description}')
        print('\nUse piper.cmd <command> --help. Relative paths are based on om6_piper.\n'
              'No control checks are bypassed. Do not run multiple programs against the same CAN/camera.')
        return 0
    if args[0] not in PROGRAMS:raise SystemExit('Unknown command; use piper.cmd --help')
    script=resource(PROGRAMS[args[0]][0])
    if not script.is_file():raise SystemExit(f'Missing program: {script}')
    forwarded=args[1:]
    if forwarded[:1]==['--']:forwarded=forwarded[1:]
    # Inherit operator console, do not background hardware control or auto-confirm.
    return subprocess.call([sys.executable,str(script),*forwarded],cwd=PIPER_ROOT)


if __name__=='__main__':raise SystemExit(main())
