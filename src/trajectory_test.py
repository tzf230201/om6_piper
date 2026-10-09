import time
import math
import argparse

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)

# ============================================================
# SETTINGS
# ============================================================

CONTROL_HZ = 100.0
DT = 1.0 / CONTROL_HZ

SPEED_PERCENT = 20
INITIAL_HOLD = 1.0
HOLD_TIME = 1.0


# ============================================================
# TRAJECTORY
# All values are OFFSETS from the sanitized start pose
# ============================================================

SEGMENTS = [
    {
        "name": "Lift to preparation",
        "duration": 11.0,
        "offset_deg": [0, 35, -45, 0, 0, 0],
    },
    {
        "name": "Open to left",
        "duration": 6.0,
        "offset_deg": [-25, 55, -50, 0, -10, -10],
    },
    {
        "name": "Sweep left to right",
        "duration": 12.0,
        "offset_deg": [25, 55, -50, 0, -10, 10],
    },
    {
        "name": "Change height",
        "duration": 6.0,
        "offset_deg": [25, 40, -45, 0, 10, 10],
    },
    {
        "name": "Sweep right to left",
        "duration": 12.0,
        "offset_deg": [-25, 40, -45, 0, 10, -10],
    },
    {
        "name": "Return to preparation",
        "duration": 6.0,
        "offset_deg": [0, 35, -45, 0, 0, 0],
    },
    {
        "name": "Return to initial",
        "duration": 11.0,
        "offset_deg": [0, 0, 0, 0, 0, 0],
    },
]


# ============================================================
# JOINT LIMITS
# Based on PiPER command limits
# ============================================================

JOINT_LIMITS_DEG = [
    (-150.0, 150.0),   # J1
    (0.0, 180.0),      # J2
    (-170.0, 0.0),     # J3
    (-100.0, 100.0),   # J4
    (-70.0, 70.0),     # J5
    (-120.0, 120.0),   # J6
]


# ============================================================
# HELPERS
# ============================================================

def rad2deg_list(q):
    return [math.degrees(x) for x in q]


def deg2rad_list(q):
    return [math.radians(x) for x in q]


def clamp(value, low, high):
    return max(low, min(high, value))


def sanitize_pose_deg(q_deg):
    result = []

    for i, q in enumerate(q_deg):
        low, high = JOINT_LIMITS_DEG[i]
        result.append(clamp(q, low, high))

    return result


def quintic(u):
    u = max(0.0, min(1.0, u))

    return (
        10.0 * u**3
        - 15.0 * u**4
        + 6.0 * u**5
    )


def interpolate_quintic(q_start, q_end, u):
    s = quintic(u)

    return [
        qs + s * (qe - qs)
        for qs, qe in zip(q_start, q_end)
    ]


def get_current_joints(robot):
    feedback = robot.get_joint_angles()

    if feedback is None:
        raise RuntimeError("No joint feedback.")

    msg = feedback.msg

    if not isinstance(msg, (list, tuple)):
        raise RuntimeError(
            f"Unexpected joint feedback: {msg}"
        )

    if len(msg) != 6:
        raise RuntimeError(
            f"Expected 6 joints, got {len(msg)}"
        )

    if not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in msg):
        raise RuntimeError('Nonfinite or invalid joint feedback')
    return [float(x) for x in msg]


def enable_arm(robot, timeout=5.0):
    start = time.perf_counter()

    while time.perf_counter() - start < timeout:

        robot.enable()

        enabled = robot.get_joints_enable_status_list()

        print("Enable:", enabled)

        if isinstance(enabled,(list,tuple)) and len(enabled)==6 and all(enabled):
            return True

        time.sleep(0.05)

    return False


def print_pose(title, q_rad):
    q_deg = rad2deg_list(q_rad)

    print(title)

    for i, q in enumerate(q_deg, start=1):
        print(f"  J{i}: {q:8.3f} deg")


# ============================================================
# HOLD
# ============================================================

def hold_pose(robot, q_target, duration):

    start = time.perf_counter()
    next_tick = start

    while True:

        now = time.perf_counter()

        if now - start >= duration:
            break

        robot.move_j(q_target)

        next_tick += DT

        sleep_time = next_tick - time.perf_counter()

        if sleep_time > 0:
            time.sleep(sleep_time)


# ============================================================
# EXECUTE ONE SEGMENT
# ============================================================

def execute_segment(
    robot,
    q_start,
    q_end,
    duration,
    name
):

    print("\n====================================")
    print(name)
    print("====================================")

    print_pose("Start:", q_start)
    print_pose("Target:", q_end)

    print(f"Duration: {duration:.1f} s")

    start_time = time.perf_counter()
    next_tick = start_time

    while True:

        now = time.perf_counter()
        elapsed = now - start_time

        if elapsed >= duration:
            break

        u = elapsed / duration

        q_cmd = interpolate_quintic(
            q_start,
            q_end,
            u
        )

        robot.move_j(q_cmd)

        next_tick += DT

        sleep_time = next_tick - time.perf_counter()

        if sleep_time > 0:
            time.sleep(sleep_time)

    # exact final target
    for _ in range(20):
        robot.move_j(q_end)
        time.sleep(DT)

    print("Segment complete.")

    feedback = get_current_joints(robot)

    print_pose(
        "Actual feedback:",
        feedback
    )

    return q_end


# ============================================================
# MAIN
# ============================================================

parser = argparse.ArgumentParser(description='Piper scan with optional RGB-D evidence recording')
parser.add_argument('--record-vision', action='store_true')
parser.add_argument('--vision-output', default='output/piper_scan_vision')
parser.add_argument('--profile', choices=['original', 'vision_slow'], default='original')
parser.add_argument('--table-plan', help='Offline table_scan.py plan; actual start must match within 2 degrees')
parser.add_argument('--map-live', action='store_true', help='Map the shared RGB-D stream, and record vision')
parser.add_argument('--map-host', default='127.0.0.1')
parser.add_argument('--map-port', type=int, default=8770)
args = parser.parse_args()
if args.map_live:args.record_vision=True
if args.map_live:
    # Fail before connecting CAN or creating capture files if the viewer cannot bind.
    from vision_capture import check_map_bind
    try:
        check_map_bind(args.map_host,args.map_port)
    except (RuntimeError,ValueError) as exc:
        parser.error(str(exc))
from vision_motion_profile import timing_profile
SEGMENTS, HOLD_TIME, SPEED_PERCENT = timing_profile(SEGMENTS, args.profile)
if not args.table_plan:
    print('Timing profile:', args.profile, '| speed percent:', SPEED_PERCENT,
          '| waypoint hold:', HOLD_TIME, 's')
vision = None

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.DEFAULT,
    interface="agx_cando",
    channel="0",
)

robot = AgxArmFactory.create_arm(cfg)


try:

    # --------------------------------------------------------
    # CONNECT
    # --------------------------------------------------------

    print("Connecting to PiPER...")
    robot.connect()

    time.sleep(1.0)

    # --------------------------------------------------------
    # READ ACTUAL START POSE
    # --------------------------------------------------------

    q_actual_start = get_current_joints(robot)

    q_actual_start_deg = rad2deg_list(
        q_actual_start
    )

    print("\nACTUAL START POSE:")

    for i, q in enumerate(
        q_actual_start_deg,
        start=1
    ):
        print(f"J{i}: {q:.3f} deg")

    # --------------------------------------------------------
    # SANITIZE START COMMAND
    #
    # Example:
    # J2 = -0.248 -> 0.0
    # J3 = +0.985 -> 0.0
    # --------------------------------------------------------

    q_start_cmd_deg = sanitize_pose_deg(
        q_actual_start_deg
    )
    if args.table_plan:
        from table_scan import load_for_execution
        table_plan=load_for_execution(args.table_plan,q_actual_start)
        SEGMENTS=table_plan['segments']
        HOLD_TIME=table_plan['hold_s'];SPEED_PERCENT=table_plan['speed_percent']
        q_start_cmd_deg=list(q_actual_start_deg)  # No silent clamp for table profile.
        print('TABLE SCAN: nominal CAD; assumed table Z=',table_plan['table_z_m'],'m')
        print('Timing profile: table_scan | speed percent:',SPEED_PERCENT,'| waypoint hold:',HOLD_TIME,'s')
        print('Not collision-certified. Check the preview, tabletop, jaws and camera cable.')

    q_start_cmd = deg2rad_list(
        q_start_cmd_deg
    )

    print("\nSANITIZED START COMMAND:")

    for i, q in enumerate(
        q_start_cmd_deg,
        start=1
    ):
        print(f"J{i}: {q:.3f} deg")

    # --------------------------------------------------------
    # GENERATE WAYPOINTS
    # --------------------------------------------------------

    waypoints = []

    print("\nWAYPOINTS:")

    for i, segment in enumerate(
        SEGMENTS,
        start=1
    ):

        target_deg = segment['target_deg'] if args.table_plan else [
            q0 + offset
            for q0, offset in zip(
                q_start_cmd_deg,
                segment["offset_deg"]
            )
        ]

        # Validate/clamp every waypoint
        sanitized_target_deg = sanitize_pose_deg(
            target_deg
        )

        if target_deg != sanitized_target_deg:
            if args.table_plan:raise RuntimeError('Table waypoint outside command limits; refusing to clamp')

            print(
                f"\nWARNING: waypoint {i} "
                f"was outside joint limits."
            )

            print(
                "Requested:",
                [
                    round(x, 3)
                    for x in target_deg
                ]
            )

            print(
                "Clamped:",
                [
                    round(x, 3)
                    for x in sanitized_target_deg
                ]
            )

        target_rad = deg2rad_list(
            sanitized_target_deg
        )

        waypoints.append(target_rad)

        print(
            f"{i}. {segment['name']}:",
            [
                round(x, 3)
                for x in sanitized_target_deg
            ]
        )

    # --------------------------------------------------------
    # CONFIRM BEFORE MOVEMENT
    # --------------------------------------------------------

    print("\n====================================")
    print("READY FOR HARDWARE EXECUTION")
    print("====================================")

    if args.table_plan:
        print('Confirm that the tabletop is at base-frame Z =',table_plan['table_z_m'],'m.')
        print('This is a new downward scan. Preview does not check obstacles, jaws, cables or self-collision.')

    input(
        "\nThe robot will move through the scan trajectory.\n"
        "Clear the workspace.\n"
        "Keep emergency stop accessible.\n"
        "Press ENTER to start..."
    )

    # Recorder shares this robot's SDK connection; never opens another CAN bus.
    # Camera/dependency startup must succeed BEFORE any enable is requested.
    if args.record_vision:
        from vision_capture import VisionCapture
        vision = VisionCapture(robot, args.vision_output,map_live=args.map_live,map_host=args.map_host,map_port=args.map_port,run_metadata=dict(
            profile='table_scan' if args.table_plan else args.profile,control_hz_requested=CONTROL_HZ,speed_percent=SPEED_PERCENT,
            table_z_m=table_plan['table_z_m'] if args.table_plan else None,
            initial_hold_s=INITIAL_HOLD,waypoint_hold_s=HOLD_TIME,segments=SEGMENTS,
            actual_start_deg=q_actual_start_deg,command_start_deg=q_start_cmd_deg,
            waypoints_deg=[rad2deg_list(q) for q in waypoints]))
        vision.start()
        print('Vision recording:', vision.output)

    # User confirmation/camera startup may take time; never execute a stale start.
    q_before_enable=get_current_joints(robot)
    if max(abs(a-b) for a,b in zip(q_before_enable,q_actual_start))>math.radians(2):
        raise RuntimeError('Start pose changed during preparation; no enable sent. Replan and retry.')

    # --------------------------------------------------------
    # ENABLE
    # --------------------------------------------------------

    print("\nEnabling arm...")

    if not enable_arm(robot):
        raise RuntimeError(
            "Failed to enable all joints."
        )

    print("All joints enabled.")

    # --------------------------------------------------------
    # CONTROL MODE
    # --------------------------------------------------------

    robot.set_auto_set_motion_mode_enabled(
        False
    )

    robot.set_speed_percent(
        SPEED_PERCENT
    )

    robot.set_motion_mode(
        robot.OPTIONS.MOTION_MODE.J
    )

    time.sleep(0.5)

    print("\nArm status:")
    print(robot.get_arm_status())

    # --------------------------------------------------------
    # FIRST:
    # move from physical pose to sanitized pose
    #
    # This should only correct tiny zero-offset errors
    # around J2/J3.
    # --------------------------------------------------------

    print("\nMoving to sanitized start pose...")

    execute_segment(
        robot,
        q_actual_start,
        q_start_cmd,
        duration=2.0,
        name="Normalize start pose"
    )

    print("\nInitial hold...")

    hold_pose(
        robot,
        q_start_cmd,
        INITIAL_HOLD
    )

    # --------------------------------------------------------
    # TRAJECTORY
    # --------------------------------------------------------

    q_prev = q_start_cmd

    trajectory_start = time.perf_counter()

    for segment, q_target in zip(
        SEGMENTS,
        waypoints
    ):

        if vision is not None:
            vision.mark('command_segment_begin',name=segment['name'],duration_s=segment['duration'])

        q_prev = execute_segment(
            robot=robot,
            q_start=q_prev,
            q_end=q_target,
            duration=segment["duration"],
            name=segment["name"],
        )

        if vision is not None:
            vision.mark('command_hold_begin',name=segment['name'],duration_s=HOLD_TIME)

        print(
            f"Holding {HOLD_TIME:.1f} s..."
        )

        hold_pose(
            robot,
            q_prev,
            HOLD_TIME
        )

        if vision is not None:
            vision.mark('command_hold_end',name=segment['name'])

    elapsed = (
        time.perf_counter()
        - trajectory_start
    )

    # --------------------------------------------------------
    # FINAL
    # --------------------------------------------------------

    print("\n====================================")
    print("TRAJECTORY COMPLETE")
    print("====================================")

    print(
        f"Trajectory time: {elapsed:.2f} s"
    )

    q_final = get_current_joints(robot)

    print_pose(
        "\nFINAL FEEDBACK:",
        q_final
    )


except KeyboardInterrupt:

    print("\nCommand loop interrupted. This is NOT confirmation that motors are disabled; use the physical stop if needed.")


finally:

    if vision is not None:
        try:
            vision.close()
        except Exception as exc:
            print('Vision cleanup error:', exc)

    try:
        robot.disconnect()
    except Exception:
        pass

    print("Disconnected.")
