import time
import math

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)

# ============================================================
# CONFIG
# ============================================================

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.DEFAULT,
    interface="agx_cando",
    channel="0",
)

robot = AgxArmFactory.create_arm(cfg)


def deg(values):
    return [math.radians(v) for v in values]


def print_joints(label):
    q = list(robot.get_joint_angles().msg)

    q_deg = [
        math.degrees(x)
        for x in q
    ]

    print(
        label,
        [round(x, 3) for x in q_deg]
    )


try:

    # ========================================================
    # CONNECT
    # ========================================================

    print("Connecting...")
    robot.connect()

    time.sleep(1)

    print_joints("Current:")

    # ========================================================
    # ENABLE
    # ========================================================

    print("\nEnabling arm...")

    for i in range(100):

        robot.enable()

        status = robot.get_joints_enable_status_list()

        print(i, status)

        if all(status):
            break

        time.sleep(0.05)

    if not all(robot.get_joints_enable_status_list()):
        raise RuntimeError(
            "Failed to enable all joints."
        )

    print("ALL JOINTS ENABLED")

    # ========================================================
    # MODE
    # ========================================================

    robot.set_auto_set_motion_mode_enabled(False)

    robot.set_speed_percent(10)

    robot.set_motion_mode(
        robot.OPTIONS.MOTION_MODE.J
    )

    time.sleep(0.5)

    print("\nArm status:")
    print(robot.get_arm_status())

    # ========================================================
    # IMPORTANT:
    #
    # Every target is inside PiPER's documented joint limits.
    #
    # J2 must be >= 0
    # J3 must be <= 0
    #
    # ========================================================

    target_deg = [
        10.0,      # J1
        1.0,       # J2  MUST >= 0
        -1.0,      # J3  MUST <= 0
        -10.0,     # J4
        10.0,      # J5
        2.0,       # J6
    ]

    target = deg(target_deg)

    print("\nTarget:")
    print(target_deg)

    input(
        "\nTEST MOTION\n"
        "J1 -> 10 deg\n"
        "J2 -> +1 deg\n"
        "J3 -> -1 deg\n"
        "Other joints move only slightly.\n\n"
        "Clear workspace and keep E-stop accessible.\n"
        "Press ENTER to start..."
    )

    # ========================================================
    # STREAM TARGET
    # ========================================================

    for i in range(400):

        robot.move_j(target)

        if i % 50 == 0:
            print_joints("Feedback:")

        time.sleep(0.01)

    # ========================================================
    # FINAL
    # ========================================================

    print()
    print_joints("FINAL:")


except KeyboardInterrupt:
    print("\nStopped.")


finally:

    try:
        robot.disconnect()
    except Exception:
        pass

    print("Disconnected.")