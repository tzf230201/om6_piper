import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.DEFAULT,
    interface="agx_cando",
    channel="0",
)

robot = AgxArmFactory.create_arm(cfg)

gripper = robot.init_effector(
    robot.OPTIONS.EFFECTOR.AGX_GRIPPER
)

try:
    print("Connecting...")
    robot.connect()
    time.sleep(1)

    # Enable arm
    print("Enabling arm...")
    for _ in range(100):
        robot.enable()

        if all(robot.get_joints_enable_status_list()):
            break

        time.sleep(0.05)

    print(robot.get_joints_enable_status_list())

    input("\nENTER -> OPEN 30 mm")

    # IMPORTANT:
    # send repeatedly, like the raw CAN version
    for _ in range(300):
        gripper.move_gripper_m(
            value=0.03,
            force=1.0
        )
        time.sleep(0.005)

    print("\nAfter OPEN:")
    print(gripper.get_gripper_status())

    input("\nENTER -> CLOSE")

    for _ in range(300):
        gripper.move_gripper_m(
            value=0.0,
            force=1.0
        )
        time.sleep(0.005)

    print("\nAfter CLOSE:")
    print(gripper.get_gripper_status())

finally:
    robot.disconnect()