import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)


# ============================================================
# PiPER configuration for Windows
# ============================================================

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.DEFAULT,
    interface="agx_cando",
    channel="0",
)

robot = AgxArmFactory.create_arm(cfg)


try:
    print("Connecting to PiPER...")
    robot.connect()

    print("Connected.")
    print("Reading joint positions...\n")

    while True:

        joint_data = robot.get_joint_angles()

        if joint_data is not None:

            print("Raw joint message:")
            print(joint_data.msg)

            print(f"Hz        : {joint_data.hz}")
            print(f"Timestamp : {joint_data.timestamp}")

            print("-" * 50)

        time.sleep(0.05)   # 20 Hz

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    robot.disconnect()
    print("Disconnected.")