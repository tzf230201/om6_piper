import can
import time
import struct


CAN_ID_GRIPPER = 0x159


def send_gripper(bus, position_mm, effort=1000, status=0x01, set_zero=0x00):
    """
    position_mm : target opening in mm
    effort      : 1000 = approx 1 N
    status:
        0x00 = disable
        0x01 = enable
        0x02 = disable + clear error
        0x03 = enable + clear error
    """

    # unit = 0.001 mm
    position_raw = int(position_mm * 1000)

    # CAN payload:
    # bytes 0-3 : gripper position, signed int32, big endian
    # bytes 4-5 : effort, uint16, big endian
    # byte 6    : enable/status
    # byte 7    : set zero
    data = struct.pack(
        ">iHBB",
        position_raw,
        effort,
        status,
        set_zero
    )

    msg = can.Message(
        arbitration_id=CAN_ID_GRIPPER,
        data=data,
        is_extended_id=False
    )

    bus.send(msg)

    print(
        f"TX 0x159 | position={position_mm:.1f} mm "
        f"| raw={position_raw} "
        f"| status=0x{status:02X} "
        f"| data={data.hex(' ')}"
    )


print("Opening CAN...")

bus = can.interface.Bus(
    interface="agx_cando",
    channel="0",
    bitrate=1_000_000
)

try:
    print("CAN opened.")

    # -----------------------------
    # clear error
    # -----------------------------
    print("\nClearing gripper error...")
    for _ in range(20):
        send_gripper(
            bus,
            position_mm=0,
            effort=1000,
            status=0x02
        )
        time.sleep(0.01)

    # -----------------------------
    # enable
    # -----------------------------
    print("\nEnabling gripper...")
    for _ in range(20):
        send_gripper(
            bus,
            position_mm=0,
            effort=1000,
            status=0x01
        )
        time.sleep(0.01)

    input("\nPress ENTER -> OPEN to 30 mm")

    # PiPER demo continuously sends commands
    for _ in range(300):
        send_gripper(
            bus,
            position_mm=30.0,
            effort=1000,
            status=0x01
        )
        time.sleep(0.005)

    input("\nPress ENTER -> CLOSE")

    for _ in range(300):
        send_gripper(
            bus,
            position_mm=0.0,
            effort=1000,
            status=0x01
        )
        time.sleep(0.005)

finally:
    bus.shutdown()
    print("\nCAN closed.")