import can

print("Testing AgileX CAN adapter...")

try:
    bus = can.interface.Bus(
        interface="agx_cando",
        channel="0",
        bitrate=1000000
    )

    print("SUCCESS: CAN adapter opened!")
    print(bus)

    bus.shutdown()

except Exception as e:
    print("FAILED:")
    print(type(e).__name__)
    print(e)