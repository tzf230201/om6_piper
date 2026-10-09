# Legacy manual hardware experiments

These scripts were moved here unchanged, away from offline test discovery:

- `gripper_test.py`, `gripper_piper_sdk.py`: historical gripper experiments;
- `joint_raw_test.py`, `read_joint.py`: historical joint/driver experiments;
- `test_can.py`: opens a real CAN adapter; **not an offline unit test**.

Several execute hardware operations at module level. Do not import them to inspect
the installation and do not launch them as part of a build/test job. They are kept
for reference, not advertised as a reviewed universal wake-up procedure.

Start with [robot operations](../../docs/ROBOT_OPERATIONS.md) and the main launcher.
The reorganization did not execute or alter these scripts.
