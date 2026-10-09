"""Separate telemetry quality from controller faults. No motion authorization.

Deadlines remain the existing local 150 ms policy, independently named by
feedback class. They are not manufacturer specifications and are not relaxed
based on observed delays. Raw decoding uses the frozen RX payloads, not SDK
objects that the receive thread might be changing during snapshot creation.
"""
import math
import struct

GROUPS = {
    'controller': (0x2A1,),
    'joint_position': (0x2A5, 0x2A6, 0x2A7),
    'driver_status': tuple(range(0x261, 0x267)),
    'gripper': (0x2A8,),
}
LOCAL_DEADLINES_S = {name: .15 for name in GROUPS}
TRANSPORT_FAULTS = ('usb_errors', 'malformed_frames', 'adapter_overflow_reports',
                    'can_error_frames', 'hardware_clock_discontinuities', 'host_queue_overflows', 'rx_pump_errors')


def describe_feedback(snapshot):
    ages = snapshot.get('rx_age_s', {})
    groups = {}
    for name, ids in GROUPS.items():
        late, invalid = [], []
        values = []
        for ident in ids:
            key = f'0x{ident:03X}'
            age = ages.get(key)
            if isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
                invalid.append(key)
            else:
                values.append(age)
                if age > LOCAL_DEADLINES_S[name]:
                    late.append(key)
        groups[name] = dict(local_deadline_s=LOCAL_DEADLINES_S[name], late_ids=late,
                            missing_or_invalid_ids=invalid,
                            max_age_s=max(values) if values else None,
                            fresh=not late and not invalid)
    health = snapshot.get('transport_health') or {}
    counts = health.get('counts', {})
    faults = {k: counts[k] for k in TRANSPORT_FAULTS if counts.get(k, 0)}
    return dict(groups=groups, transport_faults_since_connect=faults,
                deadlines_are_manufacturer_specs=False, execution_allowed=False)


def decode_frozen_control(raw, ages):
    """Decode exactly the payloads associated with recorded RX ages. No clamps."""
    def frame(ident):
        key = f'0x{ident:03X}'
        data = bytes.fromhex(raw[key])
        age = ages[key]
        if len(data) != 8 or not math.isfinite(age) or age < 0:
            raise ValueError(f'Invalid feedback {key}')
        return data
    try:
        q = [x for ident in GROUPS['joint_position'] for x in struct.unpack('>ii', frame(ident))]
        arm = frame(0x2A1)
        codes = [frame(i)[5] for i in GROUPS['driver_status']]
        gripper = frame(0x2A8)
        return dict(valid_payloads=True, joint_millidegrees=q, driver_codes=codes,
                    gripper_status_code=gripper[6],
                    ctrl_mode=arm[0], arm_status=arm[1], motion_mode=arm[2],
                    teach_status=arm[3], motion_status=arm[4], trajectory_num=arm[5],
                    err_code=int.from_bytes(arm[6:8], 'big'),
                    note='Coherent host cache, not simultaneous joint sampling; check age of each CAN ID separately')
    except (KeyError, TypeError, ValueError, OverflowError):
        return dict(valid_payloads=False)
