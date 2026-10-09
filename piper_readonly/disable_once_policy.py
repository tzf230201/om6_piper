"""Narrow maintenance policy for one disable; never authorizes motion.

No hardware transport here. A separate executor must consume this permit
before any USB write, record completion, and remain receive-only afterward.
"""
import math
import time
from feedback_health import describe_feedback
from reset_piper_disabled_once import IDENTITY

DISABLE_ID = 0x471
DISABLE_DATA = bytes.fromhex('0701000000000000')


def preflight_reasons(snapshot, reference_deg=None):
    reasons=[]
    health=describe_feedback(snapshot)
    if any(not group['fresh'] for group in health['groups'].values()):
        reasons.append('Required feedback missing, invalid or stale')
    if health['transport_faults_since_connect']:
        reasons.append('Transport fault recorded')
    if any((snapshot.get('firmware') or {}).get(k)!=v for k,v in IDENTITY.items()):
        reasons.append('Same-session robot identity mismatch')
    control=snapshot.get('frozen_control') or {}
    if not control.get('valid_payloads'):
        reasons.append('Invalid frozen payloads')
    # motion_status=1 is allowed: this is target-arrival status, not a speed
    # measurement. Stationarity must be observed over a separate time window.
    for key,expected in (('ctrl_mode',1),('arm_status',0),('teach_status',0),('err_code',0)):
        if type(control.get(key)) is not int or control[key]!=expected:
            reasons.append(f'Unexpected controller field: {key}')
    codes=control.get('driver_codes')
    if not isinstance(codes,list) or len(codes)!=6 or any(type(x) is not int or x not in (0,64) for x in codes):
        reasons.append('Driver fault or invalid status')
    grip=control.get('gripper_status_code')
    if type(grip) is not int or grip not in (0,64):
        reasons.append('Gripper fault or invalid status')
    q=control.get('joint_millidegrees')
    valid=isinstance(q,list) and len(q)==6 and all(type(x) is int for x in q)
    if not valid:
        reasons.append('Invalid joint positions')
    elif reference_deg is not None:
        if len(reference_deg)!=6 or any(not math.isfinite(x) for x in reference_deg):
            reasons.append('Invalid reference')
        elif max(abs(x/1000-y) for x,y in zip(q,reference_deg))>.1:
            reasons.append('Joint change exceeds local 0.1 degree stationarity bound')
    return reasons


class DisableOncePermit:
    def __init__(self):
        self.consumed=False
        self.snapshot=None
        self.reference=None
        self.expires=0

    def arm(self, snapshot, reference_deg, *, supported, stationary_window_verified):
        if supported is not True or stationary_window_verified is not True:
            raise PermissionError('Mechanical support and observed stationary window required')
        if self.consumed or self.snapshot is not None:
            raise PermissionError('One-use permit cannot be re-armed')
        reasons=preflight_reasons(snapshot(),reference_deg)
        if reasons:raise PermissionError('; '.join(reasons))
        self.snapshot=snapshot
        self.reference=tuple(reference_deg)
        self.expires=time.monotonic()+2

    def consume(self, msg):
        if self.consumed or self.snapshot is None:
            raise PermissionError('No unused disable permit')
        self.consumed=True  # Every attempt consumes; ambiguous delivery cannot retry.
        if (msg.arbitration_id!=DISABLE_ID or msg.dlc!=8 or bytes(msg.data)!=DISABLE_DATA
            or any((msg.is_extended_id,msg.is_remote_frame,msg.is_error_frame,msg.is_fd))):
            raise PermissionError('Only the exact disable-all frame is permitted')
        if time.monotonic()>self.expires:
            raise PermissionError('Disable permit expired')
        reasons=preflight_reasons(self.snapshot(),self.reference)
        if reasons:raise PermissionError('; '.join(reasons))
