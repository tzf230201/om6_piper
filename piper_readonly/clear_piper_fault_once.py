"""User-requested, one-shot joint-fault clear; NEVER enable, reset, or move.

Separate maintenance process. The existing readonly_bus.py is not modified.
Without --execute-once this only gathers a preflight. A live send is permitted
only for the exact official clear_joint_error(255) packet, once, with fresh
disabled/idle feedback. An ACK alone is not treated as a cleared fault.
"""
import argparse
import json
import math
from pathlib import Path
import time
import can
import numpy as np
from can.interfaces.gs_usb import GsUsbBus
from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile
from read_robot import Reader
from readonly_bus import ReadOnlyGsUsbBus, is_firmware_query
from feedback_health import TRANSPORT_FAULTS

# SDK default: 07 all joints, 00 no zeroing, 00 acceleration config OFF,
# 01f4 inactive acceleration field, AE clear fault, 0000 reserved.
CLEAR_BYTES=bytes.fromhex('07000001f4ae0000')
REQUIRED_IDS=[0x2A1,0x2A5,0x2A6,0x2A7,0x2A8,*range(0x261,0x267)]


def disabled_maintenance_reasons(s, reference=None, *, expected_ctrl_mode):
    """Shared physical checks; callers must explicitly choose Standby or Teaching.

    This does not grant transmission permission. ClearOncePolicy still requires
    Standby, while the separate recovery tool can check a disabled Teaching arm.
    """
    if expected_ctrl_mode not in (0,2):
        raise ValueError('Maintenance only supports Standby or Teaching state')
    reasons=[]
    transport=(s.get('transport_health') or {}).get('counts',{})
    for key in TRANSPORT_FAULTS:
        if transport.get(key,0):reasons.append(f'Transport fault since connection: {key}')
    fw=s.get('firmware') or {}
    if fw.get('software_version')!='S-V1.5-7' or fw.get('hardware_version')!='H-V1.2-1':
        reasons.append('Device firmware/hardware differs from the identified robot or is unknown')
    if s.get('firmware_driver')!='default':reasons.append('Unexpected SDK profile')
    ages=s.get('rx_age_s',{})
    for ident in REQUIRED_IDS:
        age=ages.get(f'0x{ident:03X}',float('inf'))
        if not isinstance(age,(int,float)) or not math.isfinite(age) or not 0<=age<=.15:
            reasons.append(f'Missing/stale feedback 0x{ident:03X}')
    q=np.asarray(s.get('joint_angles_rad',[]),dtype=float)
    if not s.get('all_joint_feedback_fresh') or q.shape!=(6,) or not np.isfinite(q).all():
        reasons.append('Invalid joint feedback')
    elif reference is not None:
        reference=np.asarray(reference,dtype=float)
        if reference.shape!=(6,) or not np.isfinite(reference).all():
            reasons.append('Invalid stationary reference')
        elif np.rad2deg(np.abs(q-reference)).max()>.2:
            reasons.append('Joint motion exceeds 0.2 degree observation allowance')
    arm=s.get('arm_status') or {}
    if arm.get('ctrl_mode')!=expected_ctrl_mode:
        reasons.append(f'Unexpected ctrl_mode (required {expected_ctrl_mode})')
    for key in ['arm_status','teach_status','motion_status','trajectory_num','err_code']:
        if arm.get(key)!=0:reasons.append(f'Controller not idle/standby: {key}')
    drivers=s.get('drivers') or []
    if len(drivers)!=6:reasons.append('Missing driver state')
    for j,d in enumerate(drivers,1):
        code=(d or {}).get('foc_status_code')
        if code not in [0,0x10,0x20,0x30]:
            reasons.append(f'J{j}: enabled or additional voltage/thermal/current/stall flag')
        if (d or {}).get('foc_status',{}).get('driver_enable_status') is not False:
            reasons.append(f'J{j}: disabled state not confirmed')
    gripper=s.get('gripper') or {}
    if gripper.get('status_code')!=0 or gripper.get('foc_status',{}).get('driver_enable_status') is not False:
        reasons.append('Gripper disabled/normal state not confirmed')
    return reasons


def disabled_idle_reasons(s, reference=None):
    # Preserve the original clear-only guard: Teaching never grants a clear.
    return disabled_maintenance_reasons(s, reference, expected_ctrl_mode=0)


def is_exact_clear(msg):
    return (msg.arbitration_id==0x475 and msg.dlc==8 and bytes(msg.data)==CLEAR_BYTES
            and not msg.is_extended_id and not msg.is_remote_frame
            and not msg.is_error_frame and not msg.is_fd)


class ClearOncePolicy:
    def __init__(self):
        self.attempted=False
        self.deadline=0
        self.snapshot=None
        self.reference=None

    def arm(self, snapshot, reference):
        if self.attempted:raise PermissionError('No repeat of a clear attempt')
        self.snapshot=snapshot;self.reference=np.asarray(reference).copy()
        reasons=disabled_idle_reasons(snapshot(),self.reference)
        if reasons:raise PermissionError('; '.join(reasons))
        self.deadline=time.monotonic()+2

    def consume(self,msg):
        if not is_exact_clear(msg):raise PermissionError('Only the exact clear-fault packet is allowed')
        if self.attempted or self.snapshot is None or time.monotonic()>self.deadline:
            raise PermissionError('One-shot clear permit absent, expired, or already used')
        # Consume before the send, including an ambiguous USB failure.
        self.attempted=True;self.deadline=0
        reasons=disabled_idle_reasons(self.snapshot(),self.reference)
        if reasons:raise PermissionError('; '.join(reasons))


class FaultClearGsUsbBus(ReadOnlyGsUsbBus):
    def __init__(self,*args,**kwargs):
        self.clear_policy=ClearOncePolicy()
        self.clear_tx_successful=0
        self.clear_frames=[]
        super().__init__(*args,**kwargs)

    def send(self,msg,timeout=None):
        if is_firmware_query(msg):return super().send(msg,timeout)
        try:self.clear_policy.consume(msg)
        except Exception:
            self.blocked_commands+=1
            raise
        self.clear_frames.append(dict(id='0x475',data=bytes(msg.data).hex(),attempted_monotonic=time.monotonic()))
        # Deliberately separate maintenance capability: never widen the monitor.
        result=GsUsbBus.send(self,msg,timeout)
        self.clear_tx_successful+=1
        return result


class FaultClearReader(Reader):
    def __init__(self):
        can.interfaces.BACKENDS['gs_usb']=('clear_piper_fault_once','FaultClearGsUsbBus')
        self.channel=0;self.firmware_driver='default';self.firmware=None
        self.robot=AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper',firmeware_version='default',interface='gs_usb',channel=0,
            bitrate=1000000,enable_check_can=False,auto_connect=False,timeout=.02))
        self.robot.connect()
        self.bus=self.robot.get_context().get_comm().recv_bus
        if self.robot.get_context().get_comm().send_bus is not self.bus:
            self.robot.disconnect();raise RuntimeError('Expected shared guarded RX/TX bus')
        self.gripper=self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)


def write_json(path,data):
    path.write_text(json.dumps(data,indent=2,allow_nan=False),encoding='utf8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-once',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    # A run directory must be new. Never rerun after an ambiguous outcome.
    args.output.mkdir(parents=True,exist_ok=False)
    result=dict(execute_requested=args.execute_once,clear_attempted=False,ack_received=None,
                fault_clear_verified=False,motor_enable_commands_sent=0,motion_commands_sent=0,
                controller_reset_commands_sent=0,protection_changes_sent=0)
    reader=None
    try:
        reader=FaultClearReader()
        # Receive first: the SDK firmware parser consults its receive FPS.
        time.sleep(.8)
        reader.query_firmware()
        before=reader.snapshot();write_json(args.output/'before.json',before)
        reasons=disabled_idle_reasons(before)
        if reasons:raise RuntimeError('Preflight rejected: '+'; '.join(reasons))
        profile=resolve_firmware_profile('piper',before['firmware']['software_version'])
        if profile!='default':raise RuntimeError('SDK profile mismatch')
        reference=np.asarray(before['joint_angles_rad'])
        observations=[]
        for _ in range(20):
            time.sleep(.05);s=reader.snapshot();observations.append(s)
            reasons=disabled_idle_reasons(s,reference)
            if reasons:raise RuntimeError('Pre-clear observation rejected: '+'; '.join(reasons))
        write_json(args.output/'pre_clear_observations.json',observations)
        if not any(d['foc_status_code'] for d in observations[-1]['drivers']):
            result['outcome']='already_clear_no_write'
        elif not args.execute_once:
            result['outcome']='dry_run_no_write'
        else:
            write_json(args.output/'intent.json',dict(command='clear_joint_error(255)',
                expected_can_id='0x475',expected_data_hex=CLEAR_BYTES.hex(),maximum_attempts=1,
                motors_must_remain_disabled=True,automatic_motion_after_clear=False))
            reader.bus.clear_policy.arm(reader.snapshot,reference)
            result['clear_attempted']=True
            result['ack_received']=bool(reader.robot.clear_joint_error(joint_index=255,timeout=1.0))
            after_samples=[]
            for _ in range(60):
                time.sleep(.05);s=reader.snapshot();after_samples.append(s)
                write_json(args.output/'after_latest.json',s)
                reasons=disabled_idle_reasons(s,reference)
                if reasons:raise RuntimeError('Post-clear state unexpected; no further writes: '+'; '.join(reasons))
            write_json(args.output/'after_observations.json',after_samples)
            result['fault_clear_verified']=all(
                all(d['foc_status_code']==0 for d in s['drivers']) for s in after_samples[-20:])
            result['outcome']='cleared_motors_disabled' if result['fault_clear_verified'] else 'flags_persist_no_retry'
            result['after_codes']=[d['foc_status_code'] for d in after_samples[-1]['drivers']]
            result['max_observed_joint_delta_deg']=float(np.rad2deg(np.abs(
                np.array([s['joint_angles_rad'] for s in after_samples])-reference)).max())
    except Exception as exc:
        result['outcome']='aborted_no_further_write'
        result['error']=str(exc)
    finally:
        if reader:
            result['clear_tx_successful']=reader.bus.clear_tx_successful
            result['clear_frames']=reader.bus.clear_frames
            result['firmware_queries_sent']=reader.bus.queries_sent
            result['blocked_commands']=reader.bus.blocked_commands
            reader.close()
        write_json(args.output/'result.json',result)
        print(json.dumps(result,indent=2),flush=True)
    return 0 if result['outcome'] in ['cleared_motors_disabled','already_clear_no_write','dry_run_no_write'] else 2


if __name__=='__main__':raise SystemExit(main())
