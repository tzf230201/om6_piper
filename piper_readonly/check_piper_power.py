"""Passive startup/power observation. NEVER sends even a firmware READ query.

No application CAN TX; USB adapter configuration and CAN acknowledgement are
still active. Silence is not proof of electrical isolation or motor safety.
This diagnostic does not unlock commissioning or certify motion readiness.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from feedback_health import GROUPS, describe_feedback
from read_robot import Reader


DRIVER_FAULT_BITS = (
    (0, 'voltage_too_low'), (1, 'motor_overheating'),
    (2, 'driver_overcurrent'), (3, 'driver_overheating'),
    (4, 'collision_protection'), (5, 'driver_error'), (7, 'stall_protection'),
)


def reported_faults(control, codes):
    """Report fresh status bits separately from communication availability.

    These SDK-defined labels are not a diagnosis of the physical cause, and
    NO_FLAGS_REPORTED does not certify that motion or stopping is safe.
    """
    unknown=dict(state='UNKNOWN', driver_faults=None, driver_enabled=None,
                 controller_arm_status=None, controller_error_code=None,
                 gripper_fault_mask=None, execution_allowed=False)
    if not isinstance(codes,list) or len(codes)!=6 or any(
            type(code) is not int or not 0<=code<=255 for code in codes):
        return unknown
    arm=control.get('arm_status'); err=control.get('err_code')
    gripper=control.get('gripper_status_code')
    if (type(arm) is not int or not 0<=arm<=255 or
        type(err) is not int or not 0<=err<=65535 or
        type(gripper) is not int or not 0<=gripper<=255):
        return unknown
    faults=[dict(joint=j, code_hex=f'0x{code:02X}',
                 flags=[name for bit,name in DRIVER_FAULT_BITS if code & (1<<bit)])
            for j,code in enumerate(codes,1) if code & 0xBF]
    gripper_mask=gripper & 0xBF
    return dict(state='FAULT_OR_PROTECTION_REPORTED' if faults or arm or err or gripper_mask
                else 'NO_FLAGS_REPORTED', driver_faults=faults,
                driver_enabled=[bool(code & 0x40) for code in codes],
                controller_arm_status=arm, controller_error_code=err,
                gripper_fault_mask=gripper_mask, execution_allowed=False,
                note='Status-bit report only; cause and safe motion/stop are not established')


def receive_inventory(samples):
    """Describe all cached standard data IDs, not just motion-required IDs.

    ID patterns are diagnostic clues, never a controller-mode determination.
    Counts describe the recorded window; no assertion about current hardware.
    """
    keys=sorted({key for s in samples for key in s.get('rx_ids',{})})
    for key in keys:
        try:
            ident=int(key,16)
        except (TypeError,ValueError):
            raise ValueError('Invalid standard CAN identifier') from None
        if not 0<=ident<=0x7FF or key!=f'0x{ident:03X}':
            raise ValueError('Noncanonical/nonstandard CAN identifier')
        counts=[s.get('rx_ids',{}).get(key,0) for s in samples]
        if any(type(v) is not int or v<0 for v in counts) or any(a>b for a,b in zip(counts,counts[1:])):
            raise ValueError('Invalid/decreasing receive counters')
    first=samples[0].get('rx_ids',{})
    last=samples[-1].get('rx_ids',{})
    delta={key:last.get(key,0)-first.get(key,0) for key in keys}
    new={key:value for key,value in delta.items() if value>0}
    expected=set(range(0x251,0x257))|set(range(0x261,0x267))|set(range(0x2A1,0x2A9))
    alternative=set(range(0x2B1,0x2B9))|set(range(0x2C1,0x2C9))|set(range(0x3A1,0x3A9))
    expected_new={key:v for key,v in new.items() if int(key,16) in expected}
    alternative_new={key:v for key,v in new.items() if int(key,16) in alternative}
    if not new:
        pattern='NO_NEW_STANDARD_DATA'
    elif expected_new:
        pattern='EXPECTED_TELEMETRY_ACTIVITY'
    elif alternative_new:
        pattern='ALTERNATIVE_ID_ACTIVITY_MODE_UNKNOWN'
    elif set(new)=={'0x4AF'}:
        pattern='FIRMWARE_ONLY_ACTIVITY'
    else:
        pattern='OTHER_STANDARD_DATA_ACTIVITY'
    return dict(pattern=pattern,new_standard_frames=sum(new.values()),new_frames_by_id=new,
        expected_telemetry_new_frames_by_id=expected_new,
        alternative_new_frames_by_id=alternative_new,
        firmware_frames_received=new.get('0x4AF',0),
        firmware_received_while_application_tx_blocked=bool(new.get('0x4AF',0)) and all(
            s.get('passive_application_can_tx') is True and s.get('firmware_queries_sent')==0 for s in samples),
        firmware_response_trigger_verified=False,controller_mode_inferred=False,
        execution_allowed=False,
        note='Standard data only; errors/echoes are in transport_health. Firmware/alternative IDs do not establish motor state. Unchanged cached counts are not new activity.')


def summarize_power_window(samples):
    if len(samples) < 2:
        raise ValueError('Need at least two snapshots over a two-second window')
    stamps = [s.get('snapshot_perf_s') for s in samples]
    if any(type(t) not in (int,float) or not math.isfinite(t) for t in stamps):
        raise ValueError('Invalid snapshot clock')
    if any(a >= b for a,b in zip(stamps,stamps[1:])) or stamps[-1]-stamps[0] < 2:
        raise ValueError('Nonmonotonic/too short observation')
    inventory=receive_inventory(samples)
    ids = [f'0x{i:03X}' for group in GROUPS.values() for i in group]
    delta = {key:samples[-1].get('rx_ids',{}).get(key,0)-samples[0].get('rx_ids',{}).get(key,0) for key in ids}
    health = describe_feedback(samples[-1])
    fresh = all(g['fresh'] for g in health['groups'].values())
    c = samples[-1].get('frozen_control') or {}
    if not any(delta.values()):
        state = 'NO_NEW_ROBOT_FEEDBACK'
    elif all(delta[k] >= 3 for k in ids) and fresh and c.get('valid_payloads') and not health['transport_faults_since_connect']:
        state = 'ROBOT_FEEDBACK_PRESENT'
    else:
        state = 'PARTIAL_OR_UNHEALTHY_FEEDBACK'
    codes = c.get('driver_codes') if fresh and c.get('valid_payloads') else None
    faults = reported_faults(c, codes)
    disabled = (state == 'ROBOT_FEEDBACK_PRESENT' and faults['state']=='NO_FLAGS_REPORTED' and codes == [0]*6
        and all(c.get(k) == 0 for k in ('ctrl_mode','arm_status','teach_status','motion_status','err_code','trajectory_num','gripper_status_code')))
    return dict(state=state, first_time=samples[0].get('captured_at'),last_time=samples[-1].get('captured_at'),
        state_scope='Telemetry availability/freshness only, NOT fault clearance or motion readiness; see reported_faults and receive_inventory',
        receive_inventory=inventory,
        duration_s=stamps[-1]-stamps[0], new_feedback_by_id=delta,
        latest_feedback_health=health, latest_driver_codes=codes, reported_faults=faults,
        disabled_standby_observed=disabled, execution_allowed=False,
        electrical_power_isolation_verified=False, identity_verified=False,
        interpretation='Missing required telemetry does not necessarily mean no CAN traffic. Firmware replies do not establish motor state. No data can mean power off, disconnected wiring or communication failure; not a safety certification')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=4)
    p.add_argument('--live',action='store_true',help='Print receiver readiness and periodic passive feedback; never sends commands')
    p.add_argument('--replay',type=Path,help='Analyze a saved samples.json OFFLINE; does not construct Reader or open USB')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(argv)
    if not 3 <= args.seconds <= 120:
        p.error('Use a 3..120 second bounded observation')
    if args.replay and args.live:
        p.error('--replay is offline and cannot be combined with --live')
    args.output.mkdir(parents=True,exist_ok=False)
    if args.replay:
        result=dict(mode='offline_replay',execution_allowed=False,hardware_opened=False,
            application_can_frames_sent=0,source=str(args.replay.resolve()))
        try:
            source=args.replay.read_bytes()
            result['source_sha256']=hashlib.sha256(source).hexdigest()
            result.update(summarize_power_window(json.loads(source)))
            result['note']='Historical observation only, not the current robot state'
        except Exception as exc:
            result.update(state='OBSERVATION_FAILED',error=f'{type(exc).__name__}: {exc}')
        (args.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps(result,indent=2))
        return 2 if result['state']=='OBSERVATION_FAILED' else 0
    samples=[]
    reader=None
    result=dict(execution_allowed=False,application_can_tx_allowed=False)
    try:
        reader=Reader(low_latency=True,buffered_rx=True,passive=True)
        # Drain possible adapter startup remnants; never call query_firmware.
        time.sleep(1)
        reader.bus.usb_receiver.begin_observation()
        until=time.perf_counter()+args.seconds
        next_update=time.perf_counter()+2.1
        if args.live:
            print(json.dumps(dict(event='PASSIVE_RECEIVER_READY',seconds=args.seconds,
                execution_allowed=False,application_can_tx_allowed=False,
                note='Receiver ready only; NOT permission to move and NOT a verified motor state')),flush=True)
        while time.perf_counter()<until:
            samples.append(reader.snapshot())
            now=time.perf_counter()
            if args.live and now>=next_update:
                window=samples[-60:]
                if len(window)>=2 and window[-1]['snapshot_perf_s']-window[0]['snapshot_perf_s']>=2:
                    observation=summarize_power_window(window)
                    health=observation['latest_feedback_health']
                    fresh=all(group['fresh'] for group in health['groups'].values())
                    control=samples[-1].get('frozen_control') or {}
                    current=control if fresh and control.get('valid_payloads') else {}
                    print(json.dumps(dict(event='PASSIVE_FEEDBACK',
                        captured_at=samples[-1].get('captured_at'),remaining_s=max(0,until-now),
                        state=observation['state'],driver_codes=observation['latest_driver_codes'],
                        reported_faults=observation['reported_faults'],
                        receive_pattern=observation['receive_inventory']['pattern'],
                        new_standard_frames=observation['receive_inventory']['new_standard_frames'],
                        joint_deg=[v/1000 for v in current['joint_millidegrees']] if current else None,
                        ctrl_mode=current.get('ctrl_mode'),arm_status=current.get('arm_status'),
                        err_code=current.get('err_code'),
                        transport_faults=health['transport_faults_since_connect'],
                        firmware_queries_sent=reader.bus.queries_sent,
                        blocked_commands=reader.bus.blocked_commands,execution_allowed=False)),flush=True)
                next_update=now+2
            time.sleep(.05)
        result.update(summarize_power_window(samples))
    except Exception as exc:
        result.update(state='OBSERVATION_FAILED',error=f'{type(exc).__name__}: {exc}')
    finally:
        if reader:
            result['firmware_queries_sent']=reader.bus.queries_sent
            result['blocked_commands']=reader.bus.blocked_commands
            try:
                reader.close()
            except Exception as exc:
                result['close_error']=str(exc)
        (args.output/'samples.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
        (args.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
    return 2 if result['state']=='OBSERVATION_FAILED' else 0


if __name__=='__main__':
    raise SystemExit(main())
