"""One supported-arm disable maintenance test; no enable/reset/targets/retry.

Default is an offline plan, with no USB opened. Hardware execution is separate
from the motion interlock and never releases it. Disable can release torque.
"""
import argparse
import json
from pathlib import Path
import threading
import time
import can
from pyAgxArm import AgxArmFactory, create_agx_arm_config
from read_robot import Reader
from readonly_bus import ReadOnlyGsUsbBus, is_firmware_query
from feedback_timing import ReceiverTiming
from commission_piper import bounded_usb_write
from reset_piper_disabled_once import ADAPTER_SERIAL
from disable_once_policy import DisableOncePermit, DISABLE_ID, DISABLE_DATA, preflight_reasons
from disable_verification import verify_disabled


class DisableOnlyBus(ReadOnlyGsUsbBus):
    def __init__(self,*args,**kwargs):
        self.permit=DisableOncePermit()
        self.tx_closed=False
        self.tx_events=[]
        self.tx_lock=threading.Lock()
        super().__init__(*args,**kwargs)

    def close_transmit(self):
        with self.tx_lock:self.tx_closed=True

    def send(self,msg,timeout=None):
        with self.tx_lock:
            try:
                if self.tx_closed:raise PermissionError('Receive-only latch closed')
                if is_firmware_query(msg):
                    if self.queries_sent or self.permit.snapshot is not None:
                        raise PermissionError('One preflight identity query only')
                    self.queries_sent+=1
                else:
                    try:self.permit.consume(msg)
                    finally:self.tx_closed=True
            except BaseException:
                self.blocked_commands+=1
                raise
            event=dict(id=f'0x{msg.arbitration_id:03X}',data=bytes(msg.data).hex(),
                started_perf_s=time.perf_counter(),usb_write_completed=False)
            self.tx_events.append(event)
            try:
                event.update(bounded_usb_write(self.gs_usb,msg.arbitration_id,bytes(msg.data),echo_id=len(self.tx_events)))
                event.update(usb_write_completed=True,completed_perf_s=time.perf_counter())
            except BaseException as exc:
                self.tx_closed=True
                event['error']=str(exc)
                raise


class DisableReader(Reader):
    def __init__(self):
        can.interfaces.BACKENDS['gs_usb']=('disable_piper_once','DisableOnlyBus')
        self.channel,self.firmware_driver,self.firmware=0,'default',None
        self._receive_timing=ReceiverTiming(enabled=True)
        self.robot=AgxArmFactory.create_arm(create_agx_arm_config(robot='piper',
            firmeware_version='default',interface='gs_usb',channel=0,bitrate=1000000,
            enable_check_can=False,auto_connect=False,timeout=.02))
        try:
            self._receive_timing.start();self.robot.connect()
            comm=self.robot.get_context().get_comm();self.bus=comm.recv_bus
            if comm.send_bus is not self.bus or self.bus.gs_usb.serial_number!=ADAPTER_SERIAL:
                raise RuntimeError('Adapter identity or shared bus mismatch')
            self.gripper=self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        except BaseException:
            self.close();raise


def write(path,value):
    with path.open('x',encoding='utf-8') as target:json.dump(value,target,indent=2)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute-once',action='store_true')
    p.add_argument('--operator-supported',action='store_true')
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if not args.execute_once:
        print('OFFLINE ONLY: no USB. Planned disable(255): 0x471 / '+DISABLE_DATA.hex())
        return 0
    if not args.operator_supported or args.output is None:
        p.error('Hardware test requires supported-arm confirmation and unique output directory')
    # Exclusive directory records an attempt before USB; never reuse it.
    args.output.mkdir(parents=True,exist_ok=False)
    write(args.output/'intent.json',dict(command='disable(255)',supported=True,
        max_attempts=1,enable_allowed=False,reset_allowed=False,targets_allowed=False,motion_release=False))
    reader=None;result=dict(motion_release=False,physical_stop_verified=False,disable_confirmed=False)
    try:
        reader=DisableReader();time.sleep(.8);reader.query_firmware()
        initial=reader.snapshot();reasons=preflight_reasons(initial)
        if reasons:raise RuntimeError('; '.join(reasons))
        reference=[q/1000 for q in initial['frozen_control']['joint_millidegrees']]
        before=[]
        for _ in range(20):
            time.sleep(.05);sample=reader.snapshot();before.append(sample)
            reasons=preflight_reasons(sample,reference)
            if reasons:raise RuntimeError('; '.join(reasons))
        write(args.output/'before.json',before)
        reader.bus.permit.arm(reader.snapshot,reference,supported=True,stationary_window_verified=True)
        try:
            reader.robot.disable(255)  # SDK return value intentionally NOT used as proof.
        except Exception as exc:result['write_error']=str(exc)
        finally:reader.bus.close_transmit()
        observations=[];end=time.perf_counter()+5
        while time.perf_counter()<end:
            time.sleep(.05);observations.append(reader.snapshot())
        write(args.output/'after.json',observations)
        event=next((e for e in reader.bus.tx_events if e['id']=='0x471'),{})
        completed=event.get('completed_perf_s')
        tail=observations[-10:]
        checks=[verify_disabled(s,completed) for s in tail]
        result['disable_confirmed']=bool(len(tail)==10 and
            tail[-1]['snapshot_perf_s']-tail[0]['snapshot_perf_s']>=.4 and
            all(c['disabled_confirmed'] for c in checks) and not result.get('write_error'))
        result['verification']=checks
        result['final_control']=observations[-1]['frozen_control']
        qlast=observations[-1]['frozen_control'].get('joint_millidegrees')
        result['final_minus_initial_deg']=[q/1000-r for q,r in zip(qlast,reference)] if qlast else None
        result['outcome']='DISABLED_FEEDBACK_CONFIRMED_NO_MOTION_RELEASE' if result['disable_confirmed'] else 'UNCONFIRMED_NO_RETRY'
    except BaseException as exc:
        result.update(outcome='ABORTED_NO_FURTHER_WRITES',error=f'{type(exc).__name__}: {exc}')
    finally:
        if reader:
            reader.bus.close_transmit();result['tx_events']=reader.bus.tx_events
            try:reader.close()
            except Exception as exc:result['close_error']=str(exc)
        write(args.output/'result.json',result)
        print(json.dumps(result,indent=2))
    return 0 if result['disable_confirmed'] else 2


if __name__=='__main__':raise SystemExit(main())
