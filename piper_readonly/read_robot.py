"""Read-only Piper status through official pyAgxArm and local gs_usb adapter."""
import argparse
import json
import time
from datetime import datetime
from enum import Enum
from pathlib import Path
import numpy as np
from pyAgxArm import AgxArmFactory, create_agx_arm_config
from readonly_bus import register_backend
from feedback_timing import ReceiverTiming
from feedback_health import decode_frozen_control

from app_paths import LEGACY_ROOT

ROOT = LEGACY_ROOT  # Preserve the authoritative status/output location.

def plain(value):
    if isinstance(value, Enum):
        return value.value
    if value is None or isinstance(value,(str,int,float,bool)):
        return value
    if isinstance(value,(list,tuple)):
        return [plain(v) for v in value]
    if isinstance(value,dict):
        return {str(k):plain(v) for k,v in value.items()}
    if isinstance(value,(bytes,bytearray)):
        return value.hex()
    return {k.lstrip('_'):plain(v) for k,v in vars(value).items()}

class Reader:
    def __init__(self, channel=0, firmware='default', low_latency=False, buffered_rx=False, passive=False):
        register_backend(buffered_rx=buffered_rx, passive=passive)
        self.passive = passive
        self.channel=channel
        self.firmware_driver=firmware
        self.robot=AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper',firmeware_version=firmware,interface='gs_usb',
            channel=channel,bitrate=1000000,enable_check_can=False,
            auto_connect=False,timeout=.02))
        self._receive_timing=ReceiverTiming(enabled=low_latency)
        try:
            self._receive_timing.start()
            self.robot.connect()
            self.bus=self.robot.get_context().get_comm().recv_bus
            self.gripper=self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
            self.firmware=None
        except BaseException:
            try:self.robot.disconnect()
            finally:self._receive_timing.close()
            raise

    def close(self):
        try:self.robot.disconnect()
        finally:
            timing=getattr(self,'_receive_timing',None)
            if timing is not None:timing.close()

    def query_firmware(self):
        if getattr(self, 'passive', False):
            raise PermissionError('Passive reader must not query firmware')
        self.firmware=self.robot.get_firmware(timeout=2,min_interval=3)
        if self.firmware is None:
            # A lost fragment invalidates the whole 11-frame identity reply.
            # Retry this READ query once, never substitute a cached identity.
            time.sleep(3.05)
            self.firmware=self.robot.get_firmware(timeout=2,min_interval=3)
        return self.firmware

    def fresh(self, identifiers, max_age=.5):
        received=self.bus.capture_receive()[0] if hasattr(self.bus,'capture_receive') else self.bus.last_received.copy()
        now=time.perf_counter()
        return all(i in received and 0<=now-received[i]<max_age for i in identifiers)

    def get(self, value, identifiers):
        return plain(value.msg) if value is not None and self.fresh(identifiers) else None

    def snapshot(self):
        r=self.robot
        # Freeze receive times BEFORE reading the clock and SDK values. A newer
        # RX event must not retroactively freshen an older snapshot component.
        if hasattr(self.bus,'capture_receive'):
            received,raw_latest,counts=self.bus.capture_receive()
        else:
            received=self.bus.last_received.copy()
            raw_latest=self.bus.raw_latest.copy()
            counts=self.bus.received.copy()
        def copy_msg(value):return plain(value.msg) if value is not None else None
        values=dict(joints=copy_msg(r.get_joint_angles()),pose=None,arm=None,gripper=None)
        values['pose']=copy_msg(r.get_flange_pose())
        values['arm']=copy_msg(r.get_arm_status())
        values['gripper']=copy_msg(self.gripper.get_gripper_status()) if self.gripper else None
        states=[copy_msg(r.get_driver_states(i)) for i in range(1,7)]
        motors=[copy_msg(r.get_motor_states(i)) for i in range(1,7)]
        stamp=time.perf_counter()
        ages={i:stamp-t for i,t in received.items()}
        # Use unrounded ages for validation; never clamp stale/negative ages.
        def checked(value, identifiers):
            return value if all(i in ages and 0<=ages[i]<.5 for i in identifiers) else None
        joints=checked(values['joints'],[0x2A5,0x2A6,0x2A7])
        pose=checked(values['pose'],[0x2A2,0x2A3,0x2A4])
        states=[checked(v,[0x261+i]) for i,v in enumerate(states)]
        motors=[checked(v,[0x251+i]) for i,v in enumerate(motors)]
        gripper=checked(values['gripper'],[0x2A8])
        return {'captured_at':datetime.now().astimezone().isoformat(),
            'sdk':'pyAgxArm','firmware':self.firmware,'firmware_driver':self.firmware_driver,
            'passive_application_can_tx':getattr(self,'passive',False),
            'interface':'candleLight / gs_usb','bitrate':1000000,
            'joint_angles_rad':joints,'joint_angles_deg':np.rad2deg(joints).tolist() if joints else None,
            'flange_pose_m_rad':pose,'arm_status':checked(values['arm'],[0x2A1]),
            'drivers':states,'motors':motors,'gripper':gripper,
            'all_joint_feedback_fresh':joints is not None,
            'rx_frame_count':sum(counts.values()),
            'rx_ids':{f'0x{k:03X}':v for k,v in sorted(counts.items())},
            'rx_age_s':{f'0x{k:03X}':v for k,v in ages.items()},
            'rx_age_clock':'time.perf_counter; frozen before SDK reads; no clamping',
            'rx_clock_resolution_s':time.get_clock_info('perf_counter').resolution,
            'snapshot_perf_s':stamp,
            'frozen_control':decode_frozen_control(
                {f'0x{k:03X}':v for k,v in raw_latest.items()},
                {f'0x{k:03X}':v for k,v in ages.items()}),
            'transport_health':self.bus.usb_receiver.diagnostics() if hasattr(self.bus,'usb_receiver') else None,
            'receive_timing':self._receive_timing.profile() if hasattr(self,'_receive_timing') else None,
            'firmware_queries_sent':self.bus.queries_sent,
            'blocked_commands':self.bus.blocked_commands,
            'raw_latest':{f'0x{k:03X}':v for k,v in raw_latest.items()}}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=5)
    p.add_argument('--firmware',choices=['default','v183','v188','v189'],default='default')
    p.add_argument('--output',default=str(ROOT/'output'/'status.json'))
    args=p.parse_args()
    reader=Reader(firmware=args.firmware)
    try:
        time.sleep(.8)
        reader.query_firmware()
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:
            time.sleep(.1)
        data=reader.snapshot()
        target=Path(args.output)
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(data,indent=2),encoding='utf-8')
        print(json.dumps(data,indent=2),flush=True)
    finally:
        reader.close()

if __name__=='__main__':
    main()
