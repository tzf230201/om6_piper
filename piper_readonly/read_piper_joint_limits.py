"""Bounded controller-limit READ queries; no enable, motion, reset or settings.

Normal readonly_bus.py is unchanged. This isolated diagnostic backend adds only
six individually permitted official 0x472 angle/speed READ requests, once each.
"""
import argparse
import json
import math
from pathlib import Path
import time

import can
from can.interfaces.gs_usb import GsUsbBus
from pyAgxArm import AgxArmFactory, create_agx_arm_config
from readonly_bus import ReadOnlyGsUsbBus, is_firmware_query
from read_robot import Reader, plain
from clear_piper_fault_once import write_json


def query_bytes(joint):
    if type(joint) is not int or not 1<=joint<=6:raise ValueError('Joint must be 1..6')
    return bytes([joint,1,0,0,0,0,0,0])


def is_limit_query(msg):
    return (msg.arbitration_id==0x472 and msg.dlc==8 and len(msg.data)==8
            and 1<=msg.data[0]<=6 and bytes(msg.data)==query_bytes(msg.data[0])
            and not msg.is_extended_id and not msg.is_remote_frame
            and not msg.is_error_frame and not msg.is_fd)


class QueryBudget:
    def __init__(self):
        self.used=set()
        self.pending=None

    def allow(self,joint):
        query_bytes(joint)
        if joint in self.used or self.pending is not None:
            raise PermissionError('No duplicate or overlapping limit query')
        self.pending=joint

    def consume(self,msg):
        if not is_limit_query(msg) or self.pending!=msg.data[0]:
            raise PermissionError('Only the individually permitted limit READ is allowed')
        joint=self.pending
        self.pending=None
        self.used.add(joint)  # Never retry ambiguous IO.


class LimitsReadBus(ReadOnlyGsUsbBus):
    def __init__(self,*args,**kwargs):
        self.query_budget=QueryBudget()
        self.limit_query_frames=[]
        self.limit_query_tx_successful=0
        super().__init__(*args,**kwargs)

    def send(self,msg,timeout=None):
        if is_firmware_query(msg):return super().send(msg,timeout)
        try:self.query_budget.consume(msg)
        except Exception:
            self.blocked_commands+=1
            raise
        self.limit_query_frames.append(dict(id='0x472',data=bytes(msg.data).hex()))
        result=GsUsbBus.send(self,msg,timeout)
        self.limit_query_tx_successful+=1
        return result


class LimitsReader(Reader):
    def __init__(self):
        can.interfaces.BACKENDS['gs_usb']=('read_piper_joint_limits','LimitsReadBus')
        self.channel=0;self.firmware_driver='default';self.firmware=None
        self.robot=AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper',firmeware_version='default',interface='gs_usb',channel=0,
            bitrate=1000000,enable_check_can=False,auto_connect=False,timeout=.02))
        self.robot.connect()
        self.bus=self.robot.get_context().get_comm().recv_bus
        if self.robot.get_context().get_comm().send_bus is not self.bus:
            self.robot.disconnect();raise RuntimeError('Expected shared guarded RX/TX bus')
        self.gripper=self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)


def compare_limits(limits,angles):
    results=[]
    for i,limit in enumerate(limits):
        valid=(limit is not None and limit.get('joint_index')==i+1
               and all(isinstance(limit.get(k),(int,float)) and math.isfinite(limit[k])
                       for k in ['min_angle_limit','max_angle_limit','max_joint_spd'])
               and limit['min_angle_limit']<limit['max_angle_limit'] and limit['max_joint_spd']>0)
        row=dict(joint=i+1,valid_limit_response=valid)
        if valid and angles is not None:
            q=angles[i];lo=limit['min_angle_limit'];hi=limit['max_angle_limit']
            row.update(angle_deg=math.degrees(q),min_deg=math.degrees(lo),max_deg=math.degrees(hi),
                       within_reported_limits=lo<=q<=hi)
        results.append(row)
    return results


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    reader=None
    result=dict(limits=[],responses=[],execution_allowed=False,motor_enable_commands_sent=0,
                motion_commands_sent=0,reset_commands_sent=0,settings_commands_sent=0)
    try:
        reader=LimitsReader();time.sleep(.8);reader.query_firmware()
        before=reader.snapshot();write_json(args.output/'before.json',before)
        fw=before.get('firmware') or {}
        if fw.get('software_version')!='S-V1.5-7' or fw.get('hardware_version')!='H-V1.2-1':
            raise RuntimeError('Device identity not verified; no limit queries sent')
        for joint in range(1,7):
            reader.bus.query_budget.allow(joint)
            limit=reader.robot.get_joint_angle_vel_limits(joint,timeout=1.0,min_interval=1.0)
            result['limits'].append(plain(limit.msg) if limit is not None else None)
            result['responses'].append(dict(joint=joint,raw_0x473=reader.bus.raw_latest.get(0x473),
                                            response_timestamp=getattr(limit,'timestamp',None)))
            time.sleep(.1)
        after=reader.snapshot();write_json(args.output/'after.json',after)
        result['comparison']=compare_limits(result['limits'],after.get('joint_angles_rad'))
        result['outcome']='read_complete_no_motion'
    except Exception as exc:
        result['outcome']='read_failed_no_motion';result['error']=str(exc)
    finally:
        if reader:
            result['limit_query_frames']=reader.bus.limit_query_frames
            result['limit_query_tx_successful']=reader.bus.limit_query_tx_successful
            result['firmware_queries_sent']=reader.bus.queries_sent
            result['blocked_commands']=reader.bus.blocked_commands
            reader.close()
        write_json(args.output/'result.json',result)
        print(json.dumps(result,indent=2))


if __name__=='__main__':main()
