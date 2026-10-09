"""Offline commissioning state machine. There is deliberately NO live transport.

This tests bounded point-to-point command sequencing and stop verification. It
does not certify robot braking, collisions, firmware initialization or Windows
timing. SDK electronic emergency stop applies damping and can lower the arm;
it is NOT a hold-position command. No reset/disable/automatic resume is emitted.
"""
from dataclasses import dataclass
import math
import struct

from pyAgxArm import AgxArmFactory, create_agx_arm_config
from pyAgxArm.protocols.can_protocol.msgs.piper.default import (
    ArmMsgModeCtrl, ArmMsgMotionCtrl, ArmMsgMotorEnableDisableConfig)
from feedback_health import describe_feedback, GROUPS


def finite_six(values):
    if len(values) != 6 or any(isinstance(x, bool) or not isinstance(x, (int, float))
                               or not math.isfinite(x) for x in values):
        raise ValueError('Expected six finite numeric joint angles')
    return tuple(values)


@dataclass(frozen=True)
class JogPlan:
    start_deg: tuple
    target_deg: tuple
    enable_frames: tuple
    target_frames: tuple
    stop_frames: tuple


def compile_jog(start_deg, target_deg, limits):
    """Compile exact SDK frames offline; reject targets, never silently clamp.

    Initial encoder values are not mechanically classified by numeric limits.
    The explicitly requested target must be within the queried parameters.
    The two-degree cap and one-percent speed are LOCAL commissioning choices,
    not a claim about physical speed or a manufacturer safety rating.
    """
    start, target = finite_six(start_deg), finite_six(target_deg)
    target = tuple(round(x * 1000) / 1000 for x in target)
    if not .05 <= max(abs(a-b) for a,b in zip(start,target)) <= 2:
        raise ValueError('Commissioning jog must be 0.05..2 degrees; no scan trajectory')
    if len(limits) != 6:
        raise ValueError('Need six controller limit responses')
    for j,(q,lim) in enumerate(zip(target,limits),1):
        if not isinstance(lim,dict) or lim.get('joint_index') != j:
            raise ValueError('Missing or mismatched controller limits')
        lo,hi=lim.get('min_angle_limit'),lim.get('max_angle_limit')
        if not all(isinstance(x,(int,float)) and math.isfinite(x) for x in (lo,hi)) or lo>=hi:
            raise ValueError('Invalid controller limits')
        if not lo <= math.radians(q) <= hi:
            raise ValueError(f'J{j} target outside queried parameters; no clamping')
    robot=AgxArmFactory.create_arm(create_agx_arm_config(robot='piper',
        firmeware_version='default',enable_check_can=False,auto_connect=False))
    try:
        if robot.is_connected():raise RuntimeError('Compiler must never connect')
        def packet(msg):
            frame=robot._parser.pack(msg)
            return frame.arbitration_id,bytes(frame.data)
        # Individual joint enables: no all-motor selector, no gripper packet.
        enables=tuple(packet(ArmMsgMotorEnableDisableConfig(joint_index=j,enable_flag=2)) for j in range(1,7))
        mode=packet(ArmMsgModeCtrl(ctrl_mode=1,move_mode=1,move_spd_rate_ctrl=1,mit_mode=0))
        targets=tuple(packet(m) for m in robot._deal_move_j_msgs([math.radians(q) for q in target]))
        decoded=tuple(x/1000 for _,data in targets for x in struct.unpack('>ii',data))
        if decoded != target:raise ValueError('SDK changed target; refusing compiled plan')
        return JogPlan(start,target,enables,(mode,*targets),(packet(ArmMsgMotionCtrl(1)),))
    finally:robot.disconnect()


class DryRunTransport:
    """In-memory transcript only. No CAN imports, USB opening, or transmission."""
    def __init__(self):self.events=[]
    def record(self,kind,frames,now):
        self.events.append(dict(kind=kind,time=now,frames=[
            dict(can_id=f'0x{i:03X}',data_hex=data.hex()) for i,data in frames]))


class MotionSupervisor:
    """One session, fail-closed; STOP_REQUESTED is never confused with STOPPED.

    `now` must share the snapshot's perf_counter domain. No cached snapshot can
    prove enable or stop. Production transport is deliberately not accepted.
    """
    def __init__(self, plan, transport):
        if type(transport) is not DryRunTransport:
            raise TypeError('Physical execution is not commissioned; dry-run only')
        self.plan=plan;self.transport=transport;self.state='IDLE'
        self.reason=None;self.started=None;self.last_now=None
        self.stop_at=None;self.stop_counts={};self.last_counts={}
        self.stable_since=None;self.stable_reference=None;self.stable_samples=0

    def _feedback(self,s,now):
        t=s.get('snapshot_perf_s')
        if not isinstance(t,(int,float)) or not math.isfinite(t) or not 0<=now-t<=.15:
            return None,'snapshot age invalid'
        health=describe_feedback({**s,'rx_age_s':{
            k: v+now-t if isinstance(v,(int,float)) and not isinstance(v,bool) else v
            for k,v in s.get('rx_age_s',{}).items()}})
        if any(not g['fresh'] for g in health['groups'].values()):
            return None,'missing/stale feedback: '+','.join(k for k,v in health['groups'].items() if not v['fresh'])
        if s.get('transport_health') is None or health['transport_faults_since_connect']:
            return None,'transport health unknown or faulted'
        fw=s.get('firmware') or {}
        if fw.get('software_version')!='S-V1.5-7' or fw.get('hardware_version')!='H-V1.2-1':
            return None,'identity/profile not verified'
        if s.get('firmware_driver')!='default':return None,'profile mismatch'
        c=s.get('frozen_control') or {}
        if not c.get('valid_payloads'):return None,'invalid frozen payloads'
        try:
            finite_six(c['joint_millidegrees']);codes=c['driver_codes']
            for key in ('ctrl_mode','arm_status','motion_mode','teach_status','motion_status','trajectory_num','err_code'):
                if type(c[key]) is not int:return None,'invalid controller fields'
            if c['motion_status'] not in (0,1):return None,'unknown motion status'
            if len(codes)!=6 or any(type(x) is not int or not 0<=x<=255 for x in codes):
                return None,'invalid driver codes'
            if self.state not in ('STOP_REQUESTED','STOP_UNCONFIRMED'):
                if c['err_code'] or c['arm_status'] or c['teach_status'] or c['trajectory_num']:
                    return None,'controller fault or incompatible mode'
                if c['ctrl_mode'] not in (0,1) or c['motion_mode'] not in (0,1):
                    return None,'controller not in standby/position mode'
                if any(x & ~0x40 for x in codes):return None,'driver fault'
            return c,None
        except (KeyError,TypeError,ValueError):return None,'invalid frozen payloads'

    def start(self,s,now):
        if self.state!='IDLE':raise RuntimeError('No automatic retry/resume')
        c,error=self._feedback(s,now)
        if error:raise ValueError(error)
        if c['ctrl_mode']!=0 or c['motion_status']!=0 or any(c['driver_codes']):
            raise ValueError('Start must be idle with all joints disabled and normal')
        grip=s.get('gripper') or {}
        if grip.get('status_code')!=0:raise ValueError('Gripper must be disabled and normal')
        if max(abs(q/1000-r) for q,r in zip(c['joint_millidegrees'],self.plan.start_deg))>.2:
            raise ValueError('Plan start is stale')
        self.started=self.last_now=now;self.last_counts=dict(s.get('rx_ids',{}))
        self.state='ENABLING'
        self.transport.record('enable_requested',self.plan.enable_frames,now)

    def request_stop(self,s,now,reason):
        if self.state in ('STOP_REQUESTED','STOP_UNCONFIRMED','STOPPED','DONE','IDLE'):return
        # Consume the stop permit before recording/sending: never retry an
        # ambiguous send. Future production transport must preserve this.
        self.state='STOP_REQUESTED';self.reason=reason;self.stop_at=now
        self.stop_counts=dict(s.get('rx_ids',{}));self.last_counts=self.stop_counts.copy()
        self.transport.record('damped_stop_requested',self.plan.stop_frames,now)

    def update(self,s,now):
        if self.state in ('IDLE','DONE','STOPPED','STOP_UNCONFIRMED'):return self.state
        if not math.isfinite(now) or now < self.last_now:
            self.request_stop(s,self.last_now,'invalid supervisor clock')
            self.state='STOP_UNCONFIRMED';return self.state
        if now-self.last_now>.15:
            self.request_stop(s,now,'supervisor deadline missed')
        self.last_now=now
        c,error=self._feedback(s,now)
        if self.state=='STOP_REQUESTED':
            self._verify_stop(s,c,now,error)
            return self.state
        if error:
            self.request_stop(s,now,error);return self.state
        q=tuple(v/1000 for v in c['joint_millidegrees'])
        if self.state=='ENABLING':
            if max(abs(a-b) for a,b in zip(q,self.plan.start_deg))>.2:
                self.request_stop(s,now,'unexpected motion while enabling')
            elif now-self.started>2:
                self.request_stop(s,now,'enable feedback timeout')
            elif all(x==0x40 for x in c['driver_codes']) and self._new_counts(s,self.last_counts,range(0x261,0x267)):
                self.state='RUNNING'
                self.transport.record('bounded_target_requested',self.plan.target_frames,now)
                self.last_counts=dict(s['rx_ids'])
        elif self.state=='RUNNING':
            if c['ctrl_mode']!=1 or c['motion_mode']!=1 or any(x!=0x40 for x in c['driver_codes']):
                self.request_stop(s,now,'control mode/enable lost')
            elif any(x<min(a,b)-.25 or x>max(a,b)+.25 for x,a,b in zip(q,self.plan.start_deg,self.plan.target_deg)):
                self.request_stop(s,now,'joint envelope exceeded')
            elif now-self.started>15:
                self.request_stop(s,now,'target timeout')
            elif c['motion_status']==0 and max(abs(a-b) for a,b in zip(q,self.plan.target_deg))<=.15 and self._new_counts(s,self.last_counts,(*GROUPS['joint_position'],0x2A1)):
                self.state='DONE'  # No disable/reset: powering off can drop arm.
        return self.state

    @staticmethod
    def _new_counts(s,baseline,ids):
        counts=s.get('rx_ids',{})
        return all(counts.get(f'0x{i:03X}',0)>baseline.get(f'0x{i:03X}',0) for i in ids)

    def _verify_stop(self,s,c,now,error):
        if now-self.stop_at>2:
            self.state='STOP_UNCONFIRMED';return
        ids=(*GROUPS['joint_position'],0x2A1)
        if error or c['arm_status']!=1 or not self._new_counts(s,self.last_counts,ids):
            self.stable_since=None;self.stable_samples=0;return
        # Status alone is not proof of zero motion (damping can lower arm).
        q=tuple(v/1000 for v in c['joint_millidegrees'])
        self.last_counts=dict(s['rx_ids'])
        if self.stable_since is None or max(abs(a-b) for a,b in zip(q,self.stable_reference))>.1:
            self.stable_since=now;self.stable_reference=q;self.stable_samples=1
        else:self.stable_samples+=1
        if now-self.stable_since>=.5 and self.stable_samples>=5:
            self.state='STOPPED'  # feedback-observed stationary, not safety certification
