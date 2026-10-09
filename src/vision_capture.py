"""Opt-in RGB-D capture alongside an existing robot connection. No motor IO.

Joint samples bracket SDK reads at host time; they are NOT verified fresh,
exposure-synchronized, or suitable ground-truth world poses.
"""
import json
from pathlib import Path
import threading
import time
import math
from datetime import datetime
import socket


def create_capture_directory(output):
    """Atomically reserve a new run; never reopen or overwrite a previous run."""
    requested=Path(output).resolve()
    requested.parent.mkdir(parents=True,exist_ok=True)
    try:
        requested.mkdir()
        return requested
    except FileExistsError:
        if not requested.is_dir():
            raise NotADirectoryError(f'Capture output exists but is not a directory: {requested}')
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    for index in range(1000):
        suffix='' if index==0 else f'_{index:03d}'
        candidate=requested.with_name(f'{requested.name}_{stamp}{suffix}')
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise FileExistsError('Unable to reserve a unique capture directory')


def check_map_bind(host,port):
    """Probe only the local HTTP bind. No camera, CAN, or robot access.

    This preflight cannot reserve the port until camera startup; the real server
    still fails closed if another process takes it in the meantime.
    """
    if isinstance(port,bool) or not isinstance(port,int) or not 1<=port<=65535:
        raise ValueError('--map-port must be an integer in 1..65535')
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as probe:
            if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):
                probe.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
            probe.bind((host,port))
    except OSError as exc:
        if getattr(exc,'winerror',None)==10049:
            hint='This IP is not assigned to this PC. Use --map-host 127.0.0.1 for a local viewer, or restore Tailscale first.'
        elif getattr(exc,'winerror',None)==10048:
            hint='This port is already in use. Close the owning viewer or select another --map-port.'
        else:
            hint='Check the local IP, port availability and permissions. No automatic network fallback is performed.'
        raise RuntimeError(f'Cannot start mapping viewer at {host}:{port}. {hint} ({exc})') from exc


class JointCacheAudit:
    """Observe all three SDK pair caches, never infer freshness from pair 5/6 alone.

    The public aggregate timestamp is only joint_56's timestamp in this SDK.
    This is a cache-update watchdog, NOT proof of CAN/exposure synchronization.
    """
    def __init__(self):
        self.last={};self.changed={}

    def inspect(self,robot,now):
        parser=getattr(robot,'_parser',None)
        q=[];stamps=[]
        for pair,indices in (('joint_12',(1,2)),('joint_34',(3,4)),('joint_56',(5,6))):
            cached=getattr(parser,pair,None)
            stamp=getattr(cached,'timestamp',None)
            if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or stamp<=0:return {}
            values=[getattr(getattr(cached,'msg',None),'joint_'+str(i),None) for i in indices]
            if stamp!=getattr(cached,'timestamp',None):return {}
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):return {}
            previous=self.last.get(pair)
            if previous is not None and stamp<previous:
                self.changed.clear();self.last.clear();return {}
            if previous is not None and stamp>previous:self.changed[pair]=now
            self.last[pair]=stamp;q.extend(values);stamps.append(stamp)
        ages=[now-self.changed.get(pair,-float('inf')) for pair in ('joint_12','joint_34','joint_56')]
        recent=all(0<=age<=.1 for age in ages)
        return dict(cache_updates_recent=recent,cache_q_rad=q,
                    pair_sdk_timestamps=stamps,cache_observation_age_s=[x if math.isfinite(x) else None for x in ages],
                    cache_audit_note='Host-observed SDK timestamp changes; not exposure or wire-age verification')


def feedback_record(robot,audit=None):
    begin=time.perf_counter()
    feedback=robot.get_joint_angles()
    value=getattr(feedback,'msg',None)
    value=list(value) if isinstance(value,(list,tuple)) else value
    end=time.perf_counter()
    valid=isinstance(value,(list,tuple)) and len(value)==6 and all(
        isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in value)
    result=dict(read_begin_perf_s=begin,read_end_perf_s=end,
        q_rad=list(value) if valid else None,
        fresh_verified=False,exposure_synchronized=False)
    if audit is not None:
        result.update(audit.inspect(robot,end))
        # Use the same pair copies that were audited, not another aggregate read.
        if 'cache_q_rad' in result:result['q_rad']=result.pop('cache_q_rad')
    return result


class FramePairAudit:
    """Gate duplicate/regressed numbers; retain timestamp anomalies as evidence."""
    def __init__(self):
        self.last=None;self.rejected=0;self.timing_anomalies=0

    def inspect(self,depth_number,color_number,depth_ms,color_ms,depth_domain,color_domain):
        if self.last is not None and any(n<=old for n,old in zip((depth_number,color_number),self.last)):
            self.rejected+=1
            return None
        self.last=(depth_number,color_number)
        comparable=depth_domain==color_domain and all(math.isfinite(x) for x in (depth_ms,color_ms))
        delta=color_ms-depth_ms if comparable else None
        aligned=comparable and abs(delta)<=5.
        if not aligned:self.timing_anomalies+=1
        return dict(rgb_depth_timestamp_delta_ms=delta,timestamp_pair_within_5ms=aligned,
                    timestamp_check_note='Diagnostic gate only; does not establish robot exposure synchronization')


class VisionCapture:
    def __init__(self,robot,output,run_metadata=None,map_live=False,map_host='127.0.0.1',map_port=8770):
        self.robot=robot;self.requested_output=Path(output).resolve()
        self.output=create_capture_directory(output)
        if self.output!=self.requested_output:
            print('Previous capture preserved:',self.requested_output,flush=True)
        print('New vision capture directory:',self.output,flush=True)
        self.stop=threading.Event();self.ready=threading.Event()
        self.error=None;self.frames=0
        self.pair_audit=FramePairAudit()
        self.joint_audit=JointCacheAudit()
        self.map_live=map_live;self.map_host=map_host;self.map_port=map_port;self.mapper=None
        self.run_metadata=run_metadata or {}
        self.events=[]
        self.events_lock=threading.Lock()
        self.thread=threading.Thread(target=self.run,name='rgbd-evidence',daemon=True)

    def mark(self,event,**fields):
        with self.events_lock:
            self.events.append(dict(event=event,host_perf_s=time.perf_counter(),**fields))

    def start(self):
        self.thread.start()
        if not self.ready.wait(20) or self.error:
            self.stop.set();self.thread.join(5)
            raise RuntimeError('Vision startup failed: '+str(self.error or 'timeout'))

    def close(self):
        self.stop.set();self.thread.join(5)
        if self.thread.is_alive():raise RuntimeError('Vision recorder did not close')
        if self.error:print('Vision capture error (not a motor stop):',self.error)

    def run(self):
        pipeline=writer=None;started=False
        try:
            import pyrealsense2 as rs
            import cv2
            import numpy as np
            context=rs.context()
            devices=[d for d in context.query_devices() if 'D405' in d.get_info(rs.camera_info.name)]
            if len(devices)!=1:raise RuntimeError('Exactly one D405 required')
            serial=devices[0].get_info(rs.camera_info.serial_number)
            pipeline=rs.pipeline(context);config=rs.config();config.enable_device(serial)
            config.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
            config.enable_stream(rs.stream.color,640,480,rs.format.rgb8,30)
            config.enable_record_to_file(str(self.output/'rgbd.bag'))
            profile=pipeline.start(config);started=True
            metadata=dict(recorder_schema=3,serial=serial,depth_scale=profile.get_device().first_depth_sensor().get_depth_scale(),
                motion_protocol=self.run_metadata,
                requested_output=str(self.requested_output),actual_output=str(self.output),
                requested_fps=30,world_map_generated=False,live_map_requested=self.map_live,pose_freshness_verified=False,
                timing='Host receipt/SDK read brackets, NOT calibrated exposure synchronization',
                streams={})
            for name,kind in (('depth',rs.stream.depth),('color',rs.stream.color)):
                intr=profile.get_stream(kind).as_video_stream_profile().get_intrinsics()
                metadata['streams'][name]=dict(width=intr.width,height=intr.height,fx=intr.fx,fy=intr.fy,
                    cx=intr.ppx,cy=intr.ppy,model=str(intr.model),coeffs=list(intr.coeffs))
            with (self.output/'metadata.json').open('x',encoding='utf-8') as f:json.dump(metadata,f,indent=2)
            writer=cv2.VideoWriter(str(self.output/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),30,(640,480))
            if not writer.isOpened():raise RuntimeError('RGB video writer unavailable')
            if self.map_live:
                from live_mapping import LiveMapper
                from replay_robot_mapping import PROFILE
                expected=json.loads(PROFILE.read_text()).get('serial')
                # Configuration uses camera_serial in some revisions.
                expected=expected or json.loads(PROFILE.read_text()).get('camera_serial')
                if expected and serial!=expected:raise RuntimeError('Camera serial does not match the CAD optical profile')
                self.mapper=LiveMapper(metadata['streams']['depth'],self.output/'live_map',self.map_host,self.map_port)
            else:self.ready.set()
            with (self.output/'frames.jsonl').open('x',encoding='utf-8') as log:
                while not self.stop.is_set():
                    before=feedback_record(self.robot,self.joint_audit)
                    frames=pipeline.wait_for_frames(1000);arrival=time.perf_counter();wall=time.time()
                    depth,color=frames.get_depth_frame(),frames.get_color_frame()
                    after=feedback_record(self.robot,self.joint_audit)
                    if not depth or not color:continue
                    record=dict(index=self.frames,host_arrival_perf_s=arrival,host_arrival_wall_s=wall,
                        depth_number=depth.get_frame_number(),color_number=color.get_frame_number(),
                        depth_timestamp_ms=depth.get_timestamp(),color_timestamp_ms=color.get_timestamp(),
                        depth_domain=str(depth.get_frame_timestamp_domain()),color_domain=str(color.get_frame_timestamp_domain()),
                        joints_before=before,joints_after=after)
                    audit=self.pair_audit.inspect(record['depth_number'],record['color_number'],
                        record['depth_timestamp_ms'],record['color_timestamp_ms'],record['depth_domain'],record['color_domain'])
                    if audit is None:continue
                    record.update(audit)
                    if self.mapper is not None:
                        self.mapper.submit(np.asanyarray(depth.get_data())*metadata['depth_scale'],np.asanyarray(color.get_data()),record)
                        if self.mapper.counts['processed']>0:self.ready.set()
                    writer.write(cv2.cvtColor(np.asanyarray(color.get_data()),cv2.COLOR_RGB2BGR))
                    log.write(json.dumps(record)+'\n');self.frames+=1
        except Exception as exc:self.error=f'{type(exc).__name__}: {exc}'
        finally:
            if self.mapper is not None:
                try:self.mapper.close()
                except Exception as exc:self.error=self.error or f'Mapper cleanup: {exc}'
            if writer is not None:writer.release()
            if started:pipeline.stop()
            self.ready.set()
            with self.events_lock:
                with (self.output/'motion_events.json').open('w',encoding='utf-8') as f:
                    json.dump(dict(note='Command scheduling markers, not actual arrival or exposure timestamps',events=self.events),f,indent=2)
            with (self.output/'result.json').open('w',encoding='utf-8') as f:
                json.dump(dict(frames=self.frames,error=self.error,motor_commands_sent_by_recorder=0,
                    world_map_generated=bool(self.mapper and self.mapper.counts['processed']),
                    mapping_result='live_map/result.json' if self.mapper else None,
                    repeated_or_regressed_pairs_skipped=self.pair_audit.rejected,
                    rgb_depth_timestamp_anomalies=self.pair_audit.timing_anomalies,
                    rgb_video_timebase='Fixed 30 fps preview; use bag/frames.jsonl for original timestamps'),f,indent=2)
