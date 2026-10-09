"""Audit a completed live run and reconstruct ONLY its admitted frames.

Offline BAG playback only. No robot SDK import, camera capture, or motor IO.
Geometry replay is checked against the saved live final buffers. Its timing is
never substituted for measured live timing.
"""
import argparse
import json
from pathlib import Path
import shutil
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from replay_robot_mapping import NativeGraph, PROFILE, DLL, sha, summary, reprojection_residual
from table_scan import configuration, camera_world_pose


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text(encoding='utf8').splitlines() if line.strip()]


def write(path,value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf8')


def stage_at(events,stamp):
    stage='Startup';kind='startup'
    for event in events:
        if event['host_perf_s']>stamp:break
        if event['event']=='command_segment_begin':stage=event['name'];kind='moving_command'
        elif event['event']=='command_hold_begin':stage=event['name'];kind='hold_command'
    return stage,kind


def validate_trace(rows,updates,result):
    if not rows or not updates:raise ValueError('Empty capture/mapper journal')
    for key,data in [('depth_number',rows),('depth_number',updates)]:
        if any(b[key]<=a[key] for a,b in zip(data,data[1:])):raise ValueError('Nonmonotonic frame IDs')
    lookup={r['depth_number']:r for r in rows}
    for u in updates:
        r=lookup.get(u['depth_number'])
        if r is None or r['depth_timestamp_ms']!=u['source_depth_ms'] or r['host_arrival_perf_s']!=u['arrival_perf_s']:
            raise ValueError('Mapper/capture association mismatch')
    if result['counts']['processed']!=len(updates):raise ValueError('Processed count mismatch')
    if len(updates)+result['queue_replaced']+sum(v for k,v in result['counts'].items() if k.startswith('rejected_'))!=len(rows):
        raise ValueError('Capture/mapper accounting mismatch (possibly a pending shutdown frame)')
    if any(not 0<=u['queue_age_ms']<=150 for u in updates):raise ValueError('Invalid accepted queue age')
    return lookup


def audit(source,out):
    rows=read_rows(source/'frames.jsonl');updates=read_rows(source/'live_map/updates.jsonl')
    result=json.loads((source/'live_map/result.json').read_text())
    capture=json.loads((source/'result.json').read_text());meta=json.loads((source/'metadata.json').read_text())
    events=json.loads((source/'motion_events.json').read_text())['events']
    validate_trace(rows,updates,result)
    if result['error'] or capture['error']:raise ValueError('Run has a recorded error')
    model,_,config=configuration()
    poses=np.array([camera_world_pose(model,r['joints_after']['q_rad'],config) for r in rows])
    down=np.rad2deg(np.arcsin(np.clip(-poses[:,2,2],-1,1)))
    scan_names={s['name'] for s in meta['motion_protocol']['segments'] if s['scan']}
    stages=[stage_at(events,r['host_arrival_perf_s']) for r in rows]
    moving_scan=np.array([stage in scan_names for stage,_ in stages])
    host=np.array([r['host_arrival_perf_s'] for r in rows]);duration=float(host[-1]-host[0])
    q=np.rad2deg([r['joints_after']['q_rad'] for r in rows]);targets={s['name']:s['target_deg'] for s in meta['motion_protocol']['segments']}
    settled=[]
    for event in events:
        if event['event']!='command_hold_end':continue
        ids=np.where((host<=event['host_perf_s'])&(host>=event['host_perf_s']-.3))[0]
        if len(ids):
            err=q[ids[-1]]-targets[event['name']]
            settled.append(dict(stage=event['name'],max_joint_error_deg=float(abs(err).max()),delta_deg=err.tolist()))
    groups=[]
    for stage in dict.fromkeys(s for s,_ in stages):
        selected=[u for u in updates if stage_at(events,u['arrival_perf_s'])[0]==stage]
        mask=np.array([s==stage for s,_ in stages])
        groups.append(dict(stage=stage,frames=int(mask.sum()),down_angle_deg=summary(down[mask]),
            native_ms=summary([u['native_recent_ms'] for u in selected]),
            worker_ms=summary([u['worker_ms'] for u in selected])))
    # CAN timestamps are compared to the host epoch as a diagnostic only. This
    # does not calibrate transport delay, exposure time, or cache-copy atomicity.
    pair_delta=np.array([[(r['host_arrival_wall_s']-t)*1000 for t in r['joints_after']['pair_sdk_timestamps']] for r in rows])
    report=dict(source=str(source),source_duration_s=duration,recorded_frames=len(rows),mapped_frames=len(updates),
        acquisition_hz=(len(rows)-1)/duration,admitted_mapping_hz=(len(updates)-1)/duration,
        replaced_frames=result['queue_replaced'],rejected_frames={k:v for k,v in result['counts'].items() if k.startswith('rejected_')},
        native_recent_ms=summary([u['native_recent_ms'] for u in updates]),
        mapping_ms=summary([u['mapping_ms'] for u in updates]),
        worker_ms=summary([u['worker_ms'] for u in updates]),
        arrival_to_worker_end_ms=summary([u['queue_age_ms']+u['worker_ms'] for u in updates]),
        queue_ms=summary([u['queue_age_ms'] for u in updates]),
        worker_calls_over_33_33_ms=sum(u['worker_ms']>1000/30 for u in updates),
        sensor_time_to_host_receipt_ms=summary([(r['host_arrival_wall_s']-r['depth_timestamp_ms']/1000)*1000 for r in rows]),
        rgb_depth_absolute_gap_ms=summary([abs(r['color_timestamp_ms']-r['depth_timestamp_ms']) for r in rows]),
        cache_update_watchdog_passed=sum(r['joints_after'].get('cache_updates_recent',False) for r in rows),
        receipt_minus_joint_pair_stamp_ms=summary(pair_delta.ravel()),
        timestamp_note='Pair timestamps are host/driver diagnostics, not exposure synchronization proof.',
        scan_down_angle_deg=summary(down[moving_scan]),scan_seconds=float(moving_scan.sum()/((len(rows)-1)/duration)),
        settled_waypoint_errors=settled,final_minus_initial_deg=(q[-1]-q[0]).tolist(),stages=groups,
        ground_truth_available=False,hand_eye_calibrated=False,firmware_verified=False,
        timing_scope='Measured live worker (FK + C++ + snapshot submission); excludes camera acquisition, USB transport, recording, asynchronous rendering and network/display latency.',
        recorder_error=capture['error'],mapper_error=result['error'])
    write(out/'live_audit.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('stages','settled_waypoint_errors')},indent=2),flush=True)
    return rows,updates,meta,events,poses,report


def reconstruct(source,out):
    out.mkdir(parents=True,exist_ok=False);(out/'snapshots').mkdir()
    rows,updates,meta,events,poses,report=audit(source,out)
    inputs=[source/name for name in ('metadata.json','frames.jsonl','motion_events.json','rgb.mp4','rgbd.bag','result.json')]
    inputs += [source/'live_map'/name for name in ('updates.jsonl','result.json','final.npz')]
    before={str(p.relative_to(source)):dict(bytes=p.stat().st_size,mtime_ns=p.stat().st_mtime_ns,sha256=sha(p)) for p in inputs}
    model,_,config=configuration();intr=meta['streams']['depth']
    assert not any(intr['coeffs']),'Rectification required'
    protocol=dict(source=str(source),hardware_access=False,
        replay='Exact admitted live frame IDs and host timestamps; no new observations admitted',
        recent_ttl_s=.5,history_ttl_s=180.,history_period_s=.2,cell_m=.01,stride=8,
        capacity_nodes=40000,capacity_edges=80000,world_offset_m=[0,0,0],
        pose='Recorded joints_after + nominal CAD, same transforms as live run',
        video='Offline reconstruction of live admissions, NOT a recording of browser delivery times',
        optical_profile=config,model=model,native_dll_sha256=sha(DLL),input_fingerprints=before)
    write(out/'protocol.json',protocol)
    lookup={r['depth_number']:i for i,r in enumerate(rows)};expected={u['depth_number']:u for u in updates}
    recent=NativeGraph(intr);history=NativeGraph(intr,180.)
    pipe=rs.pipeline();cfg=rs.config();cfg.enable_device_from_file(str(source/'rgbd.bag'),False)
    rgb=cv2.VideoCapture(str(source/'rgb.mp4'))
    if not rgb.isOpened() or int(rgb.get(cv2.CAP_PROP_FRAME_COUNT))!=len(rows):raise ValueError('RGB/log length mismatch')
    seen=set();captures=[];last_history=-float('inf');last_snapshot=-float('inf');started=False
    rgb_index=-1;rgb_image=None;previous=None;residuals=[];probes=[]
    last_recent=(np.empty((0,3)),np.empty((0,2),int));last_hist=last_recent
    try:
        profile=pipe.start(cfg);started=True;playback=profile.get_device().as_playback();playback.set_real_time(False)
        while len(seen)<len(expected):
            try:frames=pipe.wait_for_frames(5000)
            except RuntimeError:
                if playback.current_status()==rs.playback_status.stopped:break
                raise
            d=frames.get_depth_frame()
            if not d:continue
            number=d.get_frame_number()
            if number not in expected or number in seen:continue
            i=lookup[number];row=rows[i];u=expected[number];stamp=row['host_arrival_perf_s']
            if abs(d.get_timestamp()-row['depth_timestamp_ms'])>.01:raise ValueError('BAG/log depth timestamp mismatch')
            if seen and number<max(seen):raise ValueError('Playback order changed')
            seen.add(number);raw=np.asanyarray(d.get_data()).copy();depth=raw.astype(float)*meta['depth_scale']
            stats=recent.update(depth,poses[i],number,stamp)
            if int(stats[0])!=u['recent_nodes'] or int(stats[1])!=u['recent_edges']:
                raise ValueError(f'Live/replay graph counts differ at frame {number}')
            updated_history=stamp-last_history>=.2
            if updated_history:
                history.update(depth,poses[i],number,stamp);last_history=stamp
            if stamp-last_snapshot>=.5 or number==updates[-1]['depth_number']:
                while rgb_index<i:
                    ok,rgb_image=rgb.read();rgb_index+=1
                    if not ok:raise ValueError('RGB ended before log')
                rp,re=recent.snapshot();hp,he=history.snapshot()
                stem=f'{len(captures):04d}';stage,kind=stage_at(events,stamp)
                np.savez_compressed(out/'snapshots'/f'{stem}.npz',points=rp,edges=re,history_points=hp,history_edges=he,
                                    pose=poses[i],depth=raw)
                if not cv2.imwrite(str(out/'snapshots'/f'{stem}.jpg'),rgb_image):raise RuntimeError('RGB snapshot failed')
                capture=dict(stem=stem,log_index=i,depth_number=number,source_s=stamp-rows[0]['host_arrival_perf_s'],
                    stage=stage,command_kind=kind,down_angle_deg=float(np.rad2deg(np.arcsin(-poses[i,2,2]))),
                    recent_nodes=len(rp),recent_edges=len(re),history_nodes=len(hp),history_edges=len(he),
                    live_native_ms=u['native_recent_ms'],live_worker_ms=u['worker_ms'],
                    rgb_depth_gap_ms=row['color_timestamp_ms']-row['depth_timestamp_ms'])
                captures.append(capture);last_snapshot=stamp;last_recent=(rp,re);last_hist=(hp,he)
                if previous is not None:
                    old_depth,old_pose,old_time=previous
                    r=reprojection_residual(old_depth,depth,old_pose,poses[i],intr)
                    probes.append(dict(source_s=capture['source_s'],stage=stage,samples=len(r),absolute_mm=summary(abs(r)*1000)))
                    residuals.extend(r.tolist())
                previous=(depth,poses[i],stamp)
            if len(seen)%750==0:print(f'Reconstructed {len(seen)}/{len(expected)} admitted frames',flush=True)
        if seen!=set(expected):raise ValueError(f'Incomplete playback: {len(seen)}/{len(expected)}')
        live_final=np.load(source/'live_map/final.npz')
        reconstructed=dict(recent_points=last_recent[0],recent_edges=last_recent[1],history_points=last_hist[0],history_edges=last_hist[1])
        equality={k:bool(np.array_equal(v,live_final[k])) for k,v in reconstructed.items()}
        if not all(equality.values()):raise ValueError('Final live graph does not match reconstruction: '+str(equality))
        result=dict(captures=captures,probes=probes,final_live_array_equal=equality,admitted_frames=len(seen),
            snapshot_count=len(captures),history_snapshots_at_node_cap=sum(c['history_nodes']==40000 for c in captures),
            reprojection_samples=len(residuals),reprojection_absolute_mm=summary(abs(np.asarray(residuals))*1000),
            residual_note='Cross-view depth disagreement, including occlusion/noise/CAD/timing error; not ground-truth accuracy',
            timing_source='live_audit.json; replay wall times are not benchmark results')
        write(out/'reconstruction.json',result)
    finally:
        if started:pipe.stop()
        rgb.release();recent.close();history.close()
    for p in inputs:
        old=before[str(p.relative_to(source))]
        if (p.stat().st_size,p.stat().st_mtime_ns)!=(old['bytes'],old['mtime_ns']):raise ValueError('Source file changed during review')
    for p in (Path(__file__),Path(__file__).with_name('replay_robot_mapping.py'),Path(__file__).with_name('live_mapping.py')):
        shutil.copy2(p,out/p.name)
    print(json.dumps({k:v for k,v in result.items() if k not in ('captures','probes')},indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('source',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    reconstruct(args.source.resolve(),args.output.resolve())
