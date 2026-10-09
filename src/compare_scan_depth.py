"""Offline bag audit. No camera device, CAN, robot import, or motion command.

Compare availability of measured depth, not reconstruction accuracy. Motion
labels are proxies from host-bracketed SDK angles, not exposure-time poses.
"""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import pyrealsense2 as rs


def analyze(source,out):
    rows=[json.loads(x) for x in (source/'frames.jsonl').read_text().splitlines() if x.strip()]
    meta=json.loads((source/'metadata.json').read_text())
    host=np.array([r['host_arrival_perf_s'] for r in rows])
    q=np.rad2deg(np.array([r['joints_after']['q_rad'] for r in rows]))
    # A +/- 0.25 s window avoids labeling individual cached joint reads as holds.
    left=np.searchsorted(host,host-.25);right=np.minimum(np.searchsorted(host,host+.25),len(rows)-1)
    speed=np.max(abs(q[right]-q[left]),axis=1)/np.maximum(host[right]-host[left],1e-6)
    lookup={}
    for i,r in enumerate(rows):lookup.setdefault(r['depth_number'],i)
    pipe=rs.pipeline();config=rs.config()
    config.enable_device_from_file(str((source/'rgbd.bag').resolve()),False)
    started=False;records=[];thumbs=[];seen=set();decoded=0
    start=time.perf_counter()
    try:
        profile=pipe.start(config);started=True
        playback=profile.get_device().as_playback();playback.set_real_time(False)
        while True:
            try:frames=pipe.wait_for_frames(5000)
            except RuntimeError:
                if playback.current_status()==rs.playback_status.stopped:break
                raise
            d=frames.get_depth_frame();c=frames.get_color_frame()
            if not d:continue
            decoded+=1;n=d.get_frame_number()
            if n not in lookup or n in seen:continue
            seen.add(n);i=lookup[n]
            if i%15:continue
            raw=np.asanyarray(d.get_data());z=raw.astype(np.float32)*meta['depth_scale']
            valid=raw>0;usable=valid&(z>=.07)&(z<=1.5)
            # Availability in a closer candidate workspace, NOT a D405 accuracy claim.
            close=valid&(z>=.07)&(z<=.5)
            label='stationary_proxy' if speed[i]<.2 else 'moving_proxy' if speed[i]>1 else 'transition_proxy'
            records.append(dict(log_index=i,elapsed_s=float(host[i]-host[0]),depth_number=n,
                valid_fraction=float(valid.mean()),in_0_07_to_1_5_fraction=float(usable.mean()),
                in_0_07_to_0_5_fraction=float(close.mean()),speed_proxy_deg_s=float(speed[i]),group=label,
                median_valid_depth_m=float(np.median(z[valid])) if valid.any() else None))
            if c:
                rgb=cv2.cvtColor(np.asanyarray(c.get_data()),cv2.COLOR_RGB2BGR)
                heat=cv2.applyColorMap((np.clip(z/1.5,0,1)*255).astype(np.uint8),cv2.COLORMAP_VIRIDIS)
                heat[~valid]=(224,224,224)
                thumbs.append((records[-1],cv2.resize(rgb,(480,360)),cv2.resize(heat,(480,360))))
    finally:
        if started:pipe.stop()
    groups={}
    for label in ('all','stationary_proxy','moving_proxy','transition_proxy'):
        subset=[r for r in records if label=='all' or r['group']==label]
        groups[label]=dict(samples=len(subset),**{k:float(np.mean([r[k] for r in subset])) if subset else None
            for k in ('valid_fraction','in_0_07_to_1_5_fraction','in_0_07_to_0_5_fraction')})
    summary=dict(source=str(source),decoded_frames=decoded,matched_unique_depth_frames=len(seen),
        logged_unique_depth_frames=len(lookup),depth_samples=len(records),sample_stride_log_frames=15,
        analysis_wall_s=time.perf_counter()-start,groups=groups,
        initial_joint_deg=q[0].tolist(),peak_windowed_joint_speed_deg_s=float(speed.max()),
        caveat='One run per profile; viewpoint/content and host pose timing confound comparison. No accuracy or mapping-latency measurement.')
    out.mkdir(parents=True,exist_ok=False)
    (out/'depth_audit.json').write_text(json.dumps(dict(summary=summary,frames=records),indent=2))
    # Decode RGB and depth before export; visualization is annotated with data time.
    writer=cv2.VideoWriter(str(out/'rgb_depth_audit.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),8,(960,468))
    if not writer.isOpened():raise RuntimeError('Cannot create audit video')
    contacts=[]
    picks=set(np.linspace(0,len(thumbs)-1,4,dtype=int))
    try:
        for j,(record,rgb,depth) in enumerate(thumbs):
            canvas=np.full((468,960,3),246,np.uint8)
            canvas[42:402,:480]=rgb;canvas[42:402,480:]=depth
            title='Slower scan' if 'slow' in source.name else 'Original scan'
            cv2.putText(canvas,f"{title} | recorded time {record['elapsed_s']:.1f} s",(12,27),cv2.FONT_HERSHEY_SIMPLEX,.65,(35,35,35),1,cv2.LINE_AA)
            cv2.putText(canvas,'RGB', (420,27),cv2.FONT_HERSHEY_SIMPLEX,.55,(35,35,35),1,cv2.LINE_AA)
            cv2.putText(canvas,'Depth (m)',(500,27),cv2.FONT_HERSHEY_SIMPLEX,.55,(35,35,35),1,cv2.LINE_AA)
            cv2.putText(canvas,'Approx. 4x sampled preview',(12,428),cv2.FONT_HERSHEY_SIMPLEX,.5,(35,35,35),1,cv2.LINE_AA)
            cv2.putText(canvas,f"Pixels with depth: {100*record['valid_fraction']:.1f}%",(12,451),cv2.FONT_HERSHEY_SIMPLEX,.5,(35,35,35),1,cv2.LINE_AA)
            bar=cv2.applyColorMap(np.tile(np.linspace(0,255,300,dtype=np.uint8),(12,1)),cv2.COLORMAP_VIRIDIS)
            canvas[418:430,500:800]=bar
            for x,label in ((500,'0'),(600,'0.5'),(700,'1.0'),(785,'>=1.5')):
                cv2.putText(canvas,label,(x,449),cv2.FONT_HERSHEY_SIMPLEX,.42,(35,35,35),1,cv2.LINE_AA)
            canvas[418:431,857:872]=(224,224,224)
            cv2.putText(canvas,'Invalid',(880,430),cv2.FONT_HERSHEY_SIMPLEX,.42,(35,35,35),1,cv2.LINE_AA)
            writer.write(canvas)
            if j in picks:contacts.append(cv2.resize(canvas,(720,351)))
    finally:writer.release()
    if contacts:cv2.imwrite(str(out/'rgb_depth_contact.jpg'),np.vstack(contacts))
    print(json.dumps(summary,indent=2),flush=True)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    summaries={name:analyze(args.root/name,args.output/name) for name in ('piper_scan_vision','piper_scan_vision_slow_v1')}
    (args.output/'comparison.json').write_text(json.dumps(summaries,indent=2))


if __name__=='__main__':main()
