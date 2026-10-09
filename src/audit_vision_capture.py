"""Offline audit of recorded scan; no camera, CAN, or motion access."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def stats(values):
    a=np.asarray(values,float)
    return dict(min=float(a.min()),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max()))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    with (args.capture/'frames.jsonl').open(encoding='utf-8') as f:rows=[json.loads(line) for line in f if line.strip()]
    if len(rows)<2:raise ValueError('Need at least two frames')
    host=np.array([r['host_arrival_perf_s'] for r in rows]);dt=np.diff(host)
    domain_match=all(r['depth_domain']==r['color_domain'] for r in rows)
    depth=np.array([r['depth_number'] for r in rows]);color=np.array([r['color_number'] for r in rows])
    q=np.array([r['joints_after']['q_rad'] if r['joints_after']['q_rad'] is not None else [np.nan]*6 for r in rows],float)
    qdeg=np.rad2deg(q)
    report=dict(frames=len(rows),host_duration_s=float(host[-1]-host[0]),
        effective_host_fps=float((len(rows)-1)/(host[-1]-host[0])),host_interval_ms=stats(dt*1000),
        nonmonotonic_host_intervals=int(np.sum(dt<=0)),
        depth_missing_numbers=int(np.maximum(np.diff(depth)-1,0).sum()),
        color_missing_numbers=int(np.maximum(np.diff(color)-1,0).sum()),
        depth_duplicate_or_backward=int(np.sum(np.diff(depth)<=0)),
        color_duplicate_or_backward=int(np.sum(np.diff(color)<=0)),
        joint_invalid_rows=int(np.sum(~np.isfinite(q).all(axis=1))),
        joint_range_deg=(np.nanmax(qdeg,axis=0)-np.nanmin(qdeg,axis=0)).tolist(),
        final_minus_initial_deg=(qdeg[-1]-qdeg[0]).tolist(),
        identical_adjacent_joint_rows=int(np.sum(np.all(np.diff(q,axis=0)==0,axis=1))),
        sdk_read_bracket_ms=stats([(r['joints_after']['read_end_perf_s']-r['joints_before']['read_begin_perf_s'])*1000 for r in rows]),
        rgb_depth_same_domain=domain_match,
        rgb_minus_depth_ms=stats([r['color_timestamp_ms']-r['depth_timestamp_ms'] for r in rows]) if domain_match else None,
        pose_freshness_verified=False,exposure_synchronized=False,world_mapping_evaluated=False)
    cap=cv2.VideoCapture(str(args.capture/'rgb.mp4'))
    if not cap.isOpened():raise RuntimeError('Cannot decode RGB video')
    report['video_frames']=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));report['video_fps']=float(cap.get(cv2.CAP_PROP_FPS))
    tiles=[]
    for index in np.linspace(0,len(rows)-1,9,dtype=int):
        cap.set(cv2.CAP_PROP_POS_FRAMES,int(index));ok,frame=cap.read()
        if not ok:raise RuntimeError(f'Cannot decode frame {index}')
        tile=np.full((276,320,3),245,np.uint8);tile[:240]=cv2.resize(frame,(320,240))
        cv2.putText(tile,f'{host[index]-host[0]:.1f} s | frame {index}',(10,264),cv2.FONT_HERSHEY_SIMPLEX,.5,(40,40,40),1,cv2.LINE_AA)
        tiles.append(tile)
    cap.release()
    sheet=np.vstack([np.hstack(tiles[i:i+3]) for i in range(0,9,3)])
    if not cv2.imwrite(str(args.output/'rgb_contact_sheet.jpg'),sheet):raise RuntimeError('Image export failed')
    with (args.output/'audit.json').open('x',encoding='utf-8') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
