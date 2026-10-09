"""Paced RGB-D BAG replay to the live mapper. Explicitly offline; no robot IO."""
import argparse
import json
import subprocess
import time
from pathlib import Path
import cv2
import numpy as np
import pyrealsense2 as rs
from live_mapping import LiveMapper
from table_scan import ROOT,DEFAULT_SOURCE


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=DEFAULT_SOURCE)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=35.)
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=8770)
    args=p.parse_args()
    if not 1<=args.seconds<=300:p.error('--seconds must be in [1,300]')
    rows=[json.loads(l) for l in (args.source/'frames.jsonl').read_text().splitlines()]
    lookup={}
    for row in rows:lookup.setdefault(row['depth_number'],row)
    meta=json.loads((args.source/'metadata.json').read_text())
    mapper=LiveMapper(meta['streams']['depth'],args.output,args.host,args.port,replay=True)
    pipe=rs.pipeline();cfg=rs.config();cfg.enable_device_from_file(str(args.source/'rgbd.bag'),False)
    rgb=cv2.VideoCapture(str(args.source/'rgb.mp4'))
    process=None;started=False;first=None;last=-1;rgb_index=-1;submitted=0;video_index=0
    try:
        profile=pipe.start(cfg);started=True;profile.get_device().as_playback().set_real_time(False)
        command=[str(ROOT/'tmp/video_tools/ffmpeg.exe'),'-hide_banner','-loglevel','error','-n',
            '-f','rawvideo','-pix_fmt','bgr24','-s','1280x720','-r','5','-i','-',
            '-an','-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(args.output/'demo.mp4')]
        process=subprocess.Popen(command,stdin=subprocess.PIPE)
        while True:
            frames=pipe.wait_for_frames(3000);depth=frames.get_depth_frame()
            if not depth:continue
            number=depth.get_frame_number();row=lookup.get(number)
            if row is None or number<=last:continue
            if abs(depth.get_timestamp()-row['depth_timestamp_ms'])>.01:raise RuntimeError('Depth/log mismatch')
            if first is None:first=row['depth_timestamp_ms'];start=time.perf_counter()
            source_s=(row['depth_timestamp_ms']-first)/1000
            if source_s>args.seconds:break
            while rgb_index<row['index']:
                ok,image=rgb.read();rgb_index+=1
                if not ok:raise RuntimeError('Recorded RGB exhausted')
            delay=start+source_s-time.perf_counter()
            if delay>0:time.sleep(delay)
            packet={**row,'original_host_arrival_perf_s':row['host_arrival_perf_s'],
                    'host_arrival_perf_s':start+source_s}
            mapper.submit(np.asanyarray(depth.get_data())*meta['depth_scale'],cv2.cvtColor(image,cv2.COLOR_BGR2RGB),packet)
            submitted+=1;last=number
            # Causal screen capture: never show a map that had not yet been published.
            now=time.perf_counter()
            while video_index/5 <= min(source_s,args.seconds):
                with mapper.lock:jpeg=mapper.jpeg;published=mapper.published
                im=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR) if jpeg else np.full((720,1280,3),245,np.uint8)
                age=None if published is None else now-published
                cv2.rectangle(im,(0,0),(1280,20),(245,245,245),-1)
                cv2.putText(im,f'OFFLINE PACED REPLAY | source {source_s:.2f} s | screen age {age if age is not None else -1:.2f} s',
                            (25,15),cv2.FONT_HERSHEY_SIMPLEX,.42,(38,59,72),1,cv2.LINE_AA)
                process.stdin.write(im.tobytes());video_index+=1
        time.sleep(.2)
    finally:
        if started:pipe.stop()
        rgb.release();mapper.close()
        if process:
            process.stdin.close()
            if process.wait(timeout=30):raise RuntimeError('Demo encode failed')
    report=json.loads((args.output/'result.json').read_text())
    report.update(source=str(args.source),source_duration_s=source_s,submitted=submitted,
                  replay=True,hardware_access=False,video_fps=5,video_frames=video_index,
                  note='Old horizontal-scan recording, paced by depth timestamps. NOT an acquired table scan or proof of live sensor latency.')
    (args.output/'result.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
