"""Bounded latest-frame RGB-D mapper. NO robot connection, enable or motion APIs.

Shares VisionCapture's camera/feedback. CAD + host-receipt pose is an estimate,
not exposure-calibrated odometry. Recent geometry and history stay separate.
"""
import json
import threading
import time
from collections import Counter, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import cv2
import numpy as np
from replay_robot_mapping import NativeGraph, PROFILE, camera_world_pose, load_model, summary


class LatestSlot:
    def __init__(self):
        self.lock=threading.Lock();self.event=threading.Event();self.item=None;self.dropped=0

    def put(self,item):
        with self.lock:
            if self.item is not None:self.dropped+=1
            self.item=item;self.event.set()

    def get(self,timeout=.1):
        if not self.event.wait(timeout):return None
        with self.lock:
            item=self.item;self.item=None;self.event.clear()
            return item


def pose_admission(record,now,replay=False):
    if not 0 <= now-record['host_arrival_perf_s'] <= .15:return 'frame_queue_age'
    q=(record.get('joints_after') or {}).get('q_rad')
    if q is None or np.shape(q)!=(6,) or not np.isfinite(q).all():return 'missing_pose'
    if not replay:
        if not record['joints_after'].get('cache_updates_recent'):return 'joint_cache_not_updating'
        if 'system_time' not in record['depth_domain'] and 'global_time' not in record['depth_domain']:
            return 'depth_clock_not_host_comparable'
        lag=record['host_arrival_wall_s']-record['depth_timestamp_ms']/1000
        if not -.02 <= lag <= .15:return 'depth_timestamp_age'
    return None


def label(im,text,xy,size=.6,color=(38,59,72)):
    cv2.putText(im,text,xy,cv2.FONT_HERSHEY_SIMPLEX,size,color,1,cv2.LINE_AA)


def graph_panel(points,edges,pose,view,size=(600,380),height_range=(0.,1.),bounds_points=None):
    w,h=size;im=np.full((h,w,3),250,np.uint8)
    # Fit only the current snapshot, never future geometry, and reserve a header.
    if view=='top':basis=np.array([[1,0],[0,1],[0,0.]])
    elif view=='side':basis=np.array([[1,0],[0,0],[0,1.]])
    else:basis=np.array([[.707,-.354],[.707,.354],[0,.866]])
    bounds=np.array([[0,0,0],[.6,0,0],[0,0,.6]])
    if len(points):bounds=np.vstack((bounds,points))
    if bounds_points is not None:bounds=np.vstack((bounds,bounds_points))
    if pose is not None:bounds=np.vstack((bounds,pose[:3,3]))
    flat=bounds@basis;lo=flat.min(0);hi=flat.max(0)
    span=np.maximum(hi-lo,.6);centre=(lo+hi)/2
    scale=min((w-40)/span[0],(h-65)/span[1])*.92
    def project(p):
        v=np.asarray(p)@basis
        return np.column_stack((w*.5+scale*(v[:,0]-centre[0]),(h+35)*.5-scale*(v[:,1]-centre[1]))).astype(np.int32)
    grid=[]
    for k in np.arange(-.6,1.21,.1):grid.extend([[[k,-.6,0],[k,.6,0]],[[-.6,k,0],[1.2,k,0]]])
    for g in grid:cv2.polylines(im,[project(g)],False,(227,231,234),1,cv2.LINE_AA)
    if len(points):
        p=np.asarray(points);v=project(p)
        if len(edges):
            selected=edges[::max(1,int(np.ceil(len(edges)/6000)))]
            cv2.polylines(im,list(v[selected]),False,(199,206,208),1)
        order=np.arange(0,len(p),max(1,int(np.ceil(len(p)/18000))))
        uv=v[order];valid=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
        uv=uv[valid]
        low,high=height_range
        if not np.isfinite([low,high]).all() or high<=low:raise ValueError('Invalid height colour range')
        colors=cv2.applyColorMap((np.clip((p[order,2]-low)/(high-low),0,1)*255).astype('u1'),cv2.COLORMAP_VIRIDIS)[:,0,:][valid]
        im[uv[:,1],uv[:,0]]=colors
    if pose is not None:
        line=project([pose[:3,3],pose[:3,3]+.16*pose[:3,2]])
        cv2.arrowedLine(im,tuple(line[0]),tuple(line[1]),(18,117,237),2,cv2.LINE_AA,tipLength=.2)
        cv2.circle(im,tuple(line[0]),4,(18,117,237),-1)
    label(im,view.upper()+' | base frame, metres',(12,24),.53)
    return im


def dashboard(rgb,depth,recent,history,pose,status):
    im=np.full((720,1280,3),245,np.uint8)
    mode='RECORDED REPLAY' if status.get('replay') else 'LIVE INPUT'
    label(im,'Table scan | RGB-D surface mapping',(25,38),.9)
    label(im,mode+' | nominal CAD pose | not a navigation / collision map',(25,69),.57)
    im[94:334,25:345]=cv2.resize(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),(320,240))
    label(im,'RGB context (not texture registration)',(25,359),.49)
    heat=cv2.applyColorMap((np.clip(depth/1.5,0,1)*255).astype('u1'),cv2.COLORMAP_VIRIDIS)
    heat[(depth<=0)|~np.isfinite(depth)]=(180,180,180)
    im[392:632,25:345]=cv2.resize(heat,(320,240),interpolation=cv2.INTER_NEAREST)
    im[645:655,25:345]=cv2.applyColorMap(np.tile(np.linspace(0,255,320,dtype='u1'),(10,1)),cv2.COLORMAP_VIRIDIS)
    label(im,'Depth Z: 0       0.75       >=1.5 m',(25,678),.49)
    label(im,'Gray = invalid depth',(25,701),.46)
    im[94:424,375:1255]=graph_panel(*history,pose,'oblique',(880,330))
    label(im,'History: bounded, may retain changed objects',(391,448),.56)
    im[470:660,375:805]=graph_panel(*recent,pose,'top',(430,190))
    im[470:660,825:1255]=graph_panel(*recent,pose,'side',(430,190))
    label(im,'Recent buffer (0.5 s). Unobserved is UNKNOWN, not free.',(380,687),.53)
    label(im,f"Frames {status['processed']} | queue replaced {status['dropped']} | native {status['native_ms']:.1f} ms",(380,711),.47)
    return im


class LiveMapper:
    def __init__(self,intr,output,host='127.0.0.1',port=8770,replay=False):
        if any(abs(c)>1e-12 for c in intr.get('coeffs',[])):raise ValueError('Depth rectification is required')
        self.output=Path(output);self.output.mkdir(exist_ok=False,parents=True)
        self.replay=replay;self.intr=intr;self.slot=LatestSlot();self.render_slot=LatestSlot();self.stop=threading.Event()
        self.lock=threading.Lock();self.jpeg=None;self.published=None;self.started=time.perf_counter()
        self.counts=Counter();self.times=deque(maxlen=10000);self.render_times=deque(maxlen=10000);self.error=None;self.closed=False
        self.last_source=None;self.last_history=-float('inf');self.last_publish=-float('inf')
        self.model,_=load_model();self.config=json.loads(PROFILE.read_text())
        self.config['world_offset_m']=[0,0,0]  # Base coordinates; presentation lift is not physical.
        self.recent=NativeGraph(intr);self.history=None
        self.server=None
        try:
            self.history=NativeGraph(intr,180.)
            owner=self
            class Handler(BaseHTTPRequestHandler):
                def log_message(self,*_):pass
                def do_GET(self):
                    path=self.path.split('?')[0]
                    if path=='/':data=PAGE.encode();kind='text/html; charset=utf-8'
                    elif path=='/state':data=json.dumps(owner.state(),allow_nan=False).encode();kind='application/json'
                    elif path=='/view.jpg':
                        with owner.lock:data=owner.jpeg
                        kind='image/jpeg'
                        if data is None:self.send_error(503,'Waiting for accepted RGB-D/pose');return
                    else:self.send_error(404);return
                    self.send_response(200);self.send_header('Content-Type',kind)
                    self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)))
                    self.end_headers()
                    try:self.wfile.write(data)
                    except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):pass
            self.server=ThreadingHTTPServer((host,port),Handler)
            self.server.daemon_threads=True
            self.web=threading.Thread(target=self.server.serve_forever,daemon=True);self.web.start()
            self.renderer=threading.Thread(target=self.render_loop,daemon=True,name='map-display');self.renderer.start()
            self.worker=threading.Thread(target=self.run,daemon=True,name='cpp-latest-frame-map');self.worker.start()
        except Exception:
            self.recent.close()
            if self.history:self.history.close()
            if self.server:self.server.server_close()
            raise
        self.url=f'http://{host}:{self.server.server_address[1]}/'
        print('Mapping viewer:',self.url,flush=True)

    def state(self):
        with self.lock:
            age=None if self.published is None else max(0.,time.perf_counter()-self.published)
            return dict(mode='recorded_replay' if self.replay else 'live_camera',closed=self.closed,error=self.error,
                stale=age is None or age>.5,view_age_s=age,counts=dict(self.counts),queue_replaced=self.slot.dropped,
                timing_last_10000_ms=summary(list(self.times)),
                renderer_last_10000_ms=summary(list(self.render_times)),render_queue_replaced=self.render_slot.dropped,
                note='CAD estimate; host cache updates do not verify exposure-time pose. History is not a current obstacle map.')

    def submit(self,depth,rgb,record):
        if self.error:raise RuntimeError(self.error)
        if self.stop.is_set():raise RuntimeError('Mapper closed')
        # Ownership copy: RS frame buffers may be recycled after this callback.
        self.slot.put((np.array(depth,dtype=np.float64,copy=True),np.array(rgb,copy=True),record))

    def render_loop(self):
        try:
            while not self.stop.is_set():
                item=self.render_slot.get()
                if item is None:continue
                rgb,depth,recent,history,pose,status,stamp=item
                begin=time.perf_counter()
                im=dashboard(rgb,depth,recent,history,pose,status)
                ok,data=cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,85])
                if not ok:raise RuntimeError('JPEG encoding failed')
                with self.lock:
                    self.jpeg=data.tobytes();self.published=stamp
                    self.render_times.append((time.perf_counter()-begin)*1000)
        except Exception as exc:
            with self.lock:self.error=f'Renderer: {type(exc).__name__}: {exc}'

    def run(self):
        recent=(np.empty((0,3)),np.empty((0,2),int));history=recent
        try:
            with (self.output/'updates.jsonl').open('x',encoding='utf8') as log:
                while not self.stop.is_set():
                    item=self.slot.get()
                    if item is None:continue
                    depth,rgb,record=item;begin=time.perf_counter()
                    reason=pose_admission(record,begin,self.replay)
                    ident=record['depth_number'];source_stamp=record['depth_timestamp_ms']
                    if self.last_source and (ident<=self.last_source[0] or source_stamp<=self.last_source[1]):reason='nonmonotonic_source'
                    if reason:
                        with self.lock:self.counts['rejected_'+reason]+=1
                        continue
                    self.last_source=(ident,source_stamp)
                    pose=camera_world_pose(self.model,record['joints_after']['q_rad'],self.config)
                    stamp=record['host_arrival_perf_s']
                    stats=self.recent.update(depth,pose,ident,stamp)
                    if stamp-self.last_history>=.2:
                        self.history.update(depth,pose,ident,stamp);self.last_history=stamp
                    native_end=time.perf_counter()
                    with self.lock:self.counts['processed']+=1
                    render_ms=0.
                    if stamp-self.last_publish>=.2:
                        recent=self.recent.snapshot();history=self.history.snapshot()
                        status=dict(replay=self.replay,processed=self.counts['processed'],dropped=self.slot.dropped,native_ms=stats[2])
                        self.render_slot.put((rgb,depth,recent,history,pose,status,stamp))
                        self.last_publish=stamp;render_ms=(time.perf_counter()-native_end)*1000
                    total_ms=(time.perf_counter()-begin)*1000
                    with self.lock:self.times.append(total_ms)
                    log.write(json.dumps(dict(depth_number=ident,source_depth_ms=source_stamp,
                        arrival_perf_s=stamp,queue_age_ms=(begin-stamp)*1000,native_recent_ms=float(stats[2]),
                        mapping_ms=(native_end-begin)*1000,render_ms=render_ms,worker_ms=total_ms,
                        recent_nodes=int(stats[0]),recent_edges=int(stats[1]),queue_replaced=self.slot.dropped))+'\n')
        except Exception as exc:
            with self.lock:self.error=f'{type(exc).__name__}: {exc}'
        finally:
            try:
                if self.counts['processed']:
                    recent=self.recent.snapshot();history=self.history.snapshot()
                np.savez_compressed(self.output/'final.npz',recent_points=recent[0],recent_edges=recent[1],
                                    history_points=history[0],history_edges=history[1])
            finally:self.recent.close();self.history.close()

    def close(self):
        self.stop.set();self.worker.join(10);self.renderer.join(5)
        if self.worker.is_alive():raise RuntimeError('Mapper shutdown timed out')
        if self.renderer.is_alive():raise RuntimeError('Renderer shutdown timed out')
        self.closed=True
        self.server.shutdown();self.server.server_close();self.web.join(2)
        report=self.state();report.update(elapsed_s=time.perf_counter()-self.started,hardware_commands_sent=0)
        (self.output/'result.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        if self.jpeg:(self.output/'last_view.jpg').write_bytes(self.jpeg)


PAGE='''<!doctype html><html lang="en"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Piper RGB-D map</title><style>body{background:#ecf1f5;color:#193547;font:17px system-ui;margin:20px auto;max-width:1280px;padding:0 16px}header{display:flex;justify-content:space-between;gap:20px}img{width:100%;border-radius:12px;background:white}#status{padding:12px;border-radius:8px;background:#fff}p{line-height:1.5}.stale{color:#a23015}</style>
<header><h2>Wrist-camera mapping</h2><p>Read-only viewer · no motion controls</p></header>
<p id="status">Waiting for camera and joint updates...</p><img id="view" alt="Waiting for accepted RGB-D frames">
<p>Upper view: bounded scan history. Lower views: recent surfaces only. Missing observations are unknown, not free space. Camera pose is estimated from CAD and joint feedback; it is not calibrated at exposure time.</p>
<script>const status=document.getElementById('status'),view=document.getElementById('view');async function update(){try{const s=await(await fetch('/state',{cache:'no-store'})).json();status.className=s.stale?'stale':'';status.textContent=(s.stale?'STALE / WAITING — ':'UPDATING — ')+s.mode+' | view age '+(s.view_age_s==null?'none':s.view_age_s.toFixed(2)+' s')+' | processed '+(s.counts.processed||0)+' | replaced '+s.queue_replaced+(s.error?' | ERROR: '+s.error:'');view.style.opacity=s.stale?'.4':'1';if(!s.stale)view.src='/view.jpg?t='+Date.now()}catch(e){status.textContent='DISCONNECTED — no live data';status.className='stale';view.style.opacity='.3'}setTimeout(update,250)}update();</script></html>'''
