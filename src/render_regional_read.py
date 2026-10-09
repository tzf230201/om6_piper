"""Illustrated regional retrieval of an already saved map, not a live scan replay."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np
from PIL import Image,ImageDraw
from regional_read_study import ROOT,SOURCE,Reader,write,sha
from archive_retention_study import REVIEW
from render_live_scan_review import Review,page,text,TEAL,GRAY


def corners(lower,upper):return np.array(list(itertools.product(*zip(lower,upper))))


def draw_graph(g,lower,upper,view,size=(384,246),overview=False):
    w,h=size;im=np.full((h,w,3),250,np.uint8)
    basis={'top':np.array([[1,0],[0,1],[0,0]]),'side':np.array([[1,0],[0,0],[0,1]]),
           'oblique':np.array([[.707,-.354],[.707,.354],[0,.866]])}[view]
    points=g['points'];box=corners(np.asarray(lower)*.08,np.asarray(upper)*.08)
    bounds=points if overview else box
    flat=bounds@basis;lo=flat.min(axis=0);hi=flat.max(axis=0);centre=(lo+hi)/2
    scale=min((w-32)/max(hi[0]-lo[0],.01),(h-48)/max(hi[1]-lo[1],.01))*.92
    def project(p):
        a=np.asarray(p)@basis
        return np.column_stack((w/2+(a[:,0]-centre[0])*scale,h/2-(a[:,1]-centre[1])*scale)).astype('int32')
    uv=project(points)
    if overview:
        p=uv[::max(1,len(uv)//18000)];valid=(p[:,0]>=0)&(p[:,0]<w)&(p[:,1]>=0)&(p[:,1]<h)
        im[p[valid,1],p[valid,0]]=(180,188,192)
        box2=project(box);a=box2.min(axis=0);b=box2.max(axis=0)
        cv2.rectangle(im,tuple(a),tuple(b),(32,105,224),2)
    else:
        edges=g['edges'][::max(1,int(np.ceil(len(g['edges'])/9000)))]
        if len(edges):cv2.polylines(im,list(uv[edges]),False,(172,189,187),1,cv2.LINE_AA)
        selected=np.arange(0,len(points),max(1,int(np.ceil(len(points)/18000))))
        p=uv[selected];valid=(p[:,0]>=0)&(p[:,0]<w)&(p[:,1]>=0)&(p[:,1]<h)
        zlo=lower[2]*.08;zhi=upper[2]*.08
        color=cv2.applyColorMap((np.clip((points[selected,2]-zlo)/(zhi-zlo),0,1)*255).astype('u1'),cv2.COLORMAP_VIRIDIS)
        if len(p):im[p[valid,1],p[valid,0]]=color[:,0,:][valid]
        bar=int(.05*scale);cv2.line(im,(16,h-18),(16+bar,h-18),(55,55,55),2)
        cv2.putText(im,'50 mm',(16,h-27),cv2.FONT_HERSHEY_SIMPLEX,.38,(55,55,55),1,cv2.LINE_AA)
    return Image.fromarray(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))


class Movie:
    def __init__(self,root):
        self.root=root;self.summary=json.loads((root/'summary.json').read_text())
        self.jobs=json.loads((root/'jobs_vectorized/summary.json').read_text())
        self.old_jobs=json.loads((root/'jobs/summary.json').read_text())
        self.rows=json.loads((root/'jobs_vectorized/measurements.json').read_text())['trace']
        self.full=dict(np.load(SOURCE/'final.npz'));self.old=Review(REVIEW)
        self.rgb=self.old.rgb(self.old.scan_index)
        self.cases=[r for r in self.rows if r['width']==2 and r['index'] in (0,1,2,4)]
        assert all(r['complete'] for r in self.cases)
        case=self.cases[0];self.parts=[];reader=Reader(SOURCE/'archive.sqlite')
        try:
            for s in case['steps']:
                r=np.asarray(s['region']);g,stats=reader.query(r,r+1,regions=1)
                assert stats['durable_batch']==case['revision'][0]
                self.parts.append(g)
        finally:reader.close()

    def intro(self):
        im=page('Load the part of the map we need',
            'Regional read-back from the saved Piper + D405 table scan',
            'Offline retrieval demonstration after the scan. No robot commands or live camera-to-display timing.')
        im.paste(self.rgb.resize((544,408)),(28,154))
        for y,line,bold in ((160,'The scan has already been saved.',True),(210,'The viewer may only need one small area.',False),
            (293,'Loading the whole archive brings back',False),(332,'283,706 points and 1,749,244 links.',False),
            (416,'Here, we ask for nearby blocks instead.',True),(464,'The rest of the archive stays on disk.',False)):
            text(im,(608,y),line,23,bold)
        text(im,(28,611),'The camera image is recording context, not a new live measurement.',24,True)
        return im

    def problem(self):
        im=page('A small area can still contain a lot of data',
            '48 query boxes per size; the same boxes are repeated twice | One development recording',
            'Limits: 64 regions, 8 MiB raw input and 8 MiB compressed input per read. Oversized requests fail, not truncate.')
        for x,title,key in ((28,'8 cm cube','width1_r1'),(448,'16 cm cube','width2_r1'),(868,'32 cm cube','width4_r1')):
            r=self.summary['runs'][key];ImageDraw.Draw(im).rounded_rectangle((x,172,x+384,392),radius=12,fill='white',outline='#ccd9e0')
            text(im,(x+20,195),title,28,True);text(im,(x+20,259),f"{r['success']} / 48 reads completed",23,True,color=TEAL)
            text(im,(x+20,316),f"{48-r['success']} exceeded the read budget",20)
        text(im,(28,451),'A fast average does not help when a dense patch is rejected.',25,True)
        text(im,(28,508),'We split the request into blocks, while keeping a limit on the complete result.',23)
        text(im,(28,603),'No result is labelled complete until every required block and link is available.',23,True)
        return im

    def partial(self,n):
        c=self.cases[0];parts=self.parts[:n];offset=0;pairs=[]
        for g in parts:pairs.append(g['edges']+offset);offset+=len(g['points'])
        g=dict(points=np.concatenate([g['points'] for g in parts]),edges=np.concatenate(pairs))
        im=page('Read one block, then give control back',
            'Illustrated steps on a fixed saved revision; slowed down for explanation, not video-rate measurements',
            'The overview/illustrations are rendered offline. Each native read has a separate bounded transaction.')
        im.paste(draw_graph(g,c['lower'],c['upper'],'oblique',(680,470)),(28,155))
        text(im,(743,162),f'{n} / 8 blocks read',30,True,color=TEAL)
        text(im,(743,221),'INCOMPLETE',26,True,color='#b26019')
        text(im,(743,277),'Links across block boundaries',22)
        text(im,(743,312),'are kept until they can be joined.',22)
        text(im,(743,384),f"Measured step: {c['steps'][n-1]['step_ms']:.1f} ms",24,True)
        text(im,(743,428),'A short step is not a complete map.',21)
        im.paste(self.rgb.resize((256,192)),(743,468))
        text(im,(1020,480),'Recorded',19);text(im,(1020,509),'camera context',19)
        return im

    def patch(self,i):
        c=self.cases[i];g=dict(np.load(self.root/'jobs_vectorized/snapshots'/f"2_{c['index']:03d}.npz"))
        im=page('The requested patch, with its boundary links restored',
            f"Completed 16 cm query #{c['index']} | Saved revision {int(c['revision'][0])} | Exact match to the full-map crop",
            'Three views of the same retrieved graph. Missing observations remain unknown; this is not a current obstacle map.')
        text(im,(28,133),'Recorded camera context',18,True);im.paste(self.rgb.resize((256,192)),(28,163))
        text(im,(316,133),'Location in saved map',18,True)
        im.paste(draw_graph(self.full,c['lower'],c['upper'],'top',(256,192),True),(316,163))
        text(im,(619,163),f"{c['nodes']:,} points | {c['edges']:,} links",25,True,color=TEAL)
        text(im,(619,209),f"{c['restored_boundary_edges']:,} links joined across blocks",22)
        text(im,(619,251),f"Complete query: {c['total_ms']:.1f} ms",24,True)
        text(im,(619,297),'Completion includes all reads and the final join.',21)
        for x,view,title in ((28,'oblique','Oblique'),(448,'top','Top'),(868,'side','Side')):
            text(im,(x,366),title,21,True);im.paste(draw_graph(g,c['lower'],c['upper'],view),(x,397))
        text(im,(28,652),f"Colour: height Z = {c['lower'][2]*.08:.2f} to {c['upper'][2]*.08:.2f} m | Each scale bar is 50 mm",17,color=GRAY)
        return im

    def results(self):
        im=page('Small read steps; a longer wait for a complete patch',
            'Same saved archive and query boxes | Final join uses compiled NumPy operations instead of Python per-edge loops',
            'Query timing is not camera-to-display latency. Complete-job p95 excludes rejected jobs; their counts stay visible.')
        for x,label in ((28,'Measurement'),(650,'16 cm query'),(970,'32 cm query')):text(im,(x,146),label,25,True)
        rows=[('Complete queries',lambda r:f"{r['complete']} / 48"),
              ('One read step, p95',lambda r:f"{r['step_ms']['p95']:.1f} ms"),
              ('Whole job, p95',lambda r:f"{r['complete_job_ms']['p95']:.1f} ms"),
              ('Final join, p95',lambda r:f"{r['assembly_ms']['p95']:.1f} ms"),
              ('Rejected jobs',lambda r:str(r['failed']))]
        for n,(label,fn) in enumerate(rows):
            y=212+n*63;text(im,(28,y),label,24)
            for x,key in ((650,'2'),(970,'4')):text(im,(x,y),fn(self.jobs['groups'][key]),24,True,color=TEAL)
        old=self.old_jobs['groups']['2']['assembly_ms']['p95'];new=self.jobs['groups']['2']['assembly_ms']['p95']
        text(im,(28,568),f'The 16 cm join fell from {old:.1f} to {new:.1f} ms without changing the graph.',23,True)
        text(im,(28,620),'89 completed jobs checked: geometry, links, times and source witnesses match.',22)
        return im

    def limits(self):
        im=page('What is ready, and what still needs work',
            'Regional retrieval now has bounded work, explicit failures and exact boundary-link recovery.',
            'One recording, nominal CAD alignment. No new accuracy, dynamic-object or 30 Hz end-to-end realtime claim.')
        lines=[(163,'A reader does not flush or control the writer.',True),
               (218,'If the saved revision changes midway, discard the unfinished job.',False),
               (311,'A complete patch can still take longer than one camera frame.',True),
               (366,'Run retrieval and joining on a consumer worker, not camera capture.',False),
               (461,'Next: test sustained writer + reader load and different recordings.',True),
               (516,'Then integrate with live mapping and measure the whole pipeline.',False)]
        for y,line,bold in lines:text(im,(28,y),line,25,bold)
        text(im,(28,614),'The old archive, live mapper and earlier videos are preserved.',23,color=GRAY)
        return im


def main(root,out):
    out.mkdir(parents=True,exist_ok=False);m=Movie(root);fps=30;frames=0;timeline=[]
    scenes=[('intro',7.,m.intro),('problem',9.,m.problem)]
    scenes += [(f'read_{n}',.9,lambda n=n:m.partial(n)) for n in range(1,9)]
    scenes += [(f'patch_{n}',6.,lambda n=n:m.patch(n)) for n in range(4)]
    scenes += [('results',12.,m.results),('limits',10.,m.limits)]
    ff=ROOT/'tmp/video_tools/ffmpeg.exe';fp=ROOT/'tmp/video_tools/ffprobe.exe'
    proc=subprocess.Popen([str(ff),'-v','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r',str(fps),'-i','-',
        '-an','-c:v','libx264','-crf','21','-preset','fast','-threads','4','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'demo.mp4')],stdin=subprocess.PIPE)
    try:
        for name,duration,fn in scenes:
            im=fn();im.save(out/(name+'.png'));begin=frames
            for _ in range(round(duration*fps)):proc.stdin.write(im.tobytes());frames+=1
            timeline.append(dict(scene=name,start_s=begin/fps,end_s=frames/fps));print('VIDEO',name,flush=True)
        proc.stdin.close()
        if proc.wait(timeout=30):raise RuntimeError('Encoder failed')
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
    subprocess.run([str(ff),'-v','error','-xerror','-i',str(out/'demo.mp4'),'-f','null','-'],check=True)
    probe=json.loads(subprocess.check_output([str(fp),'-v','error','-count_frames','-show_entries','stream=width,height,nb_read_frames,r_frame_rate:format=duration,size','-of','json',str(out/'demo.mp4')]))
    assert int(probe['streams'][0]['nb_read_frames'])==frames
    qa=out/'encoded_qa';qa.mkdir()
    for scene in timeline:
        subprocess.run([str(ff),'-v','error','-i',str(out/'demo.mp4'),'-ss',str((scene['start_s']+scene['end_s'])/2),'-frames:v','1','-threads','1',str(qa/(scene['scene']+'.png'))],check=True)
    write(out/'verification.json',dict(ffprobe=probe,full_decode_passed=True,sha256=sha(out/'demo.mp4'),
        timeline=timeline,renderer_sha256=sha(Path(__file__)),archive_sha256=sha(SOURCE/'archive.sqlite'),
        scope='Illustrated reads from a fixed post-scan archive. No dataset-clock/live-map synchronization is claimed.',hardware_access=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    main(a.root.resolve(),a.output.resolve())
