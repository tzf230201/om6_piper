"""Offline research video: observed input, measured archive backlog, then final geometry.

No final map is shown over an earlier camera frame. No device API is called.
"""
import argparse
import bisect
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np
from PIL import Image, ImageDraw
from archive_retention_study import REVIEW, ROOT
from block_archive_study import PACK, write
from render_live_scan_review import Review, page, text, panel, height_key, TEAL, GRAY, INK

ORANGE='#b26019'


class Movie:
    def __init__(self,root,run):
        self.root=root;self.run=run;self.old=Review(REVIEW)
        self.result=json.loads((root/run/'result.json').read_text())
        if self.result['accepted']!=854 or self.result['rejected'] or not all(self.result['final_reference_parity'].values()):
            raise ValueError('This storyboard requires verified full-input parity; do not hide a failed candidate')
        self.trace=json.loads((root/run/'trace.json').read_text())
        self.times=[r['status_wall_s'] for r in self.trace]
        self.proto=json.loads((root/run/'protocol.json').read_text())
        self.inputs=json.loads((PACK/'protocol.json').read_text())['records']
        self.start=self.inputs[0]['stamp'];self.duration=self.result['source_duration_s']
        self.graph=dict(np.load(root/run/'final.npz'))
        # Canonical draw order: hash-table order is not a visual-quality improvement.
        g=self.graph;order=np.lexsort(tuple(g['keys'][:,i] for i in range(3,-1,-1)))
        remap=np.empty(len(order),dtype=int);remap[order]=np.arange(len(order))
        g['points']=g['points'][order];g['edges']=remap[g['edges']]
        edges=g['edges'];g['edges']=edges[np.lexsort((edges[:,1],edges[:,0]))]
        self.image_cache={};self.audit=[]

    def intro(self):
        im=page('Keep the map. Do not make the camera wait.',
            'A storage experiment using one recorded Piper + D405 table scan',
            'Offline replay | No new robot motion | Geometry uses nominal CAD alignment and recorded joints')
        im.paste(self.old.rgb(self.old.scan_index).resize((544,408)),(28,155))
        text(im,(610,160),'The camera keeps sending new views.',25,True)
        text(im,(610,211),'Saving every small change immediately',24)
        text(im,(610,248),'can make disk writes fall behind.',24)
        text(im,(610,327),'We collect changes in small regions,',24,True)
        text(im,(610,366),'combine repeated updates, and let',24)
        text(im,(610,405),'a separate worker save them to disk.',24)
        text(im,(610,487),'But a fast response is not enough:',24,True)
        text(im,(610,525),'we must also check what was saved.',24)
        text(im,(28,615),'Same 854 depth inputs, same poses, same 1 cm cells. Only archive storage changes.',22,True)
        return im

    def mechanism(self):
        im=page('Two jobs, with different deadlines',
            'The recent map and the long-term archive must not share one blocking disk-write path.',
            'Archive admission is not a durability acknowledgement. Unsaved updates can be lost on a crash.')
        d=ImageDraw.Draw(im)
        for x,title,lines in ((28,'1. Receive',('Read observed surfaces.','Do not invent new links.')),
                              (452,'2. Combine',('Keep the latest value','for each repeated cell.')),
                              (876,'3. Save',('Write changed regions','in a background worker.'))):
            d.rounded_rectangle((x,170,x+376,375),radius=13,fill='white',outline='#ccd9e0',width=2)
            text(im,(x+20,193),title,27,True,color=TEAL)
            for i,line in enumerate(lines):text(im,(x+20,253+39*i),line,23)
        for x in (412,836):text(im,(x,257),'>',31,True)
        text(im,(28,438),'The waiting queue has a fixed limit.',27,True)
        text(im,(28,486),'If it fills up, a whole history frame is rejected and counted.',24)
        text(im,(28,533),'A graph is called saved only after its transaction commits.',24)
        text(im,(28,610),'This test exercises the history path (up to 5 Hz), not the full 30 Hz camera pipeline.',22,color=GRAY)
        return im

    def context(self,i):
        if i not in self.image_cache:
            a=self.old.snapshot(i);depth=a['depth'].astype(float)*self.old.meta['depth_scale']
            heat=cv2.applyColorMap((np.clip(depth/1.5,0,1)*255).astype('u1'),cv2.COLORMAP_VIRIDIS)
            heat[(depth<=0)|~np.isfinite(depth)]=(195,201,205)
            self.image_cache={i:(self.old.rgb(i).resize((432,324)),
                Image.fromarray(cv2.cvtColor(heat,cv2.COLOR_BGR2RGB)).resize((224,168),Image.Resampling.NEAREST))}
        return self.image_cache[i]

    def replay(self,t):
        j=bisect.bisect_right(self.times,t)-1
        state=self.trace[j]['state'] if j>=0 else dict(accepted=0,committed=0,rejected=0,queued_batches=0,inflight=0,durable_source=None)
        source_t=min(t,self.duration);i=bisect.bisect_right(self.old.times,source_t)-1
        assert i>=0 and self.old.times[i]<=source_t
        if j>=0:assert self.times[j]<=t
        im=page('Does saving keep up with the incoming views?',
            f'Replay clock {t:6.2f} s  |  RGB-D snapshot {self.old.times[i]:6.2f} s  |  Playback: 4x',
            'Recorded RGB-D is context. Counters are measured in the paced archive replay; no final map is shown early.')
        rgb,depth=self.context(i)
        text(im,(28,135),'Recorded camera view',22,True);im.paste(rgb,(28,170))
        im.paste(depth,(28,505))
        text(im,(270,522),'Depth Z (m)',19,True)
        grad=cv2.applyColorMap(np.tile(np.linspace(0,255,172,dtype='u1'),(10,1)),cv2.COLORMAP_VIRIDIS)
        im.paste(Image.fromarray(cv2.cvtColor(grad,cv2.COLOR_BGR2RGB)),(270,562))
        text(im,(270,580),'0',16);text(im,(336,580),'0.75',16);text(im,(402,580),'>=1.5',16)
        text(im,(270,622),'Gray: no depth',16,color=GRAY)
        text(im,(500,139),'Accepted and saved are different',24,True)
        accepted=int(state['accepted']);committed=int(state['committed']);rejected=int(state['rejected'])
        for y,label,n,color in ((202,'Accepted',accepted,INK),(263,'Saved',committed,TEAL),(324,'Rejected',rejected,ORANGE)):
            text(im,(500,y),label,23,True,color=color)
            ImageDraw.Draw(im).rounded_rectangle((670,y+5,1217,y+24),radius=5,fill='#dce5e9')
            if n:ImageDraw.Draw(im).rounded_rectangle((670,y+5,670+547*n/854,y+24),radius=5,fill=color)
            text(im,(1176,y-27),str(n),22,True,color=color)
        pending=accepted-committed
        text(im,(500,410),f'{pending} accepted frames are not yet saved.',24,True)
        text(im,(500,453),f"Waiting batches: {int(state['queued_batches'])} / 2   |   Writing now: {int(state['inflight'])}",22)
        durable=state['durable_source']
        label='No transaction has committed yet.' if durable is None else f'Latest saved source time: {durable-self.start:.2f} s'
        if durable is not None:assert durable-self.start<=source_t+1e-8
        text(im,(500,505),label,24,color=TEAL)
        text(im,(500,566),'These are history-frame counts, not map accuracy.',22,color=GRAY)
        text(im,(500,610),'A short queue is useful only if updates are retained.',22,color=GRAY)
        self.audit.append(dict(replay_wall_s=t,rgbd_source_s=self.old.times[i],status_wall_s=self.times[j] if j>=0 else None,
            durable_source_s=durable-self.start if durable is not None else None))
        return im

    def final_map(self):
        r=self.result;g=self.graph
        im=page('After the final save: the same surface graph',
            'Final archive only, after draining the writer | Three views of one stored graph',
            'Drawings sample points and links. Retaining old observations does not prove geometric accuracy or free space.')
        pose=self.old.snapshot(len(self.old.records)-1)['pose']
        for x,view,title in ((28,'oblique','Oblique'),(448,'top','Top'),(868,'side','Side')):
            text(im,(x,150),title,24,True);im.paste(panel(g['points'],g['edges'],pose,view,(384,345)),(x,197))
        text(im,(28,563),f"{r['final_nodes']:,} cells and {r['final_edges']:,} observed links.",25,True)
        text(im,(28,611),'Checked against the persistent RAM reference; stored geometry and link keys match.',22)
        height_key(im,(1090,556));return im

    def drained(self):
        r=self.result
        im=page('The camera input ends. The writer finishes its queue.',
            'A stop must drain pending work before the archive can be called complete.',
            'The next scene is a final offline export, not a map available during an earlier camera frame.')
        text(im,(55,179),f"Last input: {r['source_duration_s']:.2f} s of recorded history",30,True)
        text(im,(55,259),f"Final drain: {r['drain_ms']/1000:.2f} s after the last submission",30,True,color=ORANGE)
        text(im,(55,347),f"After that: {int(r['state']['committed'])} of {r['attempted']} frames are saved",30,True,color=TEAL)
        text(im,(55,460),'We close the file, reopen it, and compare the recovered graph.',25)
        text(im,(55,514),'No points, links, or observation times may change.',25)
        text(im,(55,610),'Keeping the input queue moving does not make disk persistence instantaneous.',23,color=GRAY)
        return im

    def results(self):
        r=self.result;probe=json.loads((self.root/'paced_1s_probe/result.json').read_text())
        repeat=json.loads((self.root/'paced_r2/result.json').read_text())
        im=page('The first attempt failed. We kept that result.',
            'One recording; component-level replay. This is not a 30 Hz end-to-end realtime benchmark.',
            'Next: repeat on other recordings, measure regional read-back, and test changed objects before live integration.')
        text(im,(28,143),'16 cm blocks: the queue filled up.',26,True,color=ORANGE)
        text(im,(28,189),f"{probe['rejected']} / 854 frames rejected. Smaller blocks alone still rejected 59 frames.",23)
        text(im,(28,257),f"Selected run: {self.proto['region_width_m']*100:.0f} cm blocks, {self.proto['interval_s']:g} s coalescing window",26,True)
        entries=[('History frames saved',f"854 / 854, both runs"),
                 ('Front-end call, p95',f"{r['frontend_call_ms']['p95']:.1f} / {repeat['frontend_call_ms']['p95']:.1f} ms"),
                 ('Oldest update to disk commit, p95',f"{r['oldest_accept_to_commit_ms']['p95']/1000:.2f} / {repeat['oldest_accept_to_commit_ms']['p95']/1000:.2f} s"),
                 ('Archive size after close',f"{r['database_bytes']/1024**2:.1f} MiB")]
        for i,(a,b) in enumerate(entries):
            text(im,(28,315+i*57),a,23);text(im,(850,315+i*57),b,25,True,color=TEAL)
        text(im,(28,571),'A longer merge window kept every input, but disk saving still takes seconds.',23,True)
        text(im,(28,621),'Two runs on this recording; settings were chosen here. Old surfaces can still be stale.',21,color=GRAY)
        return im


def main(root,run,out):
    out.mkdir(parents=True,exist_ok=False);movie=Movie(root,run);fps=30;count=0;timeline=[]
    scenes=[('intro',8.,movie.intro),('mechanism',9.,movie.mechanism),('paced_replay',movie.duration/4,None),
            ('final_drain',7.,movie.drained),('final_map',9.,movie.final_map),('results',14.,movie.results)]
    ff=ROOT/'tmp/video_tools/ffmpeg.exe';fp=ROOT/'tmp/video_tools/ffprobe.exe'
    proc=subprocess.Popen([str(ff),'-hide_banner','-loglevel','error','-n','-f','rawvideo','-pix_fmt','rgb24',
        '-s','1280x720','-r',str(fps),'-i','-','-an','-c:v','libx264','-crf','23','-preset','fast','-threads','4',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(out/'demo.mp4')],stdin=subprocess.PIPE)
    try:
        for name,duration,fn in scenes:
            start=count;still=fn() if fn else None
            for i in range(int(np.ceil(duration*fps))):
                im=still if still is not None else movie.replay(min(i/fps*4,movie.duration))
                proc.stdin.write(im.tobytes());count+=1
            (still if still is not None else movie.replay(150.)).save(out/(name+'.png'))
            timeline.append(dict(scene=name,start_s=start/fps,end_s=count/fps));print('VIDEO',name,flush=True)
        proc.stdin.close()
        if proc.wait(timeout=30):raise RuntimeError('Encoder failed')
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
    write(out/'storyboard.json',dict(fps=fps,frames=count,seconds=count/fps,timeline=timeline,run=run,playback_rate=4,
        hardware_access=False,final_map_only_after_flush=True,full_graph_export_excluded_from_timing=True))
    write(out/'causality.json',movie.audit)
    subprocess.run([str(ff),'-v','error','-xerror','-i',str(out/'demo.mp4'),'-f','null','-'],check=True)
    probe=json.loads(subprocess.check_output([str(fp),'-v','error','-count_frames','-show_entries',
        'stream=width,height,nb_read_frames,r_frame_rate:format=duration,size','-of','json',str(out/'demo.mp4')]))
    s=probe['streams'][0];assert (s['width'],s['height'],int(s['nb_read_frames']))==(1280,720,count)
    qa=out/'encoded_qa';qa.mkdir()
    for scene in timeline:
        midpoint=(scene['start_s']+scene['end_s'])/2
        subprocess.run([str(ff),'-v','error','-i',str(out/'demo.mp4'),'-ss',str(midpoint),'-frames:v','1',
            '-threads','1',str(qa/(scene['scene']+'.png'))],check=True)
    write(out/'verification.json',dict(ffprobe=probe,full_ffmpeg_decode_passed=True,chronology_assertions_passed=True,
        hardware_access=False,bytes=(out/'demo.mp4').stat().st_size))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--run',default='paced_r1');a=p.parse_args();main(a.root.resolve(),a.run,a.output.resolve())
