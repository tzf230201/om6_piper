"""A causal baseline replay followed by final-map storage comparisons.

The new archive is shown only at the final recorded time, never over early RGB.
"""
import argparse
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np
from PIL import Image
from archive_retention_study import REVIEW, ROOT
from render_live_scan_review import Review, page, text, panel, height_key, TEAL, GRAY


class Movie:
    def __init__(self,root):
        self.root=root;self.result=json.loads((root/'comparison.json').read_text());self.old=Review(REVIEW)
        self.final={m:dict(np.load(root/m/'final.npz')) for m in ('bounded','large_ttl','large_persistent','disk')}
        self.record=self.old.records[-1];self.pose=self.old.snapshot(len(self.old.records)-1)['pose']
        self.bounds=np.vstack((self.final['bounded']['points'],self.final['disk']['points'],self.pose[:3,3]))

    def intro(self):
        im=page('Why does the table disappear from the map?',
            'A storage experiment using the same real robot recording',
            'Offline research comparison | No robot motion or new camera data in this experiment')
        i=self.old.scan_index;im.paste(self.old.rgb(i).resize((544,408)),(28,155))
        text(im,(617,165),'The camera scans the table,',28,True)
        text(im,(617,207),'then turns back toward the monitor.',25)
        text(im,(617,284),'The old history buffer fills up.',25,True)
        text(im,(617,325),'New views push older table cells out.',23)
        text(im,(617,399),'We test saving past observations',25,True)
        text(im,(617,440),'to disk, outside the active buffer.',25)
        text(im,(28,612),'Same depth frames, same CAD pose, same 1 cm cells. Only storage changes.',23,True)
        return im

    def final_compare(self,slice_only=False):
        title='After returning: does the table remain?' if slice_only else 'The final map with two storage policies'
        im=page(title,
            'Both maps at 192.05 s | Same scale | Last-observed geometry, not a current obstacle map',
            'Full-map export is offline. Keeping more samples does not prove a more accurate map.')
        for x,mode,title in ((28,'bounded','Old: 40,000-node history'),(664,'disk','New: observations saved to disk')):
            a=self.final[mode];p,e=a['points'],a['edges']
            if slice_only:
                valid=(p[:,0]>=.25)&(p[:,0]<=.85)&(abs(p[:,1])<=.45)&(abs(p[:,2])<=.05)
                mapping=np.full(len(p),-1,dtype=int);mapping[valid]=np.arange(valid.sum())
                e=mapping[e[np.all(valid[e],axis=1)]];p=p[valid]
                bounds=np.array([[.25,-.45,-.05],[.85,.45,.05]])
            else:bounds=self.bounds
            text(im,(x,144),title,24,True)
            im.paste(panel(p,e,None if slice_only else self.pose,'top' if slice_only else 'oblique',(588,360),bounds),(x,190))
            score=self.result['retention'][mode]['table_roi_retained_within20mm']*100
            text(im,(x,568),f'{score:.1f}% of reference table samples found nearby',21,True,color=TEAL)
        height_key(im,(1090,137))
        if slice_only:
            text(im,(28,620),'Table-height slice: X 0.25-0.85 m, Y +/-0.45 m, Z +/-0.05 m. Not semantic segmentation.',20,color=GRAY)
        else:text(im,(28,620),'Grid and colour use robot-base coordinates. Some old observations may no longer be valid.',21,color=GRAY)
        return im

    def multiview(self):
        im=page('The archived final map, from three sides',
            'Frozen at the final recorded time | All views show exactly the same stored graph',
            'Links originate from depth-image adjacency. Drawing samples links; no mesh or missing surface is invented.')
        a=self.final['disk']
        for x,view,title in ((28,'oblique','Oblique'),(448,'top','Top'),(868,'side','Side')):
            text(im,(x,147),title,24,True)
            im.paste(panel(a['points'],a['edges'],self.pose,view,(384,345)),(x,190))
        text(im,(28,560),f"{len(a['points']):,} stored cells and {len(a['edges']):,} observed links.",24,True)
        text(im,(28,605),'Old regions are retained, but moved objects can leave old traces.',24,color=GRAY)
        height_key(im,(1090,557));return im

    def results(self):
        im=page('What improved, and what did not',
            '854 identical history inputs | Exploratory offline timing, not live end-to-end latency',
            'Storage prototype only | No live realtime validation, ground-truth accuracy test, or dynamic-object test')
        text(im,(32,146),'Storage',22,True);text(im,(445,146),'Table samples kept',22,True)
        text(im,(820,146),'History update p95',22,True)
        choices=[('bounded','40k RAM, 180 s TTL'),('large_ttl','Large RAM, 180 s TTL'),
                 ('large_persistent','Large RAM, no expiry'),('disk','Disk, 32 MiB page cache')]
        for i,(m,label) in enumerate(choices):
            y=203+57*i
            r=self.result['cache_ablation']['disk32']['result'] if m=='disk' else self.result['runs'][m]
            text(im,(32,y),label,22);text(im,(489,y),f"{100*self.result['retention'][m]['table_roi_retained_within20mm']:.1f}%",24,True,color=TEAL)
            text(im,(858,y),f"{r['native_ms']['p95']:.1f} ms",24)
        text(im,(32,466),'A sample is "kept" here if a final mapped point lies within 2 cm.',22)
        text(im,(32,512),'Larger RAM also preserves the table. Disk changes where the memory is paid.',22,True)
        text(im,(32,558),'Synchronous disk updates are too slow for the 30 Hz camera path.',23,True)
        text(im,(32,609),'Next: batch changed regions in a bounded background writer, then test queue delay.',22,color=GRAY)
        return im


def main(root,out):
    out.mkdir(parents=True,exist_ok=False);movie=Movie(root)
    # The first section is the verified old live history; no new archive is shown early.
    scenes=[('intro',8.,movie.intro),('old_history_replay',(movie.old.times[-1]-90)/3,None),
            ('final_comparison',9.,movie.final_compare),('table_slice',10.,lambda:movie.final_compare(True)),
            ('multiview',9.,movie.multiview),('results',15.,movie.results)]
    fps=30;count=0;timeline=[]
    cmd=[str(ROOT/'tmp/video_tools/ffmpeg.exe'),'-hide_banner','-loglevel','error','-n',
        '-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r',str(fps),'-i','-',
        '-an','-c:v','libx264','-crf','23','-preset','fast','-threads','4','-pix_fmt','yuv420p',
        '-movflags','+faststart',str(out/'demo.mp4')]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    try:
        for name,duration,fn in scenes:
            start=count;static=fn() if fn else None
            for i in range(int(np.ceil(duration*fps))):
                frame=static if static is not None else movie.old.growth(min(90+i/fps*3,movie.old.times[-1])).copy()
                if static is None:
                    # Existing source timestamps, RGB/depth and graph remain causally matched.
                    from PIL import ImageDraw
                    ImageDraw.Draw(frame).rectangle((0,681,1280,720),fill='#193548')
                    text(frame,(28,691),'OLD HISTORY ONLY | Recorded robot input at 3x speed; new archive is compared at the final time next.',16,color='white')
                proc.stdin.write(frame.tobytes());count+=1
            preview=static if static is not None else movie.old.growth(180)
            preview.save(out/(name+'.jpg'))
            timeline.append(dict(scene=name,start_s=start/fps,end_s=count/fps));print('VIDEO',name,flush=True)
        proc.stdin.close()
        if proc.wait(timeout=30):raise RuntimeError('Encoding failed')
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
    (out/'storyboard.json').write_text(json.dumps(dict(fps=fps,frames=count,seconds=count/fps,timeline=timeline,
        replay='Old baseline only at 3x; all new archive views are frozen final time 192.05 s',hardware_access=False),indent=2))
    # Decode every frame and keep encoded scene images for human visual QA.
    (out/'encoded_qa').mkdir();cap=cv2.VideoCapture(str(out/'demo.mp4'));decoded=0
    selected={int((r['start_s']+r['end_s'])*.5*fps):r['scene'] for r in timeline}
    while True:
        ok,frame=cap.read()
        if not ok:break
        if frame.shape!=(720,1280,3):raise ValueError('Video shape mismatch')
        if decoded in selected:cv2.imwrite(str(out/'encoded_qa'/(selected[decoded]+'.jpg')),frame)
        decoded+=1
    cap.release()
    if decoded!=count:raise ValueError('Decoded frame count mismatch')
    (out/'verification.json').write_text(json.dumps(dict(decoded_frames=decoded,bytes=(out/'demo.mp4').stat().st_size,
        all_frames_decoded=True,causal_old_replay=True,new_archive_only_at_final_time=True),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();main(a.root.resolve(),a.output.resolve())
