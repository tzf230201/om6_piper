"""Offline video of real live-scan admissions; never add future map geometry."""
import argparse
import bisect
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from live_mapping import graph_panel
from table_scan import ROOT

BG='#f3f6f8';INK='#193548';GRAY='#557080';TEAL='#137f70'


def text(im,xy,value,size=21,bold=False,color=INK):
    font=ImageFont.truetype('C:/Windows/Fonts/'+('arialbd.ttf' if bold else 'arial.ttf'),size)
    ImageDraw.Draw(im).text(xy,str(value),font=font,fill=color)


def page(title,subtitle,footer):
    im=Image.new('RGB',(1280,720),BG);draw=ImageDraw.Draw(im)
    draw.rectangle((0,0,1280,6),fill=TEAL)
    text(im,(28,27),title,31,True);text(im,(29,78),subtitle,19,color=GRAY)
    draw.line((28,115,1252,115),fill='#ccd9e0',width=2)
    draw.rectangle((0,681,1280,720),fill=INK)
    text(im,(28,691),footer,16,color='white')
    return im


def panel(p,e,pose,view,size,bounds=None):
    return Image.fromarray(cv2.cvtColor(graph_panel(p,e,pose,view,size,
        height_range=(-.8,1.2),bounds_points=bounds),cv2.COLOR_BGR2RGB))


def height_key(im,xy):
    x,y=xy
    grad=cv2.applyColorMap(np.tile(np.linspace(0,255,142,dtype='u1'),(7,1)),cv2.COLORMAP_VIRIDIS)
    im.paste(Image.fromarray(cv2.cvtColor(grad,cv2.COLOR_BGR2RGB)),(x,y))
    text(im,(x,y+9),'Z: -0.8 to +1.2 m',14,color=GRAY)


class Review:
    def __init__(self,root):
        self.root=Path(root);self.recon=json.loads((self.root/'reconstruction.json').read_text())
        assert all(self.recon['final_live_array_equal'].values()),'Must verify replay matches live arrays first'
        self.audit=json.loads((self.root/'live_audit.json').read_text())
        self.protocol=json.loads((self.root/'protocol.json').read_text())
        self.meta=json.loads((Path(self.protocol['source'])/'metadata.json').read_text())
        self.records=self.recon['captures'];self.times=[r['source_s'] for r in self.records]
        self.scan_index=max(i for i,r in enumerate(self.records) if r['stage']=='Return along near row')
        self.cache={};self.drawn={}

    def snapshot(self,i):
        if i not in self.cache:
            self.cache[i]=dict(np.load(self.root/'snapshots'/f'{self.records[i]["stem"]}.npz'))
            if len(self.cache)>3:self.cache.pop(next(iter(self.cache)))
        return self.cache[i]

    def rgb(self,i):
        return Image.open(self.root/'snapshots'/f'{self.records[i]["stem"]}.jpg').convert('RGB')

    def growth(self,source_time):
        i=bisect.bisect_right(self.times,source_time)-1
        assert i>=0 and self.times[i]<=source_time,'Future snapshot selected'
        if i in self.drawn:return self.drawn[i]
        r=self.records[i];a=self.snapshot(i)
        im=page('Scanning the tabletop with a wrist camera',
            f"Recorded snapshot {r['source_s']:.2f} s  |  {r['stage']}  |  Camera down {r['down_angle_deg']:.1f} deg",
            'Real robot recording, 3x playback | Graph replay follows live admissions; timing is from the original live run')
        text(im,(28,135),'What the camera saw',22,True)
        im.paste(self.rgb(i).resize((352,264)),(28,171))
        text(im,(28,450),'Depth Z (m)',20,True)
        depth=a['depth'].astype(float)*self.meta['depth_scale']
        heat=cv2.applyColorMap((np.clip(depth/1.5,0,1)*255).astype('u1'),cv2.COLORMAP_VIRIDIS)
        heat[(depth<=0)|~np.isfinite(depth)]=(195,201,205)
        im.paste(Image.fromarray(cv2.cvtColor(heat,cv2.COLOR_BGR2RGB)).resize((208,156),Image.Resampling.NEAREST),(28,481))
        grad=cv2.applyColorMap(np.tile(np.linspace(0,255,208,dtype='u1'),(8,1)),cv2.COLORMAP_VIRIDIS)
        im.paste(Image.fromarray(cv2.cvtColor(grad,cv2.COLOR_BGR2RGB)),(28,647))
        text(im,(28,659),'0',14);text(im,(112,659),'0.75',14);text(im,(185,659),'>=1.5',14)
        text(im,(253,487),'Live worker',18,True)
        text(im,(253,514),f"{r['live_worker_ms']:.1f} ms",23,color=TEAL)
        text(im,(253,552),'RGB gap',18,True)
        text(im,(253,579),f"{r['rgb_depth_gap_ms']:+.1f} ms",21)
        text(im,(253,625),'Gray: no depth',15,color=GRAY)
        text(im,(415,135),'Bounded history',22,True)
        height_key(im,(1105,133))
        im.paste(panel(a['history_points'],a['history_edges'],a['pose'],'oblique',(837,270)),(415,171))
        text(im,(415,451),f"{r['history_nodes']:,} nodes / {r['history_edges']:,} links retained; drawing samples links",17,color=GRAY)
        text(im,(415,486),'Recent surfaces: top / side',21,True)
        im.paste(panel(a['points'],a['edges'],a['pose'],'top',(408,142)),(415,520))
        im.paste(panel(a['points'],a['edges'],a['pose'],'side',(409,142)),(843,520))
        self.drawn={i:im};return im

    def intro(self):
        im=page('The camera now looks down at the table',
            'Piper + RealSense D405 | Actual robot run | Nominal camera-to-wrist transform from CAD',
            'Recorded on this PC | No new robot motion was sent while producing this review')
        text(im,(28,151),'Start: facing forward',24,True)
        im.paste(self.rgb(0).resize((528,396)),(28,195))
        near=next(i for i,r in enumerate(self.records) if r['stage']=='Scan near row')
        text(im,(654,151),'During the scan: looking at the tabletop',24,True)
        im.paste(self.rgb(near).resize((528,396)),(654,195))
        text(im,(28,622),'Two sweep rows, followed by a return to the starting pose.',24,True)
        return im

    def multiview(self):
        r=self.records[self.scan_index];a=self.snapshot(self.scan_index)
        im=page('The same recorded map, from three sides',
            f"Frozen at {r['source_s']:.2f} s, before the return motion | All views use the same retained history",
            'CAD-based robot coordinates | Surface graph, not a watertight mesh or a safe-motion model')
        for x,view,title in ((28,'oblique','Oblique'),(448,'top','Top'),(868,'side','Side')):
            text(im,(x,147),title,24,True)
            im.paste(panel(a['history_points'],a['history_edges'],a['pose'],view,(384,348)),(x,191))
        height_key(im,(1105,135))
        text(im,(28,573),'Each dot represents measured surface data. Links come from depth-image neighbours.',22,True)
        text(im,(28,623),'The graph still depends on pose timing, CAD alignment and the memory limit.',23,color=GRAY)
        return im

    def retention(self):
        first=self.records[self.scan_index];last=self.records[-1]
        a=self.snapshot(self.scan_index);b=self.snapshot(len(self.records)-1)
        common=np.vstack((a['history_points'],b['history_points'],a['pose'][:3,3],b['pose'][:3,3]))
        im=page('The scan works, but the history still forgets',
            '40,000 history slots | Same viewing scale and colour range for both snapshots',
            'This comparison exposes a limitation of the current bounded-history buffer; it is not a global archive')
        for x,r,s in ((28,first,a),(664,last,b)):
            text(im,(x,145),f"At {r['source_s']:.1f} s: {r['stage']}",21,True)
            im.paste(panel(s['history_points'],s['history_edges'],s['pose'],'oblique',(588,368),common),(x,188))
            text(im,(x,577),f"{r['history_nodes']:,} retained nodes",22,color=TEAL)
        text(im,(28,629),'Next: preserve inactive table regions while updating only the camera-visible area.',22,True)
        height_key(im,(1089,573))
        return im

    def results(self):
        a=self.audit
        im=page('What this live run actually showed',
            'Recorded performance, not replay timing | One run; no ground-truth map accuracy measurement',
            'High throughput does not guarantee a 33 ms deadline or accurate geometry')
        entries=[('Recorded duration',f"{a['source_duration_s']:.1f} s"),
            ('RGB-D frames recorded / mapped',f"{a['recorded_frames']:,} / {a['mapped_frames']:,}"),
            ('Admitted mapping rate',f"{a['admitted_mapping_hz']:.2f} frames/s"),
            ('Live worker p95 / worst',f"{a['worker_ms']['p95']:.1f} / {a['worker_ms']['max']:.1f} ms"),
            ('Worker calls longer than 33.33 ms',f"{a['worker_calls_over_33_33_ms']:,} of {a['mapped_frames']:,}"),
            ('Arrival to worker completion, p95',f"{a['arrival_to_worker_end_ms']['p95']:.1f} ms")]
        for i,(key,value) in enumerate(entries):
            y=151+65*i;text(im,(35,y),key,24);text(im,(869,y),value,25,True,color=TEAL)
        text(im,(35,567),'The last metric starts at host receipt. It excludes sensor/USB and display latency.',21,color=GRAY)
        text(im,(35,613),'Next checks: memory retention, repeated scans, and exposure-time camera pose.',23,True)
        return im


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('reconstruction',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);review=Review(args.reconstruction)
    growth_s=review.times[-1]/3
    scenes=[('intro',5.,review.intro),('growth',growth_s,None),('multiview',8.,review.multiview),
            ('retention',8.,review.retention),('results',9.,review.results)]
    fps=30;frame_count=0;timeline=[]
    cmd=[str(ROOT/'tmp/video_tools/ffmpeg.exe'),'-hide_banner','-loglevel','error','-n',
        '-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r',str(fps),'-i','-',
        '-an','-c:v','libx264','-preset','fast','-crf','22','-threads','4',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(args.output/'demo.mp4')]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    try:
        for name,duration,fn in scenes:
            count=int(np.ceil(duration*fps));start=frame_count/fps;static=fn() if fn else None
            for i in range(count):
                im=static if static is not None else review.growth(min(i/fps*3,review.times[-1]))
                proc.stdin.write(im.tobytes());frame_count+=1
            preview=static if static is not None else review.growth(95.)
            preview.save(args.output/(name+'.jpg'))
            timeline.append(dict(scene=name,start_s=start,end_s=frame_count/fps))
            print('Rendered',name,frame_count,'frames',flush=True)
        proc.stdin.close()
        if proc.wait(timeout=30):raise RuntimeError('Video encoding failed')
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
    output=dict(fps=fps,frames=frame_count,seconds=frame_count/fps,timeline=timeline,playback_rate=3,
        live_geometry_match_verified=True,geometry_reconstructed_offline=True,hardware_access=False,
        source=str(args.reconstruction.resolve()))
    (args.output/'storyboard.json').write_text(json.dumps(output,indent=2),encoding='utf8')
    print(json.dumps(output,indent=2))


if __name__=='__main__':main()
