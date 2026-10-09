"""Causal offline demo of bounded delta storage; all measurements come from logs."""
import argparse
import json
from pathlib import Path
import subprocess
import numpy as np
from PIL import ImageDraw
from block_archive_study import ROOT, write
from render_block_archive import Movie as PreviousMovie
from render_live_scan_review import page, text, TEAL, GRAY


class Movie(PreviousMovie):
    def __init__(self,root):
        self.comparison=json.loads((root/'comparison.json').read_text())
        super().__init__(root,'fast_r1')

    def intro(self):
        im=page('Save the changes, not the whole region',
            'An archive-storage update, tested on the same recorded Piper + D405 scan',
            'Offline replay | No robot motion | Nominal CAD poses, not an independently calibrated ground-truth map')
        im.paste(self.old.rgb(self.old.scan_index).resize((544,408)),(28,155))
        for y,line,bold in ((160,'A region already holds many surface points.',True),
                            (211,'A new view changes only some of them.',False),
                            (287,'Before, we unpacked and rewrote the region.',False),
                            (329,'That repeated old work on every save.',False),
                            (410,'Now, we first save a small change record.',True),
                            (453,'We combine these records a few at a time,',False),
                            (490,'instead of rebuilding everything at once.',False)):
            text(im,(610,y),line,23,bold)
        text(im,(28,615),'The graph must stay identical. Speed alone is not a successful result.',24,True)
        return im

    def mechanism(self):
        im=page('A checkpoint, small changes, and a limited merge job',
            'Schematic only. A change record contains observed cell/link updates and their original timestamps.',
            'History input: up to 5 Hz. This is not the complete 30 Hz camera pipeline. A failed write is reported.')
        d=ImageDraw.Draw(im)
        d.rounded_rectangle((28,175,364,376),radius=13,fill='white',outline='#ccd9e0',width=2)
        text(im,(48,203),'Saved checkpoint',26,True,color=TEAL)
        text(im,(48,260),'The region built so far.',23)
        text(im,(48,303),'We keep it on disk.',23)
        text(im,(389,257),'+',34,True)
        for i in range(3):
            y=175+i*71;d.rounded_rectangle((450,y,746,y+55),radius=8,fill='#e6f1ef',outline='#96bcb6')
            text(im,(469,y+13),f'New change {i+1}',23,True)
        text(im,(777,257),'>',34,True)
        d.rounded_rectangle((849,175,1252,376),radius=13,fill='white',outline='#ccd9e0',width=2)
        text(im,(870,203),'Merge a bounded amount',25,True,color=TEAL)
        text(im,(870,260),'Make a newer checkpoint.',23)
        text(im,(870,303),'Keep later work for later.',23)
        text(im,(28,444),'Each transaction has a merge-work budget:',26,True)
        text(im,(28,490),'at most 64 regions and 32 MiB of raw input payload.',25)
        text(im,(28,542),'A region can retain at most 32 change records. Exceeding the limit stops the write.',22)
        text(im,(28,614),'These are work limits, not a hard time deadline or a total-memory guarantee.',22,color=GRAY)
        return im

    def results(self):
        c=self.comparison;old=c['baseline'];matched=c['runs']['matched_2s'];fast=c['runs']['fast_r1']
        im=page('Less repeated work, then a shorter save window',
            'Same 854 history inputs, 8 cm blocks, 1 cm cells and queue limits | One development recording',
            'These are archive-component timings, not 30 Hz camera-to-display latency. Payload bytes are not physical disk I/O.')
        for x,label in ((28,'Measurement'),(480,'Previous / 2 s'),(745,'Changes / 2 s'),(1000,'Changes / 0.5 s')):
            text(im,(x,147),label,21,True)
        rows=[('Saved input frames',lambda r:f"{int(r['state']['committed'])} / {r['attempted']}"),
              ('Write batch, p95',lambda r:f"{r['writer_batch_ms']['p95']:.0f} ms"),
              ('Oldest update to save, p95',lambda r:f"{r['oldest_accept_to_commit_ms']['p95']/1000:.2f} s"),
              ('Payload sent to SQLite',lambda r:f"{r['writer_component_totals']['written_bytes']/1024**2:.0f} MiB"),
              ('Closed archive file',lambda r:f"{r['database_bytes']/1024**2:.1f} MiB")]
        for i,(label,fn) in enumerate(rows):
            y=213+i*66;text(im,(28,y),label,23)
            for x,r in ((480,old),(745,matched),(1000,fast)):text(im,(x,y),fn(r),23,True,color=TEAL)
        text(im,(28,550),'The unchanged 2 s window isolates storage; 0.5 s tests lower save delay.',22,True)
        text(im,(28,590),f"Submitting a frame (p95): {matched['frontend_call_ms']['p95']:.1f} ms at 2 s, versus {old['frontend_call_ms']['p95']:.1f} ms before.",22)
        repeat=c['runs']['fast_r2']
        text(im,(28,630),f"Repeat of the 0.5 s run: {repeat['accepted']}/854 saved; oldest-update delay p95 {repeat['oldest_accept_to_commit_ms']['p95']/1000:.2f} s.",22)
        return im

    def stress(self):
        c=self.comparison;r=c['runs']['repeat3'];p=c['runs']['pressure'];d=r['delta_metrics']
        im=page('Do not hide unfinished work in the background',
            'Same clip repeated 3x, sent without pacing. The sender waits for queue space; this is not live throughput.',
            'Next: region-only read-back and genuinely different recordings. This archive still cannot identify moved objects.')
        text(im,(28,163),f"Repeated-input test: {r['accepted']:,} / {r['attempted']:,} frames saved",28,True)
        text(im,(28,225),f"Largest stored tail: {int(d['max_tail'])} change records per region (limit 32)",25)
        text(im,(28,282),f"Regions merged during the run: {int(d['compacted_regions']):,}",25)
        text(im,(28,352),'We also slowed the writer on purpose and removed input pacing.',24,True)
        text(im,(28,400),f"{p['accepted']} frames accepted; {p['rejected']} rejected. Only accepted frames appear after reopening.",23)
        text(im,(28,485),'Saved change records are durable, even before they are merged.',24,True)
        text(im,(28,532),'Reading a region must still combine its checkpoint and remaining records.',23)
        text(im,(28,618),'Exact graph recovery is not proof of geometric accuracy or a current obstacle map.',22,color=GRAY)
        return im


def main(root,out):
    out.mkdir(parents=True,exist_ok=False);movie=Movie(root);fps=30;count=0;timeline=[]
    scenes=[('intro',8.,movie.intro),('mechanism',9.,movie.mechanism),('paced_replay',movie.duration/4,None),
            ('final_drain',6.,movie.drained),('final_map',8.,movie.final_map),('results',13.,movie.results),('stress',10.,movie.stress)]
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
    write(out/'storyboard.json',dict(fps=fps,frames=count,seconds=count/fps,timeline=timeline,run='fast_r1',playback_rate=4,
        hardware_access=False,final_graph_only_after_flush=True,full_export_excluded_from_timing=True))
    write(out/'causality.json',movie.audit)
    subprocess.run([str(ff),'-v','error','-xerror','-i',str(out/'demo.mp4'),'-f','null','-'],check=True)
    probe=json.loads(subprocess.check_output([str(fp),'-v','error','-count_frames','-show_entries',
        'stream=width,height,nb_read_frames,r_frame_rate:format=duration,size','-of','json',str(out/'demo.mp4')]))
    s=probe['streams'][0];assert (s['width'],s['height'],int(s['nb_read_frames']))==(1280,720,count)
    qa=out/'encoded_qa';qa.mkdir()
    for scene in timeline:
        subprocess.run([str(ff),'-v','error','-i',str(out/'demo.mp4'),'-ss',str((scene['start_s']+scene['end_s'])/2),'-frames:v','1',
            '-threads','1',str(qa/(scene['scene']+'.png'))],check=True)
    write(out/'verification.json',dict(ffprobe=probe,full_ffmpeg_decode_passed=True,chronology_assertions_passed=True,
        hardware_access=False,bytes=(out/'demo.mp4').stat().st_size))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();main(a.root.resolve(),a.output.resolve())
