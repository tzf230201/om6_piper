"""Render measured offline robot graph snapshots; never interpolate geometry."""
import bisect
import json
import itertools
from pathlib import Path
import sys
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from workspace_paths import WORKSPACE_ROOT as ROOT
sys.path.insert(0, str(ROOT/'cpp_surface/tools'))
from render_multidata import graph_image, transform, rotation

BG = '#f3f6f8'
INK = '#142d40'
MUTED = '#516877'
TEAL = '#178b7a'


def text(im, xy, s, n=22, bold=False, color=INK):
    font = ImageFont.truetype('C:/Windows/Fonts/'+('arialbd.ttf' if bold else 'arial.ttf'), n)
    ImageDraw.Draw(im).text(xy, s, font=font, fill=color)


def base(title, subtitle):
    im = Image.new('RGB', (1280,720), BG)
    d = ImageDraw.Draw(im)
    d.rectangle((0,0,1280,7), fill=TEAL)
    text(im,(28,26), title, 31, True)
    text(im,(28,78), subtitle, 20, color=MUTED)
    d.line((28,116,1252,116), fill='#d5e0e7', width=2)
    d.rectangle((0,680,1280,720), fill=INK)
    text(im,(28,690),'Recorded Piper + D405  |  Offline C++ diagnostic  |  CAD pose, not measured hand-eye calibration',16,color='white')
    return im


class RobotReplay:
    def __init__(self, root):
        self.root = Path(root)
        self.slow = json.loads((self.root/'slow/result.json').read_text())
        self.original = json.loads((self.root/'original/result.json').read_text())
        self.records = self.slow['captures']
        self.times = [r['source_s'] for r in self.records]
        self.cache = {}
        self.images = {}
        self.first = np.load(self.root/'slow/snapshots/0000.npz')['pose']
        self.poses = []
        bounds = []
        growth_basis = rotation(-72,25)
        for r in self.records:
            with np.load(self.root/'slow/snapshots'/f'{r["stem"]}.npz') as a:
                self.poses.append(a['pose'])
                p = a['history_points']
                if len(p):
                    projected = transform(p, self.first, 1)
                    projected = projected @ growth_basis
                    bounds.extend([projected.min(0), projected.max(0)])
        # Full-run bounds set only a fixed view scale, never early geometry.
        bounds.extend(transform(np.asarray(self.poses)[:,:3,3], self.first, 1) @ growth_basis)
        bounds = np.asarray(bounds)
        self.extent = np.array([bounds.min(0),bounds.max(0)]) @ growth_basis.T
        self.scale = json.loads((self.root/'slow/protocol.json').read_text())
        self.depth_scale = json.loads((Path(self.scale['source'])/'metadata.json').read_text())['depth_scale']

    def snapshot(self, i):
        if i not in self.cache:
            r = self.records[i]
            self.cache[i] = dict(np.load(self.root/'slow/snapshots'/f'{r["stem"]}.npz'))
            if len(self.cache) > 3:
                self.cache.pop(next(iter(self.cache)))
        return self.cache[i]

    def growth(self, time_s):
        t = min(float(time_s), self.times[-1])
        i = bisect.bisect_right(self.times,t)-1
        assert i >= 0 and self.times[i] <= t
        if i in self.images:
            return self.images[i]
        r = self.records[i]
        a = self.snapshot(i)
        im = base('The wrist camera now moves with the arm',
                  f'Slower recorded scan, 6x playback  |  RGB-D and graph snapshot: {r["source_s"]:.2f} s')
        text(im,(28,138),'Camera view',23,True)
        rgb = Image.open(self.root/'slow/snapshots'/f'{r["stem"]}.jpg')
        im.paste(rgb.resize((320,240)),(28,173))
        text(im,(28,426),'Depth Z (m)',19,True)
        depth = a['depth'].astype(float)*self.depth_scale
        heat = cv2.cvtColor(cv2.applyColorMap((np.clip(depth/1.5,0,1)*255).astype('u1'), cv2.COLORMAP_VIRIDIS), cv2.COLOR_BGR2RGB)
        heat[(depth<=0)|~np.isfinite(depth)] = [195,201,205]
        im.paste(Image.fromarray(heat).resize((192,144),Image.Resampling.NEAREST),(28,458))
        grad = cv2.cvtColor(cv2.applyColorMap(np.linspace(0,255,290,dtype='u1')[None,:].repeat(9,axis=0),cv2.COLORMAP_VIRIDIS),cv2.COLOR_BGR2RGB)
        im.paste(Image.fromarray(grad),(28,620))
        text(im,(28,637),'0',16);text(im,(147,637),'0.75',16);text(im,(260,637),'>=1.5',16)
        text(im,(233,476),'Gray:',17,color=MUTED);text(im,(233,500),'no depth',17,color=MUTED)
        text(im,(233,540),f'RGB gap',17,color=MUTED)
        text(im,(233,565),f'{r["rgb_offset_ms"]:+.1f} ms',17,color=MUTED)
        text(im,(380,138),'Bounded surface history (not a current obstacle map)',22,True)
        p,e = a['history_points'],a['history_edges']
        im.paste(graph_image(p,e,self.poses[:i+1],self.first,1,self.extent,(872,380)),(380,174))
        text(im,(380,565),f'{len(p):,} cells / {len(e):,} links stored; up to 14,000 points / 10,000 links shown.',17,color=MUTED)
        text(im,(380,602),'Depth and recorded joint angles place each view in the robot frame.',21,True)
        text(im,(380,636),'40,000-cell cap: old data may be evicted. No fitted mesh is added.',20,color=MUTED)
        self.images = {i:im}
        return im

    def frozen(self, local):
        r = self.records[-1]
        a = self.snapshot(len(self.records)-1)
        im = base('One recorded map, three viewing angles',
                  f'Frozen history at {r["source_s"]:.2f} s  |  Same geometry, different viewing directions')
        extent = transform(np.concatenate([a['history_points'],np.asarray(self.poses)[:,:3,3]]),self.first,1)
        views = [('Oblique',-72,25),('Top',-90,90),('Side',-90,0)]
        for x,(label,az,el) in zip((28,448,868),views):
            text(im,(x,141),label,24,True)
            im.paste(graph_image(a['history_points'],a['history_edges'],self.poses,self.first,1,extent,(384,370),az,el),(x,188))
        text(im,(28,588),'These are depth-supported links, not a watertight surface or a safe-motion model.',24,True)
        text(im,(28,634),'All final retained geometry is included. Display axes are relative to the initial camera.',21,color=MUTED)
        return im

    def results(self):
        im = base('A working data path is not yet an accurate robot map',
                  'Two recordings, one per motion profile  |  Diagnostic results, not ground-truth accuracy')
        for x,label in ((35,'Measurement'),(780,'Original'),(1035,'Slower')):
            text(im,(x,159),label,24,True)
        rows = [
            ('Unique depth frames', lambda s:f'{s["unique_frames"]:,}'),
            ('Recent-graph C++ time, p95 (ms)',lambda s:f'{s["recent_native_ms"]["p95"]:.2f}'),
            ('Reprojection residual, median (mm)',lambda s:f'{s["residual_absolute_mm"]["p50"]:.1f}'),
            ('Reprojection residual, p95 (mm)',lambda s:f'{s["residual_absolute_mm"]["p95"]:.1f}'),
        ]
        for j,(label,f) in enumerate(rows):
            y=213+j*60;text(im,(35,y),label,24)
            text(im,(790,y),f(self.original),25,True,color=TEAL)
            text(im,(1040,y),f(self.slow),25,True,color=TEAL)
        text(im,(35,491),'The residual compares later depth with an earlier, transformed view.',23,True)
        text(im,(35,533),'It includes occlusion, sensor noise, mounting error and pose timing.',22,color=MUTED)
        text(im,(35,586),'C++ time excludes bag decoding, pose computation, disk writes and video.',22,color=MUTED)
        text(im,(35,630),'Next: repeated matte-target scans and exposure-time pose checks.',23,True)
        return im

    def previews(self):
        out=self.root/'previews';out.mkdir(exist_ok=True)
        self.growth(self.times[len(self.times)//2]).save(out/'growth.png')
        self.frozen(0).save(out/'multiview.png')
        self.results().save(out/'results.png')


if __name__ == '__main__':
    RobotReplay(sys.argv[1]).previews()
