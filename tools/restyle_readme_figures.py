"""Restyle README figures from original recorded-data renders; no synthetic imagery."""
from pathlib import Path
import hashlib
import json
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'om6_piper/docs/images'
INK = '#202830'
MUTED = '#59636d'
FONT = Path('C:/Windows/Fonts')
def font(size, bold=False):
    return ImageFont.truetype(str(FONT / ('segoeuib.ttf' if bold else 'segoeui.ttf')), size)
def label(draw, xy, value, size=20, bold=False, fill=INK):
    draw.text(xy, value, font=font(size, bold), fill=fill)
def canvas(title, subtitle):
    im = Image.new('RGB', (1280,720), 'white')
    d = ImageDraw.Draw(im)
    label(d,(32,24),title,30,True)
    label(d,(32,70),subtitle,20,fill=MUTED)
    return im,d

def main():
    source = ROOT / 'output/cad_preview_20261005_v1/video_preview.png'
    original = Image.open(source).convert('RGB')
    im,d = canvas('Robot and recorded surface map', 'PiPER arm with a wrist-mounted RealSense D405 camera')
    label(d,(32,120),'Robot frame and surface graph',21,True)
    im.paste(original.crop((25,170,915,645)),(24,160))
    label(d,(944,120),'Recorded RGB frame',21,True)
    im.paste(original.crop((940,173,1255,409)),(944,160))
    for y,key,value in [(424,'Time','9.82 s'),(460,'Points','3,678'),(496,'Links','17,989')]:
        label(d,(944,y),key,19,fill=MUTED)
        label(d,(1100,y),value,19)
    label(d,(944,548),'Stationary robot and camera',18,fill=MUTED)
    label(d,(944,586),'Orange: camera origin',18,fill=MUTED)
    label(d,(944,613),'and viewing direction',18,fill=MUTED)
    d.line((32,662,1248,662),fill='#dce1e5',width=1)
    label(d,(32,678),'Nominal CAD pose; +20 mm display offset in world Z. Graph overlay; 4 cm cells.',16,fill=MUTED)
    label(d,(32,699),'Recorded observations; no measured hand-eye calibration or moving-robot validation.',15,fill=MUTED)
    im.save(ASSETS/'robot-camera-overview.png')

    original = Image.open(ROOT/'output/delta_archive_20261009_v1/video/final_map.png').convert('RGB')
    im,d = canvas('Archived surface graph', 'Three views of the final archive after all pending writes completed')
    for x,title in [(28,'Oblique'),(448,'Top'),(868,'Side')]:
        label(d,(x,129),title,21,True)
        im.paste(original.crop((x,232,x+384,542)),(x,181))
    label(d,(32,526),'283,706 cells',24,True)
    label(d,(448,526),'1,749,244 observed links',24,True)
    im.paste(original.crop((1088,552,1238,585)),(1090,529))
    label(d,(32,583),'Stored geometry and link keys match the in-memory reference.',20)
    label(d,(32,617),'Robot base frame; distances in metres.',18,fill=MUTED)
    d.line((32,662,1248,662),fill='#dce1e5',width=1)
    label(d,(32,679),'Sampled points and links from the final offline archive.',16,fill=MUTED)
    label(d,(32,702),'Storage consistency does not establish geometric accuracy or free space.',15,fill=MUTED)
    im.save(ASSETS/'archived-surface-views.png')
    path=ASSETS/'provenance.json'
    records=json.loads(path.read_text())
    for row in records[:2]:
        row['image_sha256']=hashlib.sha256((ASSETS/row['image']).read_bytes()).hexdigest().upper()
        row['kind']='Recorded-data render panels preserved; English labels and minimal layout redrawn'
        row['renderer']='om6_piper/tools/restyle_readme_figures.py'
    path.write_text(json.dumps(records,indent=2)+'\n')

if __name__ == '__main__':
    main()
