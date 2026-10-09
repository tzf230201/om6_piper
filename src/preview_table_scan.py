"""Render a labelled kinematic preview; no camera or robot IO."""
import argparse
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np
from table_scan import configuration,link_poses,mesh_hulls,camera_world_pose,ROOT
from live_mapping import label


def render(plan,row,hulls,model,optical):
    im=np.full((720,1280,3),245,np.uint8)
    label(im,'Table scan: point the camera down, then sweep',(28,42),.88)
    label(im,'KINEMATIC SIMULATION | 6x preview | No robot commands sent',(28,80),.62,(35,109,156))
    label(im,f"{row['t_s']:.1f} / {plan['duration_s']:.0f} s  -  {row['stage']}",(28,117),.67)
    T=np.asarray(row['camera_pose']);poses=link_poses(model,row['q_rad'])
    down=np.rad2deg(np.arcsin(-T[2,2]))
    for xoff,view in ((28,'side'),(660,'top')):
        panel=np.full((450,592,3),255,np.uint8)
        basis=np.array([[1,0],[0,0],[0,1.]]) if view=='side' else np.array([[1,0],[0,1],[0,0.]])
        def project(points):
            v=np.asarray(points)@basis
            return np.column_stack((155+440*v[:,0],(385 if view=='side' else 270)-440*v[:,1])).astype('i4')
        for g in np.arange(-.2,1.0,.1):
            if view=='side':line=project([[g,0,-.05],[g,0,.7]])
            else:line=project([[g,-.3,0],[g,.4,0]])
            cv2.polylines(panel,[line],False,(236,238,240),1)
        table=project([[-.05,-.35,plan['table_z_m']],[.9,-.35,plan['table_z_m']],
                       [.9,.45,plan['table_z_m']],[-.05,.45,plan['table_z_m']]])
        cv2.polylines(panel,[table],True,(164,189,183),2)
        for name,points in hulls:
            world=points@poses[name][:3,:3].T+poses[name][:3,3]
            pixels=project(world)
            hull=cv2.convexHull(pixels)
            cv2.polylines(panel,[hull],True,(174,177,180),1,cv2.LINE_AA)
        chain=project([[0,0,0]]+row['chain']+[T[:3,3].tolist()])
        cv2.polylines(panel,[chain],False,(94,88,69),5,cv2.LINE_AA)
        for point in chain:cv2.circle(panel,tuple(point),5,(72,62,46),-1,cv2.LINE_AA)
        hit=row['hit'];end=np.asarray(hit) if hit is not None else T[:3,3]+.5*T[:3,2]
        ray=project([T[:3,3],end])
        cv2.arrowedLine(panel,tuple(ray[0]),tuple(ray[1]),(13,114,237),3,cv2.LINE_AA,tipLength=.06)
        cv2.circle(panel,tuple(ray[0]),7,(13,114,237),-1)
        trail=[s['hit'] for s in plan['samples'] if s['t_s']<=row['t_s'] and s['scan'] and s['hit'] is not None]
        if trail:cv2.polylines(panel,[project(trail)],False,(94,160,20),2,cv2.LINE_AA)
        label(panel,('SIDE: X / Z' if view=='side' else 'TOP: X / Y')+' (metres)',(18,30),.6)
        im[145:595,xoff:xoff+592]=panel
    label(im,f'Camera tilt: {down:.1f} degrees below horizontal',(28,628),.63)
    label(im,'Orange: optical axis  |  Green: path on assumed tabletop',(28,658),.59)
    label(im,'Table Z = %.3f m. CAD estimate; jaws, cables, obstacles and self-collisions need operator checks.'%plan['table_z_m'],(28,697),.49)
    return im


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('plan',type=Path);args=p.parse_args()
    plan=json.loads(args.plan.read_text());out=args.plan.parent
    model,assets,optical=configuration();hulls=mesh_hulls(model,assets)
    path=out/'preview.mp4'
    command=[str(ROOT/'tmp/video_tools/ffmpeg.exe'),'-hide_banner','-loglevel','error','-n',
        '-f','rawvideo','-pix_fmt','bgr24','-s','1280x720','-r','10','-i','-',
        '-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
    process=subprocess.Popen(command,stdin=subprocess.PIPE)
    times=np.array([s['t_s'] for s in plan['samples']])
    try:
        for t in np.arange(0,plan['duration_s']+.1,.6):
            row=plan['samples'][max(0,np.searchsorted(times,t,side='right')-1)]
            im=render(plan,row,hulls,model,optical);process.stdin.write(im.tobytes())
        process.stdin.close()
        if process.wait():raise RuntimeError('Preview encoding failed')
    finally:
        if process.poll() is None:process.kill();process.wait()
    near=[s for s in plan['samples'] if s['stage']=='Scan near row']
    row=near[len(near)//2]
    cv2.imwrite(str(out/'preview.jpg'),render(plan,row,hulls,model,optical))
    print(path)


if __name__=='__main__':main()
