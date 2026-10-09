"""Live, read-only Piper status and a kinematic 3D schematic. No motion UI."""
from __future__ import annotations
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
from pyAgxArm.utiles.mdh_kinematics import get_mdh
from read_robot import Reader, ROOT

TITLE='AgileX Piper | pyAgxArm | READ ONLY'
W,H=1280,780
FG=(223,231,234); MUTED=(156,168,177); GOLD=(82,196,242)
CYAN=(207,194,77); RED=(90,112,246); GREEN=(139,206,132)
CTRL={0:'Standby',1:'CAN control',2:'Teaching',3:'Ethernet',4:'Wi-Fi',5:'Remote',6:'Linkage teaching',7:'Offline trajectory'}
ARM={0:'Normal',1:'Emergency stop',2:'No IK solution',3:'Singularity',4:'Angle limit',5:'Joint comm error',6:'Brake not released',7:'Collision',8:'Teaching overspeed',9:'Joint abnormal',10:'Other exception',11:'Teaching record',12:'Teaching execution',13:'Teaching pause',14:'Controller temperature',15:'Resistor temperature'}

def label(img,s,x,y,color=FG,scale=.54):
    cv2.putText(img,str(s),(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)

def chain(joints):
    """Full modified-DH chain from the official SDK; meters, base XYZ."""
    T=np.eye(4); result=[T.copy()]
    for q,(d,a,alpha,offset) in zip(joints,get_mdh('piper')):
        ca,sa,ct,st=math.cos(alpha),math.sin(alpha),math.cos(q+offset),math.sin(q+offset)
        link=np.array([[ct,-st,0,a],[ca*st,ca*ct,-sa,-sa*d],
                       [sa*st,sa*ct,ca,ca*d],[0,0,0,1.]])
        T=T@link
        result.append(T.copy())
    return result

class View:
    def __init__(self):
        self.yaw=-.65; self.elevation=.35; self.zoom=1.; self.drag=None
        self.background=np.full((H,W,3),(24,21,19),np.uint8)
    def mouse(self,event,x,y,flags,_):
        if event==cv2.EVENT_LBUTTONDOWN and x<630 and 100<y<615:
            self.drag=(x,y)
        elif event==cv2.EVENT_MOUSEMOVE and self.drag:
            self.yaw+=(x-self.drag[0])*.007
            self.elevation=float(np.clip(self.elevation+(y-self.drag[1])*.007,-1.2,1.4))
            self.drag=(x,y)
        elif event==cv2.EVENT_LBUTTONUP:
            self.drag=None
        elif event==cv2.EVENT_MOUSEWHEEL:
            delta=(flags>>16)&0xffff
            if delta>=32768: delta-=65536
            self.zoom=float(np.clip(self.zoom*(1.1 if delta>0 else 1/1.1),.4,2.8))
    def project(self,points):
        p=np.asarray(points)-[0,0,.23]
        cy,sy,ce,se=np.cos(self.yaw),np.sin(self.yaw),np.cos(self.elevation),np.sin(self.elevation)
        x=cy*p[:,0]-sy*p[:,1]
        y=se*(sy*p[:,0]+cy*p[:,1])-ce*p[:,2]
        return np.clip(np.column_stack([x,y])*650*self.zoom+[310,365],-5000,5000).astype(int)
    def render(self,data,rate):
        im=self.background.copy()
        fresh=data['all_joint_feedback_fresh']
        status=data['arm_status']
        firmware=data.get('firmware') or {}
        label(im,'AGILEX  /  PIPER',20,35,CYAN,.85)
        label(im,'pyAgxArm    candleLight USB-CAN    1 Mbps',20,63,MUTED,.50)
        label(im,'READ ONLY',1040,35,GOLD,.74)
        label(im,'Feedback fresh' if fresh else 'NO FRESH JOINT DATA',850,63,GREEN if fresh else RED,.51)
        cv2.line(im,(20,83),(1260,83),(65,64,60),1)
        label(im,'Posisi lengan - skema kinematika SDK',20,114,FG,.54)
        label(im,'Drag: orbit    Scroll: zoom    V: reset view',20,139,MUTED,.44)
        if fresh:
            transforms=chain(data['joint_angles_rad'])
            points=np.array([t[:3,3] for t in transforms])
            # Draw ground grid using true base-frame coordinates.
            for t in np.arange(-.3,.31,.1):
                for ps in [[[-.3,t,0],[.3,t,0]],[[t,-.3,0],[t,.3,0]]]:
                    a,b=self.project(ps)
                    cv2.line(im,tuple(a),tuple(b),(47,46,42),1,cv2.LINE_AA)
            uv=self.project(points)
            for a,b in zip(uv[:-1],uv[1:]):
                cv2.line(im,tuple(a),tuple(b),(68,78,85),18,cv2.LINE_AA)
                cv2.line(im,tuple(a),tuple(b),CYAN,8,cv2.LINE_AA)
            groups={}
            for i,p in enumerate(points[1:],1):
                groups.setdefault(tuple(np.round(p,5)),[]).append(i)
            for p,indices in groups.items():
                coord=self.project([p])[0]
                flags=[data['drivers'][i-1] for i in indices]
                alerted=any(d and (d['foc_status_code'] & 0xBF) for d in flags)
                col=RED if alerted else GOLD
                cv2.circle(im,tuple(coord),10,col,-1,cv2.LINE_AA)
                label(im,'/'.join('J'+str(i) for i in indices),int(coord[0]+13),int(coord[1]-8),FG,.46)
            # Actual flange feedback remains separate from SDK forward kinematics.
            feedback=data.get('flange_pose_m_rad')
            if feedback:
                xy=self.project([feedback[:3]])[0]
                cv2.drawMarker(im,tuple(xy),GREEN,cv2.MARKER_DIAMOND,15,2)
                discrepancy=np.linalg.norm(points[-1]-feedback[:3])*1000
                label(im,f'FK SDK vs flange feedback: {discrepancy:.1f} mm',20,580,MUTED,.49)
            label(im,'Titik hijau: flange feedback controller',20,606,GREEN,.48)
            axes=self.project([[0,0,0],[.1,0,0],[0,.1,0],[0,0,.1]])
            for i,(name,col) in enumerate(zip('XYZ',[(85,90,240),(90,200,105),(225,160,80)]),1):
                cv2.arrowedLine(im,tuple(axes[0]),tuple(axes[i]),col,2,cv2.LINE_AA,tipLength=.16)
                label(im,name,int(axes[i,0]+5),int(axes[i,1]),col,.43)
        else:
            label(im,'Menunggu enam joint dengan data < 0.5 detik',30,350,RED,.58)
        x=670
        mode=CTRL.get(status['ctrl_mode'],str(status['ctrl_mode'])) if status else 'Unknown / stale'
        state=ARM.get(status['arm_status'],str(status['arm_status'])) if status else 'Unknown / stale'
        label(im,f'Controller: {state}    Mode: {mode}',x,113,GREEN if status and status['arm_status']==0 else RED,.53)
        label(im,f"Firmware: {firmware.get('software_version','Unknown')}   Hardware: {firmware.get('hardware_version','Unknown')}",x,140,MUTED,.46)
        label(im,'Joint    Sudut (deg)    Enable    Motor C    Volt    Flags',x,183,MUTED,.46)
        for i in range(6):
            y=221+i*43
            d=data['drivers'][i]
            angle=f"{data['joint_angles_deg'][i]:+8.3f}" if fresh else '   --   '
            enabled=d['foc_status']['driver_enable_status'] if d else None
            flags=d['foc_status_code'] if d else None
            flag_text=f'0x{flags:02X}' if flags is not None else '--'
            label(im,f'J{i+1}',x,y,CYAN,.59)
            label(im,angle,x+60,y,FG,.59)
            label(im,'ON' if enabled else ('OFF' if enabled is False else '--'),x+225,y,GOLD,.51)
            label(im,f"{d['motor_temp']:.0f}" if d else '--',x+330,y,FG,.55)
            label(im,f"{d['vol']:.1f}" if d else '--',x+414,y,FG,.55)
            label(im,flag_text,x+495,y,RED if flags is not None and flags & 0xBF else MUTED,.53)
        gs=data.get('gripper')
        if gs:
            unit='mm' if gs.get('mode')=='width' else 'deg'
            value=gs['value']*(1000 if unit=='mm' else 1)
            label(im,f"Gripper: {value:.1f} {unit}    Force: {gs['force']:.1f} N",x,505,FG,.53)
        else:
            label(im,'Gripper: unknown / stale',x,505,MUTED,.53)
        pose=data.get('flange_pose_m_rad')
        label(im,'Flange feedback (base frame)',x,545,CYAN,.52)
        if pose:
            label(im,'XYZ mm: '+ '  '.join(f'{v*1000:+.2f}' for v in pose[:3]),x,575,FG,.55)
            label(im,'RPY deg: '+ '  '.join(f'{np.rad2deg(v):+.2f}' for v in pose[3:]),x,603,FG,.55)
        cv2.line(im,(20,633),(1260,633),(65,64,60),1)
        flagged=[f'J{i+1} (0x{d["foc_status_code"]:02X})' for i,d in enumerate(data['drivers']) if d and d['foc_status_code']&0xBF]
        label(im,'Driver flags: '+(', '.join(flagged) if flagged else 'tidak ada pada feedback yang tersedia'),20,663,RED if flagged else FG,.57)
        label(im,'0x30: SDK menandai collision_status + driver_error_status; status controller ditampilkan terpisah.',20,691,MUTED,.47)
        label(im,f"RX {data['rx_frame_count']} frames  |  {rate:.0f} frames/s  |  Motor command TX: 0  |  Firmware query: {data['firmware_queries_sent']}",20,724,MUTED,.50)
        label(im,'S: simpan snapshot    Q / Esc: tutup monitor',20,756,FG,.50)
        label(im,data['captured_at'][11:19],1135,756,MUTED,.52)
        return im

def run(args):
    output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    view=View(); reader=None
    try:
        if args.from_json:
            data=json.loads(Path(args.from_json).read_text(encoding='utf-8'))
            image=view.render(data,0)
            cv2.imwrite(str(output/'preview.png'),image)
            return
        reader=Reader(firmware='default')
        reader.query_firmware()
        if not args.headless:
            cv2.namedWindow(TITLE,cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(TITLE,view.mouse)
        start=last_save=last_rate=time.monotonic()
        last_count=0; rate=0; image=None
        print('Piper read-only monitor connected. Motor command TX blocked.',flush=True)
        while not args.seconds or time.monotonic()-start<args.seconds:
            tick=time.monotonic()
            data=reader.snapshot()
            if tick-last_rate>1:
                rate=(data['rx_frame_count']-last_count)/(tick-last_rate)
                last_count=data['rx_frame_count']; last_rate=tick
            image=view.render(data,rate)
            if tick-last_save>2:
                # Atomic JSON publication for readers outside this process.
                temp=output/'live_status.tmp'
                temp.write_text(json.dumps(data,indent=2),encoding='utf-8')
                temp.replace(output/'live_status.json')
                cv2.imwrite(str(output/'latest_preview.jpg'),image)
                print(f"{data['captured_at']} fresh={data['all_joint_feedback_fresh']} rx={data['rx_frame_count']} q={data['joint_angles_deg']}",flush=True)
                last_save=tick
            if not args.headless:
                cv2.imshow(TITLE,image)
                key=cv2.waitKey(1)&0xff
                if key in [27,ord('q')] or cv2.getWindowProperty(TITLE,cv2.WND_PROP_VISIBLE)<1:
                    break
                if key==ord('v'):
                    view=View(); cv2.setMouseCallback(TITLE,view.mouse)
                elif key==ord('s'):
                    prefix=output/datetime.now().strftime('snapshot_%Y%m%d_%H%M%S_%f')
                    prefix.with_suffix('.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
                    cv2.imwrite(str(prefix.with_suffix('.png')),image)
            time.sleep(max(0,.05-(time.monotonic()-tick)))
        if image is not None:
            cv2.imwrite(str(output/'last_preview.png'),image)
            (output/'last_status.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    finally:
        if reader:
            reader.close()
        if not args.headless and not args.from_json:
            cv2.destroyAllWindows()

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--seconds',type=float,default=0)
    parser.add_argument('--from-json')
    parser.add_argument('--output',default=str(ROOT/'output'))
    args=parser.parse_args()
    if args.seconds<0 or not np.isfinite(args.seconds): parser.error('Invalid duration')
    if args.headless and not args.seconds and not args.from_json: parser.error('Use --seconds for headless mode')
    run(args)
