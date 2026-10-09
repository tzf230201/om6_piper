"""Offline tabletop trajectory candidate. Importing this module never opens hardware."""
import argparse
import json
import math
from pathlib import Path
import sys
from functools import lru_cache
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from workspace_paths import WORKSPACE_ROOT as ROOT
sys.path.insert(0, str(ROOT/'piper_readonly'))
from camera_pose import camera_world_pose, transform
from robot_model import load_model

PROFILE = ROOT/'output/cad_optical_20261005_v1/fgng_config.cad_nominal.json'
DEFAULT_SOURCE = ROOT/'om6_piper/output/piper_scan_vision_slow_v1'


@lru_cache(maxsize=1)
def configuration():
    model, assets = load_model()
    optical = json.loads(PROFILE.read_text())
    optical['world_offset_m'] = [0, 0, 0]  # Physical base frame, NOT presentation lift.
    return model, assets, optical


def link_poses(model, q):
    values = dict(zip(('joint'+str(i) for i in range(1,7)), q))
    poses = {model['root']: np.eye(4)}
    for j in model['joints']:
        motion = np.eye(4)
        if j['type'] == 'revolute':
            axis = np.asarray(j['axis'], float)
            motion[:3,:3] = Rotation.from_rotvec(axis/np.linalg.norm(axis)*values[j['name']]).as_matrix()
        # Prismatic jaws shown closed; clearance below excludes movable jaws.
        poses[j['child']] = poses[j['parent']] @ transform(j['origin']) @ motion
    return poses


def validate_start(q):
    q = np.asarray(q, float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('Six finite start angles required')
    model, _, _ = configuration()
    limits = np.array([[j['limit']['lower'],j['limit']['upper']] for j in model['joints'] if j['type']=='revolute'])
    if np.any(q < limits[:,0]) or np.any(q > limits[:,1]):
        raise ValueError('Start is outside the URDF limits; no automatic clamping for this profile')
    return q


def make_segments(q0):
    """Absolute camera-down waypoints, slow quintic transitions, no IK branch jumps."""
    q0 = np.rad2deg(validate_start(q0))
    if not (-30 <= q0[0] <= 30 and np.max(np.abs(q0[1:] - [1,-1,-10,10,2])) <= 12):
        raise ValueError('This candidate starts near the previously recorded folded pose; regenerate for a different start')
    centre = q0[0]
    prep = q0 + [0,35,-45,0,0,0]
    far = np.array([centre,60,-60,0,60,0.])
    near = np.array([centre,55,-45,0,55,0.])
    left, right = np.array([-20,0,0,0,0,0.]), np.array([20,0,0,0,0,0.])
    targets = [
        ('Lift to preparation',prep,False), ('Tilt camera toward table',far,False),
        ('Enter far row',far+left,True), ('Scan far row',far+right,True),
        ('Move to near row',near+right,True), ('Scan near row',near+left,True),
        ('Return along near row',near,True), ('Retract above table',far,False),
        ('Return to preparation',prep,False), ('Return to initial',q0,False)]
    segments=[]
    previous=q0
    for name, target, scan in targets:
        # Quintic peak derivative 1.875; commanded peak <= 4 degrees/s.
        duration=max(6., float(math.ceil(np.max(np.abs(target-previous))*1.875/4)))
        segments.append(dict(name=name,duration=duration,target_deg=target.tolist(),scan=scan))
        previous=target
    return segments


def sample(q0, segments, hz=10, hold=2.):
    rows=[]; previous=np.asarray(q0,float); t=0.
    for s in segments:
        target=np.deg2rad(s['target_deg'])
        for local in np.arange(0,s['duration']+hold,1/hz):
            u=np.clip(local/s['duration'],0,1)
            blend=10*u**3-15*u**4+6*u**5
            rows.append(dict(t_s=float(t+local),q_rad=(previous+blend*(target-previous)).tolist(),
                             stage=s['name'],scan=s['scan']))
        t+=s['duration']+hold;previous=target
    rows.append(dict(t_s=t,q_rad=previous.tolist(),stage='Complete',scan=False))
    return rows


def ray_hit(pose, table_z):
    origin, direction = pose[:3,3], pose[:3,2]
    if direction[2] >= -1e-6:
        return None
    distance=(table_z-origin[2])/direction[2]
    return (origin+distance*direction).tolist() if .07 <= distance <= 1.5 else None


def mesh_hulls(model, assets):
    hulls=[]
    for link in model['links']:
        # Base contacts its support by design. Prismatic fingers need real aperture.
        if link['name'] in ('base_link','link7','link8'):continue
        for visual in link['visuals']:
            raw=assets[visual['url']].read_bytes()
            count=int.from_bytes(raw[80:84],'little')
            dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
            if len(raw)!=84+50*count:raise ValueError('Expected binary STL')
            v=np.frombuffer(raw,offset=84,count=count,dtype=dtype)['v'].reshape(-1,3).astype(float)
            v=np.unique(v,axis=0)*np.asarray(visual['scale'])
            v=v[ConvexHull(v).vertices]
            T=transform(visual['origin'])
            hulls.append((link['name'],v@T[:3,:3].T+T[:3,3]))
    return hulls


def build(q0, table_z=0., hz=10):
    if not math.isfinite(table_z) or not -.5 <= table_z <= .15:
        raise ValueError('Table Z must be finite and in [-0.5, 0.15] metres relative to base')
    model, assets, optical=configuration()
    segments=make_segments(q0);rows=sample(q0,segments,hz)
    hulls=mesh_hulls(model,assets)
    moving_limits=[j['limit'] for j in model['joints'] if j['type']=='revolute']
    minimum=float('inf');scan_min=float('inf');hits=[];angles=[]
    for row in rows:
        q=np.asarray(row['q_rad'])
        if any(not l['lower'] <= value <= l['upper'] for value,l in zip(q,moving_limits)):
            raise ValueError('Candidate violates joint limits')
        poses=link_poses(model,q)
        T=camera_world_pose(model,q,optical)
        row.update(camera_pose=T.tolist(),hit=ray_hit(T,table_z),
                   chain=[poses['link'+str(i)][:3,3].tolist() for i in range(1,7)])
        clearance=min(float(np.min(v@poses[name][2,:3]+poses[name][2,3])-table_z) for name,v in hulls)
        row['mesh_plane_clearance_m']=clearance
        minimum=min(minimum,clearance)
        if row['scan']:
            scan_min=min(scan_min,clearance)
            angles.append(float(np.rad2deg(np.arcsin(-T[2,2]))))
            if row['hit'] is None:raise ValueError('Scan optical axis does not reach assumed table')
            hits.append(row['hit'])
    # Screening only: not self-collision / obstacle / cable checking.
    if minimum < .015:raise ValueError(f'Mesh/table screen failed: {minimum*1000:.1f} mm')
    return dict(schema=1,kind='offline_table_scan_candidate',hardware_commands_sent=0,
        q0_rad=list(q0),table_z_m=table_z,table_height_measured=False,
        firmware_verified=False,hand_eye_calibrated=False,execution_validated=False,
        limitations='Assumed horizontal table; URDF mesh-plane screen excludes base and movable jaws. No obstacle, self-collision, cable or load validation.',
        world_offset_m=[0,0,0],display_offset_m=[0,0,.02],hold_s=2.,speed_percent=10,
        sampling_hz=hz,peak_command_speed_deg_s=4.,duration_s=rows[-1]['t_s'],
        minimum_screened_clearance_m=minimum,scan_minimum_screened_clearance_m=scan_min,
        scan_down_angle_deg=[min(angles),max(angles)],
        optical_axis_table_bounds_m=[np.min(hits,axis=0).tolist(),np.max(hits,axis=0).tolist()],
        segments=segments,samples=rows)


def load_for_execution(path, actual_q):
    plan=json.loads(Path(path).read_text())
    if plan.get('kind')!='offline_table_scan_candidate' or plan.get('schema')!=1:
        raise ValueError('Not a supported table scan plan')
    actual=validate_start(actual_q)
    if np.max(np.abs(np.rad2deg(actual-np.asarray(plan['q0_rad'])))) > 2:
        raise ValueError('Actual start differs from preview by >2 degrees; generate a new preview')
    # Do not trust edited targets in a plan: regenerate from measured start and rescreen.
    fresh=build(actual,plan['table_z_m'],hz=10)
    return fresh


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=DEFAULT_SOURCE)
    p.add_argument('--table-z',type=float,default=0.)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    with (args.source/'frames.jsonl').open() as f:q0=json.loads(next(f))['joints_after']['q_rad']
    plan=build(q0,args.table_z)
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'plan.json').write_text(json.dumps(plan,allow_nan=False),encoding='utf8')
    print(json.dumps({k:v for k,v in plan.items() if k not in ('segments','samples')},indent=2))


if __name__=='__main__':main()
