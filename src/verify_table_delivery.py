"""Offline verification of this tabletop candidate and paced-replay delivery."""
import hashlib
import json
from pathlib import Path
import shutil
import cv2
import numpy as np
from table_scan import ROOT
from workspace_paths import repo_file


def sha(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def main():
    plan_dir=ROOT/'output/table_scan_20261007_v1'
    replay_dir=ROOT/'output/table_live_replay_20261007_v2'
    plan=json.loads((plan_dir/'plan.json').read_text())
    report=json.loads((replay_dir/'result.json').read_text())
    updates=[json.loads(line) for line in (replay_dir/'updates.jsonl').read_text().splitlines()]
    assert len(updates)==report['counts']['processed']
    assert len(updates)+report['queue_replaced']==report['submitted']
    assert report['error'] is None and report['hardware_access'] is False
    assert all(b['depth_number']>a['depth_number'] and b['source_depth_ms']>a['source_depth_ms'] for a,b in zip(updates,updates[1:]))
    assert all(0<=r['queue_age_ms']<=150 for r in updates)
    data=np.load(replay_dir/'final.npz')
    for name in ('recent','history'):
        p,e=data[name+'_points'],data[name+'_edges']
        assert np.isfinite(p).all() and len(p)<=40000 and len(e)<=80000
        assert not len(e) or (e.min()>=0 and e.max()<len(p) and np.all(e[:,0]!=e[:,1]))
    assert 54<plan['scan_down_angle_deg'][0]<=plan['scan_down_angle_deg'][1]<61
    assert plan['minimum_screened_clearance_m']>=.015 and plan['hardware_commands_sent']==0
    videos=[]
    for path in (plan_dir/'preview.mp4',replay_dir/'demo.mp4'):
        capture=cv2.VideoCapture(str(path));count=0;fps=capture.get(cv2.CAP_PROP_FPS)
        expected=int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        while True:
            ok,frame=capture.read()
            if not ok:break
            assert frame.shape==(720,1280,3);count+=1
        capture.release();assert count==expected and count>0
        videos.append(dict(path=str(path),frames=count,fps=fps,seconds=count/fps,bytes=path.stat().st_size,sha256=sha(path)))
    sources=['trajectory_test.py','table_scan.py','preview_table_scan.py','live_mapping.py',
             'replay_live_mapping.py','vision_capture.py','replay_robot_mapping.py','test_table_live_mapping.py',
             'README.md','verify_table_delivery.py']
    snapshot=plan_dir/'source';snapshot.mkdir(exist_ok=False)
    for name in sources:shutil.copy2(repo_file(name),snapshot/name)
    native=ROOT/'cpp_surface/build_robot_replay_20261007/Release/camera_local_api.dll'
    result=dict(status='passed',hardware_motion_executed=False,table_scan_physically_acquired=False,
        table_height_assumed_m=plan['table_z_m'],preview_6x=True,videos=videos,
        processed=len(updates),replaced=report['queue_replaced'],submitted=report['submitted'],
        effective_mapping_hz=len(updates)/report['source_duration_s'],
        worker_p95_ms=report['timing_last_10000_ms']['p95'],
        worker_calls_over_33_33_ms=sum(r['worker_ms']>1000/30 for r in updates),
        timing_note='One paced replay; not an end-to-end live camera timing benchmark or a hard realtime guarantee',
        native_sha256=sha(native),sources_sha256={name:sha(snapshot/name) for name in sources})
    (plan_dir/'verification.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
