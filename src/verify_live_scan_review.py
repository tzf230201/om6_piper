"""Verify decoded demo, saved graph endpoints, and source immutability offline."""
import argparse
import json
from pathlib import Path
import shutil
import cv2
import numpy as np
from scipy.spatial import cKDTree
from replay_robot_mapping import sha
from workspace_paths import repo_file


def verify(root,video_name='video'):
    recon=root/'reconstruction';video=root/video_name
    result=json.loads((recon/'reconstruction.json').read_text())
    audit=json.loads((recon/'live_audit.json').read_text())
    protocol=json.loads((recon/'protocol.json').read_text())
    captures=result['captures'];source=Path(protocol['source'])
    assert all(result['final_live_array_equal'].values())
    assert all(a['source_s']<b['source_s'] for a,b in zip(captures,captures[1:]))
    for c in captures:
        with np.load(recon/'snapshots'/f'{c["stem"]}.npz') as a:
            for prefix,pkey,ekey in [('recent','points','edges'),('history','history_points','history_edges')]:
                p,e=a[pkey],a[ekey]
                assert len(p)==c[prefix+'_nodes'] and len(e)==c[prefix+'_edges']
                assert np.isfinite(p).all() and len(p)<=40000 and len(e)<=80000
                assert not len(e) or (e.min()>=0 and e.max()<len(p) and np.all(e[:,0]!=e[:,1]))
    scan=max((c for c in captures if c['stage']=='Return along near row'),key=lambda r:r['source_s'])
    p=np.load(recon/'snapshots'/f'{scan["stem"]}.npz')['history_points']
    q=np.load(recon/'snapshots'/f'{captures[-1]["stem"]}.npz')['history_points']
    dist=cKDTree(q).query(p)[0]
    def mask(points):
        return (points[:,0]>=.25)&(points[:,0]<=.85)&(points[:,1]>=-.45)&(points[:,1]<=.45)&(abs(points[:,2])<=.05)
    roi=mask(p)
    retention=dict(scan_source_s=scan['source_s'],final_source_s=captures[-1]['source_s'],
        full_scan_nodes=len(p),full_scan_points_with_final_neighbour_within_20mm_fraction=float(np.mean(dist<=.02)),
        assumed_table_roi_m=dict(x=[.25,.85],y=[-.45,.45],z=[-.05,.05]),
        scan_points_in_roi=int(roi.sum()),final_points_in_roi=int(mask(q).sum()),
        roi_points_with_final_neighbour_within_20mm_fraction=float(np.mean(dist[roi]<=.02)),
        note='Geometric proximity diagnostic, not point identity, area coverage, semantic tabletop segmentation, ground-truth loss or a change-detection score.')
    (root/'retention_diagnostic.json').write_text(json.dumps(retention,indent=2),encoding='utf8')
    changed=[]
    for name,old in protocol['input_fingerprints'].items():
        path=source/name
        if path.stat().st_size!=old['bytes'] or sha(path)!=old['sha256']:changed.append(name)
    assert not changed,'Input hashes changed: '+str(changed)
    storyboard=json.loads((video/'storyboard.json').read_text())
    path=video/'demo.mp4';cap=cv2.VideoCapture(str(path));frames=0
    assert cap.isOpened();fps=cap.get(cv2.CAP_PROP_FPS)
    qa=video/'encoded_qa';qa.mkdir(exist_ok=False)
    targets={int(((s['start_s']+s['end_s'])/2)*fps):s['scene'] for s in storyboard['timeline']}
    while True:
        ok,frame=cap.read()
        if not ok:break
        assert frame.shape==(720,1280,3)
        if frames in targets:assert cv2.imwrite(str(qa/(targets[frames]+'.jpg')),frame)
        frames+=1
    cap.release()
    assert frames==storyboard['frames'] and fps==storyboard['fps']
    assert frames/fps<180 and path.stat().st_size<19_000_000
    sources=['review_live_scan.py','render_live_scan_review.py','test_live_scan_review.py','verify_live_scan_review.py','live_mapping.py']
    code=video/'review_source';code.mkdir(exist_ok=False)
    for name in sources:shutil.copy2(repo_file(name),code/name)
    report=dict(status='passed',hardware_access=False,source_inputs_unchanged=True,
        final_live_arrays_match=result['final_live_array_equal'],snapshot_count=len(captures),
        recorded_frames=audit['recorded_frames'],mapped_frames=audit['mapped_frames'],
        video=dict(path=str(path.resolve()),fps=fps,decoded_frames=frames,seconds=frames/fps,bytes=path.stat().st_size,sha256=sha(path)),
        code_sha256={p.name:sha(p) for p in code.iterdir()},
        caveat='Video is offline reconstruction of original live admissions, not a browser screen recording.')
    (video/'verification.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    p.add_argument('--video-dir',default='video');args=p.parse_args();verify(args.root,args.video_dir)
