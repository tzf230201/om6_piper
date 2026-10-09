"""Verify and summarize completed offline archive runs; no hardware."""
import argparse
import json
import shutil
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from block_archive_study import OLD, PACK, ROOT, compare, sorted_edges, write
from replay_robot_mapping import sha
from archive_retention_study import REVIEW
from workspace_paths import repo_file


def rich_parity(a,b):
    result=compare(a,b)
    def order(g):return np.lexsort(tuple(g['keys'][:,i] for i in range(3,-1,-1)))
    ia,ib=order(a),order(b)
    for key in ('normals','radii','node_witness'):
        result[key+'_exact']=bool(np.array_equal(a[key][ia],b[key][ib]))
    ea,eb=sorted_edges(a)[1],sorted_edges(b)[1]
    for key in ('edge_seen','edge_witness'):
        result[key+'_exact']=bool(np.array_equal(a[key][ea],b[key][eb]))
    if not all(result.values()):raise ValueError('Metadata/witness mismatch')
    return result


def main(root):
    names=['burst_r1','compressed_burst_r1','paced_r1','paced_r2','pressure']
    runs={name:json.loads((root/name/'result.json').read_text()) for name in names}
    reference=dict(np.load(OLD/'large_persistent/final.npz'))
    for name in names[:-1]:
        r=runs[name]
        if r['accepted']!=854 or r['rejected'] or r['state']['committed']!=854:
            raise ValueError('A full-run candidate lost input: '+name)
        compare(dict(np.load(root/name/'final.npz')),reference)
    stress=runs['pressure']
    if not stress['rejected'] or not all(stress['final_reference_parity'].values()):raise ValueError('Pressure rejection/partial-admission test missing')
    r1,r2=runs['paced_r1'],runs['paced_r2']
    if r1['dll_sha256']!=r2['dll_sha256'] or r1['input_depth_sha256']!=r2['input_depth_sha256']:
        raise ValueError('Repeat binary/input mismatch')
    source=json.loads((PACK/'protocol.json').read_text());expected=[r['number'] for r in source['records']]
    rich=dict(np.load(root/'compressed_burst_r1/final.npz'));metadata={}
    for name in ('paced_r1','paced_r2'):
        metadata[name]=rich_parity(dict(np.load(root/name/'final.npz')),rich)
        trace=json.loads((root/name/'trace.json').read_text());commits=json.loads((root/name/'commits.json').read_text())
        if [r['number'] for r in trace]!=expected:raise ValueError('Input-order mismatch')
        if any(r['state']['queued_batches']>2 or r['state']['pending_nodes']>40000 or r['state']['pending_edges']>80000 for r in trace):
            raise ValueError('Working-set bound exceeded')
        if any(r['state']['committed']>r['state']['accepted'] or
               (r['state']['durable_source'] is not None and r['state']['durable_source']>source['records'][i]['stamp']) for i,r in enumerate(trace)):
            raise ValueError('Future durability claim')
        if sum(m['frames'] for m in commits)!=854 or any(b['last_source']<=a['last_source'] for a,b in zip(commits,commits[1:])):
            raise ValueError('Batch chronology/accounting mismatch')
    scan=np.load(REVIEW/'snapshots/0248.npz')['history_points']
    roi=(scan[:,0]>=.25)&(scan[:,0]<=.85)&(abs(scan[:,1])<=.45)&(abs(scan[:,2])<=.05)
    final=dict(np.load(root/'paced_r1/final.npz'));distance=cKDTree(final['points']).query(scan[roi],workers=1)[0]
    previous=json.loads((OLD/'disk32/result.json').read_text())
    failure=json.loads((root/'paced_1s_probe/result.json').read_text())
    failure8=json.loads((root/'region8_probe/result.json').read_text())
    result=dict(runs=runs,failed_16cm_paced_probe=failure,failed_8cm_1s_paced_probe=failure8,
        previous_row_archive=previous,metadata_parity=metadata,
        reference_samples=int(roi.sum()),retention_fraction_by_radius_mm={str(mm):float(np.mean(distance<=mm/1000)) for mm in (1,2,5,10,20)},
        graph_identical_to_previous_persistent_reference=True,
        sqlite_bytes_ratio=r1['database_bytes']/previous['closed_database_bytes'],
        frontend_p95_range_ms=[min(r1['frontend_call_ms']['p95'],r2['frontend_call_ms']['p95']),max(r1['frontend_call_ms']['p95'],r2['frontend_call_ms']['p95'])],
        oldest_durability_p95_range_ms=[min(r1['oldest_accept_to_commit_ms']['p95'],r2['oldest_accept_to_commit_ms']['p95']),max(r1['oldest_accept_to_commit_ms']['p95'],r2['oldest_accept_to_commit_ms']['p95'])],
        chronology_verified=True,hardware_access=False,live_pipeline_changed=False,
        conclusion='Observe input admission and durable-write lag separately. Passing a 5 Hz history replay is not a 30 Hz end-to-end realtime or accuracy guarantee.',
        timing_note='Two sequential paced component replays on one robot recording, not a controlled real-time OS benchmark. Parameters were selected on this same recording (development data, not held-out validation). Raw-block pilot overlapped development work. Burst mode includes explicit producer backpressure; paced mode never waits for queue capacity and records whole-frame rejection.')
    evidence=root/'source_evidence';evidence.mkdir(exist_ok=True);hashes={}
    for name in ('block_archive_study.py','test_block_archive.py','summarize_block_archive.py','render_block_archive.py'):
        f=repo_file(name);dest=evidence/name
        if dest.exists() and sha(dest)!=sha(f):raise ValueError('Do not overwrite frozen analysis source: '+name)
        if not dest.exists():shutil.copy2(f,dest)
        hashes[name]=sha(dest)
    result['analysis_video_source_hashes']=hashes
    write(root/'summary.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('runs','previous_row_archive')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);a=p.parse_args();main(a.root.resolve())
