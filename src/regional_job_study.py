"""Incremental read jobs over the same saved map; no live-device access."""
import argparse
import json
from pathlib import Path
import time
import shutil
import numpy as np
import psutil
from regional_read_study import Reader,SOURCE,ROOT,write,summary,crop,rich_parity,sha
from regional_read_job import RegionalJob
from workspace_paths import repo_file


def run(root,name):
    out=root/name;out.mkdir();(out/'snapshots').mkdir();(out/'source').mkdir()
    hashes={}
    for f in ('regional_job_study.py','regional_read_job.py','regional_read_study.py','test_regional_reader.py'):
        src=repo_file(f);shutil.copy2(src,out/'source'/f);hashes[f]=sha(src)
    queries=json.loads((root/'requests.json').read_text());trace=[];reader=Reader(SOURCE/'archive.sqlite')
    process=psutil.Process();before=process.memory_info()
    try:
        for width in (2,4):
            for i,q in enumerate(queries[str(width)]):
                job=RegionalJob(reader,q['lower'],q['upper']);begin=time.perf_counter();g=None;error=None;peak=before.rss
                try:
                    while not job.ready_to_assemble:
                        job.step();peak=max(peak,process.memory_info().rss)
                    g=job.assemble();peak=max(peak,process.memory_info().rss)
                except RuntimeError as exc:error=str(exc)
                elapsed=(time.perf_counter()-begin)*1000
                row=dict(width=width,index=i,**q,error=error,complete=job.complete,steps=job.steps,total_ms=elapsed,
                    assembly_ms=job.assembly_ms,restored_boundary_edges=getattr(job,'restored_boundary_edges',0),
                    nodes=len(g['points']) if g else 0,edges=len(g['edges']) if g else 0,
                    retained_array_bytes=job.retained_bytes,peak_sampled_rss_increment_bytes=peak-before.rss,
                    revision=job.revision)
                trace.append(row)
                if g is not None:np.savez_compressed(out/'snapshots'/f'{width}_{i:03d}.npz',**g)
            print('JOBS',width,'complete',sum(t['complete'] for t in trace if t['width']==width),'/48',flush=True)
    finally:reader.close()
    write(out/'measurements.json',dict(trace=trace,hardware_access=False,source_hashes=hashes,
        budgets=dict(per_step_raw_kib=8192,per_step_compressed_kib=8192,regions_per_step=1,regions_per_job=64,
                     retained_nodes=40000,retained_owned_edges=200000,retained_array_bytes=32*1024**2),
        note='Native C++ read steps; Python job orchestration and final assembly prototype. No artificial pacing. Array limits are not total RSS limits; RSS excludes OS caches and includes serialization residue.'))


def analyze(root,name):
    m=json.loads((root/name/'measurements.json').read_text());ref=dict(np.load(SOURCE/'final.npz'));groups={};checked=0
    for width in (2,4):
        rows=[t for t in m['trace'] if t['width']==width]
        for row in rows:
            if not row['complete']:continue
            g=dict(np.load(root/name/'snapshots'/f"{width}_{row['index']:03d}.npz"))
            rich_parity(g,crop(ref,row['lower'],row['upper']));checked+=1
            if len(g['points'])>40000 or len(g['edges'])>200000:raise ValueError('Job output budget exceeded')
            if any(step['stats']['durable_batch']!=row['revision'][0] for step in row['steps']):raise ValueError('Mixed job revisions')
        complete=[t for t in rows if t['complete']]
        groups[str(width)]=dict(complete=len(complete),failed=len(rows)-len(complete),
            step_ms=summary([s['step_ms'] for t in rows for s in t['steps']]),
            complete_job_ms=summary([t['total_ms'] for t in complete]),assembly_ms=summary([t['assembly_ms'] for t in complete]),
            max_sampled_rss_increment_bytes=max(t['peak_sampled_rss_increment_bytes'] for t in rows),
            max_retained_array_bytes=max(t['retained_array_bytes'] for t in rows),
            failures=[dict(index=t['index'],error=t['error']) for t in rows if not t['complete']])
    proto=json.loads((root/'protocol.json').read_text())
    if sha(SOURCE/'archive.sqlite')!=proto['archive_sha256']:raise ValueError('Source archive changed')
    write(root/name/'summary.json',dict(groups=groups,exact_complete_jobs_verified=checked,hardware_access=False,
        interpretation='Small steps do not mean a complete patch is ready within one camera frame. Live commits can invalidate a job; never retry indefinitely or claim mixed-revision completeness.'))
    print(json.dumps(groups,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['run','analyze']);p.add_argument('root',type=Path)
    p.add_argument('--name',default='jobs_vectorized');a=p.parse_args()
    if Path(a.name).name!=a.name:raise ValueError('Use a single new subdirectory name')
    (run if a.command=='run' else analyze)(a.root.resolve(),a.name)
