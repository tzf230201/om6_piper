"""Compare archive policies using real measured logs; reject unsupported success claims."""
import argparse
import json
import shutil
from pathlib import Path
import numpy as np
from block_archive_study import ROOT, PACK, write
from summarize_block_archive import rich_parity
from replay_robot_mapping import sha
from workspace_paths import repo_file


def main(root):
    previous=ROOT/'output/block_archive_20261009_v1'
    names=('matched_2s','fast_r1','fast_r2','repeat3','pressure')
    folders={n:('matched_2s_v2' if n=='matched_2s' else n) for n in names}
    runs={n:json.loads((root/folders[n]/'result.json').read_text()) for n in names}
    source=json.loads((PACK/'protocol.json').read_text());ids=[r['number'] for r in source['records']]
    reference=dict(np.load(previous/'paced_r1/final.npz'));parity={}
    binary_hash=runs['matched_2s']['dll_sha256']
    for name in names[:-1]:
        run=runs[name];folder=root/folders[name];proto=json.loads((folder/'protocol.json').read_text())
        trace=json.loads((folder/'trace.json').read_text());commits=json.loads((folder/'commits.json').read_text())
        cycles=run['cycles']
        if run['dll_sha256']!=binary_hash or proto['dll_sha256']!=binary_hash:raise ValueError('Candidate binary mismatch')
        if sha(Path(proto['dll']))!=binary_hash:raise ValueError('Frozen binary changed')
        expected_interval=2. if name=='matched_2s' else .5
        if proto['interval_s']!=expected_interval:raise ValueError('Wrong accumulation interval')
        if run['accepted']!=854*cycles or run['rejected'] or run['state']['committed']!=854*cycles:
            raise ValueError('A candidate lost input: '+name)
        if run['input_depth_sha256']!=source['depth_sha256']:raise ValueError('Input mismatch')
        if [t['number'] for t in trace]!=[number+c*(ids[-1]+1) for c in range(cycles) for number in ids]:raise ValueError('Input order mismatch')
        if sum(m['frames'] for m in commits)!=len(trace):raise ValueError('Commit accounting mismatch')
        if any(b['last_source']<=a['last_source'] or b['last_id']<=a['last_id'] for a,b in zip(commits,commits[1:])):
            raise ValueError('Nonchronological commits')
        if run['sqlite_quick_check']!='ok' or run['state']['failed'] or run['state']['metrics_dropped']:
            raise ValueError('Incomplete/corrupt archive or missing metrics')
        for m in commits:
            if m['compacted_regions']>64 or m['compacted_raw_bytes']>32*1024**2 or m['max_tail']>32:
                raise ValueError('Compaction work/backlog bound exceeded')
        for t in trace:
            s=t['state']
            if s['queued_batches']>2 or s['pending_nodes']>40000 or s['pending_edges']>80000 or s['committed']>s['accepted']:
                raise ValueError('Queue/admission bound exceeded')
            if s['durable_source'] is not None and s['durable_source']>source['records'][0]['stamp']+t['source_s']+1e-8:
                raise ValueError('Future timestamp')
        if cycles==1:parity[name]=rich_parity(dict(np.load(folder/'final.npz')),reference)
        else:
            if not all(run['final_reference_parity'].values()):raise ValueError('Repeated-input shifted reference failed')
            parity[name]=run['final_reference_parity']
        if not all(run['reopen_parity'].values()):raise ValueError('Reopen mismatch')
        if proto['delta_compaction']!=dict(regions=64,kib=32768,tail=32):raise ValueError('Unexpected budgets')
    if not runs['pressure']['rejected'] or not all(runs['pressure']['final_reference_parity'].values()):
        raise ValueError('Overload partial-admission check missing')
    old=json.loads((previous/'paced_r1/result.json').read_text());matched=runs['matched_2s']
    old_protocol=json.loads((previous/'paced_r1/protocol.json').read_text());new_protocol=json.loads((root/'matched_2s_v2/protocol.json').read_text())
    for key in ('input_protocol_sha256','interval_s','region_width_m','cache_target_kib','pending_node_cap','pending_edge_cap','waiting_batch_cap'):
        if old_protocol[key]!=new_protocol[key]:raise ValueError('Matched comparison setting differs: '+key)
    result=dict(baseline=old,runs=runs,folders=folders,exact_parity=parity,
        retained_cursor_pilot=json.loads((root/'matched_2s/result.json').read_text()),
        matched_ratios={
            'application_written_payload':matched['writer_component_totals']['written_bytes']/old['writer_component_totals']['written_bytes'],
            'writer_work_wall':matched['writer_total_ms']/old['writer_total_ms'],
            'closed_file_size':matched['database_bytes']/old['database_bytes']},
        scope='Development evidence on one recorded robot clip; repeat3 is the same clip three times, not new views or three independent datasets.',
        hardware_access=False,live_pipeline_changed=False,manuscript_changed=False,
        evidence_limits=['Payload bytes are not physical disk I/O.',
            'Application compaction work is bounded by raw bytes and region count, not a hard execution-time deadline.',
            'Remaining delta segments are already durable; their read-back needs checkpoint-plus-tail reconstruction.',
            'Flush drains accepted writes, not all optional compaction. Include remaining tails when comparing archive size.',
            'No pose/geometry ground truth or moved-object test was added. Full export is offline.'])
    # Preserve the initial freeze; the final storyboard explicitly labels burst pacing.
    evidence=root/'source_evidence_v2';evidence.mkdir(exist_ok=True);hashes={}
    for name in ('block_archive_study.py','test_delta_archive.py','analyze_delta_archive.py','render_delta_archive.py',
                 'render_block_archive.py','render_live_scan_review.py','summarize_block_archive.py','verify_delta_delivery.py'):
        src=repo_file(name);dest=evidence/name
        if dest.exists() and sha(dest)!=sha(src):raise ValueError('Refuse to replace frozen source: '+name)
        if not dest.exists():shutil.copy2(src,dest)
        hashes[name]=sha(dest)
    result['analysis_video_source_hashes']=hashes
    result['source_evidence_directory']=evidence.name
    write(root/'comparison.json',result)
    print(json.dumps(dict(ratios=result['matched_ratios'],runs={n:dict(accepted=r['accepted'],rejected=r['rejected'],
        frontend_p95=r['frontend_call_ms']['p95'],durable_p95=r['oldest_accept_to_commit_ms']['p95'],
        writer_p95=r['writer_batch_ms']['p95'],mb=r['database_bytes']/1024**2,delta=r['delta_metrics']) for n,r in runs.items()}),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);a=p.parse_args();main(a.root.resolve())
