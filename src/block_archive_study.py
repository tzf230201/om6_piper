"""Offline block-archive replay. Uses prepared files; no camera/robot commands."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import shutil
import sqlite3
import time
import numpy as np
import psutil
from archive_retention_study import ROOT, DP, IP, LP, Study, canonical
from replay_robot_mapping import sha, summary
from workspace_paths import repo_file

OLD=ROOT/'output/archive_retention_20261008_v1'
PACK=OLD/'pack'
DLL=ROOT/'cpp_surface/build_block_archive_20261009_v3/Release/block_archive_api.dll'
DELTA_DLL=ROOT/'cpp_surface/build_delta_archive_20261009_v2/Release/block_archive_api.dll'
STATUS=('accepted','committed','queued_batches','pending_nodes','pending_edges','inflight','rejected',
        'peak_pending_payload','peak_waiting_payload','raw_node_updates','raw_edge_updates',
        'coalesced_nodes','coalesced_edges','sqlite_cache_bytes','metrics_dropped','failed','durable_source')
METRICS=('batch','frames','first_source','last_source','nodes','edges','regions','write_ms',
         'oldest_accept_to_commit_ms','newest_accept_to_commit_ms','queue_wait_ms','last_id',
         'group_ms','read_ms','decode_ms','merge_ms','encode_ms','sql_ms','commit_ms','read_bytes','raw_bytes','written_bytes')
DELTA_METRICS=METRICS+('compacted_regions','compacted_raw_bytes','max_tail','delta_segments','compact_ms','delta_bytes','checkpoint_bytes')


class Block:
    def __init__(self,intr,path,interval=1.,delay_ms=0,max_mib=1024,region_cells=8,dll=DLL,delta_config=None):
        self.lib=ct.CDLL(str(dll));self.lib.block_error.restype=ct.c_char_p
        self.delta_config=delta_config
        self.lib.block_create_config.argtypes=[ct.c_int]*2+[ct.c_double]*4+[ct.c_char_p,ct.c_double,ct.c_int,ct.c_int,ct.c_int]
        self.lib.block_create_config.restype=ct.c_void_p
        self.lib.block_update.argtypes=[ct.c_void_p,DP,ct.c_int,ct.c_int64,ct.c_double,DP,ct.c_int,DP]
        self.lib.block_flush.argtypes=[ct.c_void_p];self.lib.block_destroy.argtypes=[ct.c_void_p]
        self.lib.block_status.argtypes=[ct.c_void_p,DP];self.lib.block_export.argtypes=[ct.c_void_p,DP]
        self.lib.block_metrics_profile.argtypes=[ct.c_void_p,DP,ct.c_int]
        self.lib.block_copy.argtypes=[ct.c_void_p,DP,LP,LP,ct.c_int,IP,DP,LP,ct.c_int]
        self.shape=(intr['height'],intr['width'])
        args=[intr['width'],intr['height'],*[intr[k] for k in ('fx','fy','cx','cy')],str(path).encode(),interval,delay_ms,max_mib]
        if delta_config is not None:
            if region_cells!=8:raise ValueError('Delta study fixes 8-cell regions')
            self.lib.block_create_delta.argtypes=[ct.c_int]*2+[ct.c_double]*4+[ct.c_char_p,ct.c_double]+[ct.c_int]*5
            self.lib.block_create_delta.restype=ct.c_void_p;self.lib.block_metrics_delta.argtypes=[ct.c_void_p,DP,ct.c_int]
            self.h=self.lib.block_create_delta(*args,*[delta_config[k] for k in ('regions','kib','tail')])
        else:self.h=self.lib.block_create_config(*args,region_cells)
        if not self.h:raise RuntimeError(self.lib.block_error().decode())

    def check(self,code):
        if code<0:raise RuntimeError(self.lib.block_error().decode())
        return code

    def update(self,depth,pose,frame,stamp,blocking=False):
        d=np.ascontiguousarray(depth,dtype='float64');p=np.ascontiguousarray(pose,dtype='float64');s=np.empty(3)
        if d.shape!=self.shape or p.shape!=(4,4):raise ValueError('Depth/pose dimensions invalid')
        code=self.check(self.lib.block_update(self.h,d.ctypes.data_as(DP),d.size,frame,stamp,
              p.ctypes.data_as(DP),int(blocking),s.ctypes.data_as(DP)))
        return code==0,s

    def status(self):
        s=np.empty(17);self.check(self.lib.block_status(self.h,s.ctypes.data_as(DP)))
        return {k:float(v) if np.isfinite(v) else None for k,v in zip(STATUS,s)}

    def metrics(self):
        names=DELTA_METRICS if self.delta_config is not None else METRICS
        fn=self.lib.block_metrics_delta if self.delta_config is not None else self.lib.block_metrics_profile
        a=np.empty((4096,len(names)));n=self.check(fn(self.h,a.ctypes.data_as(DP),4096))
        return [dict(zip(names,row.tolist())) for row in a[:n]]

    def flush(self):self.check(self.lib.block_flush(self.h))

    def snapshot(self):
        s=np.empty(2);self.check(self.lib.block_export(self.h,s.ctypes.data_as(DP)));n,e=map(int,s)
        values=np.empty((n,8));keys=np.empty((n,4),dtype='int64');nw=np.empty((n,3),dtype='int64')
        edges=np.empty((e,2),dtype='int32');seen=np.empty(e);ew=np.empty((e,5),dtype='int64')
        self.check(self.lib.block_copy(self.h,values.ctypes.data_as(DP),keys.ctypes.data_as(LP),nw.ctypes.data_as(LP),n,
            edges.ctypes.data_as(IP),seen.ctypes.data_as(DP),ew.ctypes.data_as(LP),e))
        if not np.isfinite(values).all() or (e and (edges.min()<0 or edges.max()>=n)):
            raise ValueError('Invalid exported graph')
        return dict(points=values[:,:3],normals=values[:,3:6],radii=values[:,6],seen=values[:,7],
                    keys=keys,node_witness=nw,edges=edges,edge_seen=seen,edge_witness=ew)

    def close(self):
        if self.h:self.lib.block_destroy(self.h);self.h=None


def write(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False),encoding='utf8')


def sorted_edges(g):
    keys=g['keys'][g['edges']].reshape(-1,8)
    order=np.lexsort(tuple(keys[:,i] for i in range(7,-1,-1)))
    return keys[order],order


def compare(actual,expected):
    ka,pa=canonical(actual);kb,pb=canonical(expected)
    geometry=np.array_equal(ka,kb) and np.array_equal(pa,pb)
    ea,_=sorted_edges(actual);eb,_=sorted_edges(expected)
    edge_parity=np.array_equal(ea,eb)
    def order(g):return np.lexsort(tuple(g['keys'][:,i] for i in range(3,-1,-1)))
    seen=np.array_equal(actual['seen'][order(actual)],expected['seen'][order(expected)])
    if not(geometry and edge_parity and seen):raise ValueError('Archive lost/changed geometry, topology or node observation time')
    return dict(geometry_keys_and_points_exact=bool(geometry),edge_keys_exact=bool(edge_parity),node_seen_exact=bool(seen))


def freeze(out):
    dest=out/'source';dest.mkdir()
    files=[Path(__file__),ROOT/'cpp_surface/include/block_archive.hpp',ROOT/'cpp_surface/src/block_archive.cpp',
           ROOT/'cpp_surface/src/block_archive_api.cpp',ROOT/'cpp_surface/tests/block_archive_tests.cpp',ROOT/'cpp_surface/CMakeLists.txt',
           repo_file('summarize_block_archive.py'),repo_file('test_delta_archive.py')]
    for f in files:shutil.copy2(f,dest/f.name)
    return {str(f.relative_to(ROOT)):sha(f) for f in files}


def run(out,mode,interval,delay,limit,region_cells=8,dll=DLL,delta_config=None,cycles=1):
    out.mkdir(parents=True,exist_ok=False);source_hashes=freeze(out)
    protocol=json.loads((PACK/'protocol.json').read_text());intr=protocol['intr'];base_records=protocol['records'][:limit or None]
    if not base_records or cycles<1:raise ValueError('Need inputs and a positive cycle count')
    span=base_records[-1]['source_s']-base_records[0]['source_s']+.25;id_span=base_records[-1]['number']+1
    records=[dict(r,stamp=r['stamp']+c*span,source_s=r['source_s']+c*span,number=r['number']+c*id_span,
                  input_index=i,cycle=c) for c in range(cycles) for i,r in enumerate(base_records)]
    if sha(PACK/'depth.u16')!=protocol['depth_sha256']:raise ValueError('Prepared inputs changed')
    write(out/'protocol.json',dict(input_protocol=str(PACK/'protocol.json'),input_protocol_sha256=sha(PACK/'protocol.json'),
        dll=str(dll),dll_sha256=sha(dll),source_hashes=source_hashes,mode=mode,interval_s=interval,writer_delay_ms=delay,
        delta_compaction=delta_config,cycles=cycles,cycle_policy='Same clip repeats with synthetic monotonic source IDs/times; not independent recordings' if cycles>1 else 'Original input',
        samples=len(records),pending_node_cap=40000,pending_edge_cap=80000,waiting_batch_cap=2,
        storage_codec='XPRESS_HUFF lossless buffer mode + FNV-1a corruption checksum (not authentication)',
        cache_target_kib=8192,region_width_m=.01*region_cells,region_record_caps=[100000,1000000],region_blob_cap_mib=128,
        hardware_access=False,claim='History storage replay only; not end-to-end live mapping',
        transient_memory_note='One incoming frame, one pending batch, up to two waiting batches, one in-flight batch, one decoded region plus serialized blob, SQLite and allocator overhead. RSS is measured, not claimed equal to payload caps.'))
    process=psutil.Process();before=process.memory_info();a=Block(intr,out/'archive.sqlite',interval,delay,region_cells=region_cells,dll=dll,delta_config=delta_config)
    trace=[];commits=[];accepted_indices=[];pixels=intr['height']*intr['width'];start=time.perf_counter()
    try:
        with (PACK/'depth.u16').open('rb') as f:
            for i,r in enumerate(records):
                if r['input_index']==0:f.seek(0)
                raw=f.read(pixels*2)
                if len(raw)!=pixels*2:raise ValueError('Truncated input')
                depth=np.frombuffer(raw,dtype='<u2').reshape(intr['height'],intr['width']).astype(float)*protocol['depth_scale']
                target=start+r['source_s']-records[0]['source_s']
                if mode=='paced':
                    remaining=target-time.perf_counter()
                    if remaining>0:time.sleep(remaining)
                begin=time.perf_counter();accepted,stats=a.update(depth,r['pose'],r['number'],r['stamp'],blocking=mode=='burst')
                call_ms=(time.perf_counter()-begin)*1000
                if accepted:accepted_indices.append(i)
                state=a.status();mem=process.memory_info();commits+=a.metrics()
                trace.append(dict(index=i,number=r['number'],source_s=r['source_s'],wall_s=begin-start,
                    status_wall_s=time.perf_counter()-start,accepted=accepted,
                    schedule_lateness_ms=max(0,(begin-target)*1000) if mode=='paced' else None,
                    extract_ms=stats[0],admit_ms=stats[1],native_ms=stats[2],call_ms=call_ms,state=state,
                    rss=mem.rss,private=mem.private,disk_bytes=sum(p.stat().st_size for p in out.glob('archive.sqlite*'))))
                if i%150==0:print(mode,i,'/',len(records),'committed',state['committed'],'queued',state['queued_batches'],flush=True)
        drain=time.perf_counter();a.flush();drain_ms=(time.perf_counter()-drain)*1000
        elapsed=time.perf_counter()-start;commits+=a.metrics();state=a.status();last_mem=process.memory_info()
        after_drain_disk=sum(p.stat().st_size for p in out.glob('archive.sqlite*'))
        final=a.snapshot();np.savez_compressed(out/'final.npz',**final)
    finally:a.close()
    if state['accepted']!=len(accepted_indices) or state['committed']!=len(accepted_indices) or sum(r['frames'] for r in commits)!=len(accepted_indices):
        raise ValueError('Admission/durability accounting mismatch')
    if state['metrics_dropped'] or state['failed'] or state['queued_batches'] or state['inflight']:raise ValueError('Incomplete/failing archive')
    # Validation happens after measured processing: it cannot inflate the update-memory metric.
    if len(base_records)==len(protocol['records']) and len(accepted_indices)==len(records):
        reference=dict(np.load(ROOT/'output/block_archive_20261009_v1/paced_r1/final.npz'))
        reference['seen']=reference['seen']+(cycles-1)*span;reference['edge_seen']=reference['edge_seen']+(cycles-1)*span
        for key in ('node_witness','edge_witness'):reference[key][:,0]+=(cycles-1)*id_span
        from summarize_block_archive import rich_parity
        parity=rich_parity(final,reference)
    else:
        ref=Study(intr,'large_persistent',out/'unused')
        try:
            with (PACK/'depth.u16').open('rb') as f:
                for i in accepted_indices:
                    r=records[i];f.seek(r['input_index']*pixels*2);raw=f.read(pixels*2)
                    depth=np.frombuffer(raw,dtype='<u2').reshape(intr['height'],intr['width']).astype(float)*protocol['depth_scale']
                    ref.update(depth,r['pose'],r['number'],r['stamp'])
            parity=compare(final,ref.snapshot())
        finally:ref.close()
    reopened=Block(intr,out/'archive.sqlite',interval,region_cells=region_cells,dll=dll,delta_config=delta_config)
    from summarize_block_archive import rich_parity
    try:reopen_parity=rich_parity(reopened.snapshot(),final)
    finally:reopened.close()
    conn=sqlite3.connect('file:'+str(out/'archive.sqlite')+'?mode=ro',uri=True)
    integrity=conn.execute('PRAGMA quick_check').fetchone()[0]
    regions=conn.execute('SELECT count(*) FROM '+('regions' if delta_config is not None else 'blocks')).fetchone()[0]
    tail=dict(segments=conn.execute('SELECT count(*) FROM deltas').fetchone()[0],max_tail=conn.execute('SELECT max(tail_count) FROM regions').fetchone()[0],
              ready_regions=conn.execute('SELECT count(*) FROM regions WHERE tail_count>=8').fetchone()[0]) if delta_config is not None else None
    conn.close()
    if integrity!='ok':raise ValueError(integrity)
    result=dict(mode=mode,attempted=len(records),accepted=len(accepted_indices),rejected=len(records)-len(accepted_indices),
        source_duration_s=records[-1]['source_s']-records[0]['source_s'],processing_wall_including_drain_s=elapsed,drain_ms=drain_ms,
        frontend_native_ms=summary([r['native_ms'] for r in trace]),frontend_call_ms=summary([r['call_ms'] for r in trace]),
        frontend_over_33ms=sum(r['call_ms']>1000/30 for r in trace),
        schedule_lateness_ms=summary([r['schedule_lateness_ms'] for r in trace]) if mode=='paced' else None,
        writer_batch_ms=summary([r['write_ms'] for r in commits]),writer_total_ms=sum(r['write_ms'] for r in commits),
        writer_component_totals={k:sum(r[k] for r in commits) for k in METRICS[12:]},
        delta_metrics=dict(compacted_regions=sum(r['compacted_regions'] for r in commits),compacted_raw_bytes=sum(r['compacted_raw_bytes'] for r in commits),
            max_tail=max(r['max_tail'] for r in commits),max_compacted_raw_bytes=max(r['compacted_raw_bytes'] for r in commits),
            max_compacted_regions=max(r['compacted_regions'] for r in commits),final_tail=tail) if delta_config is not None else None,
        oldest_accept_to_commit_ms=summary([r['oldest_accept_to_commit_ms'] for r in commits]),
        newest_accept_to_commit_ms=summary([r['newest_accept_to_commit_ms'] for r in commits]),
        queue_wait_ms=summary([r['queue_wait_ms'] for r in commits]),state=state,
        peak_rss_increment_bytes=max([r['rss'] for r in trace]+[last_mem.rss])-before.rss,
        peak_private_increment_bytes=max([r['private'] for r in trace]+[last_mem.private])-before.private,
        sampled_database_plus_wal_bytes=max([r['disk_bytes'] for r in trace]+[after_drain_disk]),database_bytes=(out/'archive.sqlite').stat().st_size,
        final_nodes=len(final['points']),final_edges=len(final['edges']),regions=regions,batches=len(commits),
        final_reference_parity=parity,reopen_parity=reopen_parity,sqlite_quick_check=integrity,
        input_depth_sha256=protocol['depth_sha256'],dll_sha256=sha(dll),hardware_access=False,cycles=cycles,
        limitations=['Offline history storage path only, not 30 Hz camera-to-display latency.',
            'Submit latency and durability lag are distinct; pending data can be lost on a crash.',
            'Last-observed surfaces can be stale; no new change-evidence mechanism.',
            'Reported RSS samples include queue/writer; exclude final offline full export and OS file cache.'])
    write(out/'trace.json',trace);write(out/'commits.json',commits);write(out/'result.json',result)
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);p.add_argument('--mode',choices=['burst','paced','stress'],default='burst')
    p.add_argument('--interval',type=float,default=1.);p.add_argument('--delay-ms',type=int,default=0);p.add_argument('--limit',type=int,default=0)
    p.add_argument('--region-cells',type=int,choices=[8,16],default=8)
    p.add_argument('--delta',action='store_true');p.add_argument('--cycles',type=int,default=1)
    args=p.parse_args();run(args.output.resolve(),args.mode,args.interval,args.delay_ms,args.limit,args.region_cells,
        DELTA_DLL if args.delta else DLL,dict(regions=64,kib=32768,tail=32) if args.delta else None,args.cycles)
