"""Bounded regional archive reads. Files only; no robot/camera device access."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import time
import numpy as np
import psutil
from block_archive_study import ROOT, Block, DP, IP, LP, write
from replay_robot_mapping import sha, summary
from summarize_block_archive import rich_parity
from workspace_paths import workspace_file

BUILD=ROOT/'cpp_surface/build_region_read_20261009_v2/Release'
DLL=BUILD/'regional_read_api.dll'
WRITER_DLL=BUILD/'block_archive_api.dll'
SOURCE=ROOT/'output/delta_archive_20261009_v1/fast_r1'
STATS=('requested_regions','found_regions','raw_bytes','compressed_bytes','owned_boundary_edges_omitted',
       'read_transaction_ms','build_ms','native_ms','durable_source','durable_frames','durable_batch','sqlite_cache_bytes')
CONFIG=dict(regions=64,kib=32768,tail=32)


class Reader:
    def __init__(self,path):
        self.lib=ct.CDLL(str(DLL));self.lib.region_error.restype=ct.c_char_p
        self.lib.region_sqlite_version.restype=ct.c_char_p
        self.version=self.lib.region_sqlite_version().decode()
        self.lib.region_open.argtypes=[ct.c_char_p];self.lib.region_open.restype=ct.c_void_p
        self.lib.region_close.argtypes=[ct.c_void_p]
        self.lib.region_query.argtypes=[ct.c_void_p,LP]+[ct.c_int]*5+[DP,DP]
        self.lib.region_copy.argtypes=[ct.c_void_p,DP,LP,LP,ct.c_int,IP,DP,LP,ct.c_int]
        self.lib.region_copy_boundary.argtypes=[ct.c_void_p,LP,DP,LP,ct.c_int]
        self.h=self.lib.region_open(str(Path(path).resolve()).encode())
        if not self.h:raise RuntimeError(self.lib.region_error().decode())
        self.valid=False

    def check(self,code):
        if code<0:raise RuntimeError(self.lib.region_error().decode())

    def query(self,lower,upper,regions=64,raw_kib=8192,compressed_kib=8192,nodes=40000,edges=80000):
        self.valid=False
        bounds=np.asarray([*lower,*upper])
        if bounds.shape!=(6,) or bounds.dtype.kind not in 'iu':raise ValueError('Need two 3D integer region bounds')
        if np.any(bounds>1000000000) or (bounds.dtype.kind=='i' and np.any(bounds < -1000000000)):raise ValueError('Region coordinates out of range')
        bounds=np.ascontiguousarray(bounds,dtype='int64')
        count=np.zeros(2);stats=np.zeros(12)
        self.check(self.lib.region_query(self.h,bounds.ctypes.data_as(LP),regions,raw_kib,compressed_kib,nodes,edges,
                                        count.ctypes.data_as(DP),stats.ctypes.data_as(DP)))
        self.valid=True;graph=self.copy(*map(int,count))
        boundary=int(stats[4]);keys=np.empty((boundary,8),dtype='int64');seen=np.empty(boundary);witness=np.empty((boundary,5),dtype='int64')
        self.check(self.lib.region_copy_boundary(self.h,keys.ctypes.data_as(LP),seen.ctypes.data_as(DP),witness.ctypes.data_as(LP),boundary))
        graph.update(boundary_keys=keys,boundary_seen=seen,boundary_witness=witness)
        return graph,{k:float(v) if np.isfinite(v) else None for k,v in zip(STATS,stats)}

    def copy(self,n,e):
        if not self.valid:raise RuntimeError('No successful regional result')
        values=np.empty((n,8));keys=np.empty((n,4),dtype='int64');nw=np.empty((n,3),dtype='int64')
        edges=np.empty((e,2),dtype='int32');seen=np.empty(e);ew=np.empty((e,5),dtype='int64')
        self.check(self.lib.region_copy(self.h,values.ctypes.data_as(DP),keys.ctypes.data_as(LP),nw.ctypes.data_as(LP),n,
            edges.ctypes.data_as(IP),seen.ctypes.data_as(DP),ew.ctypes.data_as(LP),e))
        return dict(points=values[:,:3],normals=values[:,3:6],radii=values[:,6],seen=values[:,7],
                    keys=keys,node_witness=nw,edges=edges,edge_seen=seen,edge_witness=ew)

    def close(self):
        if self.h:self.lib.region_close(self.h);self.h=None


def crop(g,lower,upper):
    region=g['keys'][:,:3]//8
    keep=np.all((region>=lower)&(region<upper),axis=1);selected=np.flatnonzero(keep)
    pair_keep=keep[g['edges']].all(axis=1)
    remap=np.full(len(keep),-1,dtype='int32');remap[selected]=np.arange(len(selected))
    result={k:g[k][keep] for k in ('points','normals','radii','seen','keys','node_witness')}
    result.update(edges=remap[g['edges'][pair_keep]],edge_seen=g['edge_seen'][pair_keep],edge_witness=g['edge_witness'][pair_keep])
    return result


def prepare(out):
    out.mkdir(parents=True,exist_ok=False)
    with sqlite3.connect('file:'+str(SOURCE/'archive.sqlite')+'?mode=ro',uri=True) as db:
        ordered=db.execute('SELECT rx,ry,rz FROM regions ORDER BY base_raw+tail_raw DESC,rx,ry,rz').fetchall()
    rng=np.random.default_rng(20261009)
    # Development queries: 16 high-cost regions + 32 reproducibly sampled regions.
    centres=ordered[:16]+[ordered[i] for i in rng.choice(len(ordered),32,replace=False)]
    requests={str(w):[dict(lower=(np.asarray(c)-w//2).tolist(),upper=(np.asarray(c)-w//2+w).tolist()) for c in centres] for w in (1,2,4)}
    write(out/'requests.json',requests)
    source=out/'source';source.mkdir();hashes={}
    paths=['cpp_surface/include/block_archive.hpp','cpp_surface/include/regional_archive_reader.hpp',
        'cpp_surface/src/block_archive.cpp','cpp_surface/src/regional_read_api.cpp','cpp_surface/src/block_archive_api.cpp',
        'cpp_surface/CMakeLists.txt','om6_piper/regional_read_study.py','om6_piper/test_regional_reader.py','om6_piper/regional_read_job.py']
    for name in paths:
        f=workspace_file(name);shutil.copy2(f,source/f.name);hashes[name]=sha(f)
    write(out/'protocol.json',dict(source=str(SOURCE),archive_sha256=sha(SOURCE/'archive.sqlite'),
        source_hashes=hashes,reader_dll_sha256=sha(DLL),writer_dll_sha256=sha(WRITER_DLL),
        limits=dict(regions=64,raw_kib=8192,compressed_kib=8192,nodes=40000,owned_edges=80000),
        queries='48 per width: 16 expensive regions + 32 seeded samples. Development, not held-out validation.',
        region_box='Half-open integer regions; 8 cm each; negative coordinates use floor division.',
        graph_semantics='Exact induced subgraph: only edges with both endpoints in the selected regions.',
        cache_note='Fresh reader per child, then persistent connection. No OS cache purge; not a cold-disk benchmark.',hardware_access=False))


def child(root,width,repeat):
    out=root/f'width{width}_r{repeat}';out.mkdir();snaps=out/'snapshots';snaps.mkdir()
    requests=json.loads((root/'requests.json').read_text())[str(width)]
    process=psutil.Process();before=process.memory_info();trace=[]
    reader=Reader(SOURCE/'archive.sqlite')
    try:
        for i,q in enumerate(requests):
            begin=time.perf_counter();g=None
            try:
                g,s=reader.query(q['lower'],q['upper']);error=None
            except RuntimeError as exc:s=None;error=str(exc)
            call_ms=(time.perf_counter()-begin)*1000;mem=process.memory_info()
            trace.append(dict(index=i,**q,stats=s,error=error,call_ms=call_ms,rss=mem.rss,private=mem.private,
                              nodes=len(g['points']) if g else 0,edges=len(g['edges']) if g else 0))
            # Saving samples is outside query timing. All are validated in a separate process.
            if g is not None:np.savez_compressed(snaps/f'{i:03d}.npz',**g)
        write(out/'result.json',dict(width=width,repeat=repeat,queries=len(trace),success=sum(t['error'] is None for t in trace),
            failures=[t for t in trace if t['error']],successful_call_ms=summary([t['call_ms'] for t in trace if t['error'] is None]),
            successful_native_ms=summary([t['stats']['native_ms'] for t in trace if t['stats']]),
            peak_sampled_rss_increment_bytes=max(t['rss'] for t in trace)-before.rss,
            peak_sampled_private_increment_bytes=max(t['private'] for t in trace)-before.private,
            sqlite_version=reader.version,trace=trace,hardware_access=False,
            memory_scope='Reader, Python output arrays, allocator and prior NPZ sample serialization; excludes full-reference validation and OS file cache.'))
    finally:reader.close()
    print('READ',width,repeat,'success',sum(t['error'] is None for t in trace),'/',len(trace),flush=True)


def full_child(root):
    out=root/'full_export';out.mkdir();shutil.copy2(SOURCE/'archive.sqlite',out/'archive.sqlite')
    intr=json.loads((ROOT/'output/archive_retention_20261008_v1/pack/protocol.json').read_text())['intr']
    p=psutil.Process();before=p.memory_info();a=Block(intr,out/'archive.sqlite',dll=WRITER_DLL,delta_config=CONFIG)
    trace=[]
    try:
        for i in range(3):
            start=time.perf_counter();g=a.snapshot();duration=(time.perf_counter()-start)*1000;mem=p.memory_info()
            trace.append(dict(call_ms=duration,nodes=len(g['points']),edges=len(g['edges']),rss=mem.rss,private=mem.private))
            del g
    finally:a.close()
    write(out/'result.json',dict(trace=trace,call_ms=summary([t['call_ms'] for t in trace]),
        peak_sampled_rss_increment_bytes=max(t['rss'] for t in trace)-before.rss,
        meaning='Full graph read + copy on an isolated archive copy, three calls. Different amount of returned data; not an equal-work algorithm speed-up.'))
    print('FULL EXPORT complete',flush=True)


def analyze(root):
    reference=dict(np.load(SOURCE/'final.npz'));runs={};checked=0
    for width in (1,2,4):
        for rep in (1,2):
            name=f'width{width}_r{rep}';r=json.loads((root/name/'result.json').read_text())
            for row in r['trace']:
                if row['error']:continue
                g=dict(np.load(root/name/'snapshots'/f"{row['index']:03d}.npz"))
                rich_parity(g,crop(reference,row['lower'],row['upper']));checked+=1
                s=row['stats']
                if s['raw_bytes']>8192*1024 or s['compressed_bytes']>8192*1024 or s['requested_regions']>64:
                    raise ValueError('Read budget exceeded')
                if np.any(g['seen']>s['durable_source']) or np.any(g['edge_seen']>s['durable_source']):raise ValueError('Future graph metadata')
            runs[name]=r
    proto=json.loads((root/'protocol.json').read_text())
    if sha(SOURCE/'archive.sqlite')!=proto['archive_sha256']:raise ValueError('Source archive changed')
    result=dict(runs=runs,full_export=json.loads((root/'full_export/result.json').read_text()),
        exact_queries_verified=checked,source_unchanged=True,hardware_access=False,
        scope='One development recording, 48 query boxes per width repeated twice. Not a camera/robot end-to-end benchmark.')
    write(root/'summary.json',result)
    print(json.dumps({n:dict(success=r['success'],p95=r['successful_call_ms']['p95'],rss_mib=r['peak_sampled_rss_increment_bytes']/1024**2) for n,r in runs.items()},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['prepare','child','full','run','analyze']);p.add_argument('root',type=Path)
    p.add_argument('--width',type=int);p.add_argument('--repeat',type=int);a=p.parse_args();root=a.root.resolve()
    if a.command=='prepare':prepare(root)
    elif a.command=='child':child(root,a.width,a.repeat)
    elif a.command=='full':full_child(root)
    elif a.command=='analyze':analyze(root)
    else:
        for w in (1,2,4):
            for r in (1,2):
                subprocess.run([sys.executable,__file__,'child',str(root),'--width',str(w),'--repeat',str(r)],check=True,creationflags=subprocess.CREATE_NO_WINDOW)
        subprocess.run([sys.executable,__file__,'full',str(root)],check=True,creationflags=subprocess.CREATE_NO_WINDOW)
        analyze(root)
