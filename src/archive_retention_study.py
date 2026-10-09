"""Offline, matched-input storage ablation. Never opens a camera or robot.

prepare reads an existing BAG explicitly; run reads only the prepared depth file.
One run per process isolates RAM variants. Update timing EXCLUDES full exports.
"""
import argparse
import ctypes as ct
import json
from pathlib import Path
import shutil
import time
import numpy as np
import psutil
from scipy.spatial import cKDTree
from replay_robot_mapping import ROOT, DP, IP, sha, summary
from review_live_scan import read_rows, validate_trace, write
from table_scan import configuration, camera_world_pose
from workspace_paths import repo_file

DLL = ROOT/'cpp_surface/build_archive_20261008_v1/Release/archive_study_api.dll'
REVIEW = ROOT/'output/table_scan_review_20261007_v1/reconstruction'
MODES = {'bounded': (40000,80000,180.), 'large_ttl': (500000,3000000,180.),
         'large_persistent': (500000,3000000,1e9), 'disk': (0,0,1e9)}
LP = ct.POINTER(ct.c_int64)


class Study:
    def __init__(self,intr,mode,path,dll=DLL,cache_kib=4096):
        self.lib=ct.CDLL(str(dll));self.lib.archive_study_error.restype=ct.c_char_p
        self.lib.archive_study_create.argtypes=[ct.c_int]*2+[ct.c_double]*4+[ct.c_char_p,ct.c_int,ct.c_int,ct.c_double]
        self.lib.archive_study_create.restype=ct.c_void_p
        self.lib.archive_study_update.argtypes=[ct.c_void_p,DP,ct.c_int,ct.c_int64,ct.c_double,DP,DP]
        self.lib.archive_study_export.argtypes=[ct.c_void_p,DP]
        self.lib.archive_study_copy.argtypes=[ct.c_void_p,DP,ct.c_int,IP,ct.c_int,LP,DP]
        self.lib.archive_study_destroy.argtypes=[ct.c_void_p]
        self.h=self.lib.archive_study_create(intr['width'],intr['height'],*[intr[k] for k in ('fx','fy','cx','cy')],
            str(path).encode('utf8') if mode=='disk' else None,*MODES[mode])
        if not self.h:raise RuntimeError(self.lib.archive_study_error().decode())
        if mode=='disk' and cache_kib!=4096:
            self.lib.archive_study_cache_kib.argtypes=[ct.c_void_p,ct.c_int]
            self.checked(self.lib.archive_study_cache_kib(self.h,cache_kib))

    def checked(self,code):
        if code:raise RuntimeError(self.lib.archive_study_error().decode())

    def update(self,depth,pose,number,stamp):
        d=np.ascontiguousarray(depth,dtype='float64');p=np.ascontiguousarray(pose,dtype='float64');s=np.empty(4)
        self.checked(self.lib.archive_study_update(self.h,d.ctypes.data_as(DP),d.size,number,stamp,
                                                  p.ctypes.data_as(DP),s.ctypes.data_as(DP)))
        return s

    def snapshot(self):
        counts=np.empty(2);self.checked(self.lib.archive_study_export(self.h,counts.ctypes.data_as(DP)))
        n,e=map(int,counts);p=np.empty((n,3));edges=np.empty((e,2),dtype='int32')
        keys=np.empty((n,4),dtype='int64');seen=np.empty(n)
        self.checked(self.lib.archive_study_copy(self.h,p.ctypes.data_as(DP),n,edges.ctypes.data_as(IP),e,
                                                 keys.ctypes.data_as(LP),seen.ctypes.data_as(DP)))
        assert np.isfinite(p).all() and (not e or (edges.min()>=0 and edges.max()<n))
        return dict(points=p,edges=edges,keys=keys,seen=seen)

    def close(self):
        if self.h:self.lib.archive_study_destroy(self.h);self.h=None


def prepare(out):
    import pyrealsense2 as rs
    out.mkdir(parents=True,exist_ok=False)
    original=json.loads((REVIEW/'protocol.json').read_text());source=Path(original['source'])
    # Full fingerprints, including the BAG, must still match the frozen live review.
    for name,info in original['input_fingerprints'].items():
        if sha(source/name)!=info['sha256']:raise ValueError('Original input changed: '+name)
    rows=read_rows(source/'frames.jsonl');updates=read_rows(source/'live_map/updates.jsonl')
    lookup=validate_trace(rows,updates,json.loads((source/'live_map/result.json').read_text()))
    selected=[];last=-float('inf');model,_,config=configuration()
    for u in updates:
        stamp=u['arrival_perf_s']
        if stamp-last>=.2:
            r=lookup[u['depth_number']]
            selected.append(dict(number=u['depth_number'],stamp=stamp,source_s=stamp-rows[0]['host_arrival_perf_s'],
                depth_ms=r['depth_timestamp_ms'],pose=camera_world_pose(model,r['joints_after']['q_rad'],config).tolist()))
            last=stamp
    meta=json.loads((source/'metadata.json').read_text());intr=meta['streams']['depth']
    if any(intr['coeffs']):raise ValueError('Rectification required')
    cfg=rs.config();cfg.enable_device_from_file(str(source/'rgbd.bag'),False);pipe=rs.pipeline()
    profile=pipe.start(cfg);profile.get_device().as_playback().set_real_time(False);i=0
    try:
        with (out/'depth.u16').open('xb') as f:
            while i<len(selected):
                frames=pipe.wait_for_frames(5000);d=frames.get_depth_frame()
                if not d:continue
                number=d.get_frame_number();r=selected[i]
                if number<r['number']:continue
                if number!=r['number'] or abs(d.get_timestamp()-r['depth_ms'])>.01:
                    raise ValueError('BAG/history admission association mismatch')
                f.write(np.asarray(d.get_data(),dtype='<u2').tobytes());i+=1
                if i%200==0:print('PACK',i,'/',len(selected),flush=True)
    finally:pipe.stop()
    write(out/'protocol.json',dict(source=str(source),original_protocol=str(REVIEW/'protocol.json'),
        input_fingerprints=original['input_fingerprints'],records=selected,intr=intr,depth_scale=meta['depth_scale'],
        hardware_access=False,cell_m=.01,stride=8,near_m=.07,far_m=1.5,
        world_offset_m=[0,0,0],pose='Recorded joint feedback + nominal CAD, not hand-eye calibrated',
        policy='Exactly the original <=5 Hz history admissions; not all 30 Hz recent updates',
        depth_sha256=sha(out/'depth.u16')))
    print('PACKED',i,flush=True)


def run(pack,mode,out,exports=False,dll=DLL,cache_kib=4096):
    out.mkdir(parents=True,exist_ok=False)
    shutil.copy2(__file__,out/'study_source.py')
    source_hash=sha(out/'study_source.py')
    protocol=json.loads((pack/'protocol.json').read_text());intr=protocol['intr'];records=protocol['records']
    if sha(pack/'depth.u16')!=protocol['depth_sha256']:raise ValueError('Prepared depth changed')
    # Export runs are separate from performance runs: full maps consume extra RAM.
    captures=json.loads((REVIEW/'reconstruction.json').read_text())['captures']
    scan=max(c['source_s'] for c in captures if c['stage']=='Return along near row')
    export_ids={i for i,r in enumerate(records) if i==0 or (exports and int(r['source_s']/5)>int(records[i-1]['source_s']/5))}
    export_ids|={max(i for i,r in enumerate(records) if r['source_s']<=scan),len(records)-1}
    if not exports:export_ids=set()
    if exports:(out/'snapshots').mkdir()
    process=psutil.Process();before=process.memory_info();mapper=Study(intr,mode,out/'archive.sqlite',dll,cache_kib)
    created=process.memory_info();trace=[];snaps=[];pixels=intr['width']*intr['height']
    start=time.perf_counter()
    try:
        with (pack/'depth.u16').open('rb') as f:
            for i,r in enumerate(records):
                raw=f.read(pixels*2)
                if len(raw)!=pixels*2:raise ValueError('Truncated prepared data')
                depth=np.frombuffer(raw,dtype='<u2').reshape(intr['height'],intr['width']).astype(float)*protocol['depth_scale']
                a=time.perf_counter();s=mapper.update(depth,r['pose'],r['number'],r['stamp']);wall=(time.perf_counter()-a)*1000
                mem=process.memory_info()
                disk_bytes=sum(p.stat().st_size for p in out.glob('archive.sqlite*'))
                trace.append(dict(index=i,number=r['number'],source_s=r['source_s'],extract_ms=s[0],store_ms=s[1],native_ms=s[2],
                    call_ms=wall,sqlite_cache_bytes=int(s[3]),rss=mem.rss,private=mem.private,disk_bytes=disk_bytes))
                if i in export_ids:
                    snap=mapper.snapshot();stem=f'{i:04d}';np.savez_compressed(out/'snapshots'/f'{stem}.npz',**snap)
                    snaps.append(dict(index=i,stem=stem,source_s=r['source_s'],nodes=len(snap['points']),edges=len(snap['edges'])))
                if i%200==0:print(mode,i,'/',len(records),flush=True)
        elapsed=time.perf_counter()-start
        final=mapper.snapshot();np.savez_compressed(out/'final.npz',**final)
    finally:mapper.close()
    write(out/'trace.json',trace)
    result=dict(mode=mode,export_run=exports,frames=len(records),source_duration_s=records[-1]['source_s'],
        update_time_scope='Offline history extraction+storage; excludes BAG decoding, 30 Hz recent mapper, export, renderer, camera/USB, and robot IO',
        native_ms=summary([r['native_ms'] for r in trace]),store_ms=summary([r['store_ms'] for r in trace]),
        extract_ms=summary([r['extract_ms'] for r in trace]),call_ms=summary([r['call_ms'] for r in trace]),
        updates_over_200_ms=sum(r['call_ms']>200 for r in trace),updates_over_33_ms=sum(r['call_ms']>1000/30 for r in trace),
        loop_wall_s=elapsed,rss_before_bytes=before.rss,rss_created_bytes=created.rss,
        private_before_bytes=before.private,private_created_bytes=created.private,
        max_update_rss_bytes=max(r['rss'] for r in trace),max_update_private_bytes=max(r['private'] for r in trace),
        max_sqlite_cache_bytes=max(r['sqlite_cache_bytes'] for r in trace),
        max_database_plus_wal_bytes=max(r['disk_bytes'] for r in trace),
        closed_database_bytes=sum(p.stat().st_size for p in out.glob('archive.sqlite*')),
        final_nodes=len(final['points']),final_edges=len(final['edges']),snapshots=snaps,
        native_dll_sha256=sha(dll),script_sha256=source_hash,pack_protocol_sha256=sha(pack/'protocol.json'),
        mode_config=MODES[mode],sqlite_cache_target_kib=cache_kib if mode=='disk' else None,
        caveats=['Archive retains last-observed surfaces, including potentially changed objects.',
                 'Performance run has no exports before final measurement; export runs are not benchmarks.',
                 'SQLite cache bound is not a bound on total process/OS file-cache RAM.',
                 'No hardware commands or live realtime validation.'])
    write(out/'result.json',result);print(json.dumps({k:v for k,v in result.items() if k!='snapshots'},indent=2),flush=True)


def canonical(a):
    k=a['keys'];order=np.lexsort(tuple(k[:,i] for i in range(3,-1,-1)))
    return k[order],a['points'][order]


def analyze(root):
    runs={name:json.loads((root/name/'result.json').read_text()) for name in MODES}
    arrays={name:dict(np.load(root/name/'final.npz')) for name in MODES}
    a,b=canonical(arrays['large_persistent']),canonical(arrays['disk'])
    parity=bool(np.array_equal(a[0],b[0]) and np.array_equal(a[1],b[1]))
    if not parity:raise ValueError('Disk / large persistent RAM geometry mismatch or RAM saturation')
    # Edge comparison in key space; row order / LRU slot numbers are not identities.
    def edge_keys(g):
        e=g['keys'][g['edges']].reshape(-1,8)
        return e[np.lexsort(tuple(e[:,i] for i in range(7,-1,-1)))]
    edge_parity=bool(np.array_equal(edge_keys(arrays['large_persistent']),edge_keys(arrays['disk'])))
    if not edge_parity:raise ValueError('Disk / RAM topology mismatch')
    extra={}
    if (root/'disk32/result.json').exists():
        r32=json.loads((root/'disk32/result.json').read_text());g32=dict(np.load(root/'disk32/final.npz'))
        c32=canonical(g32)
        same=bool(np.array_equal(b[0],c32[0]) and np.array_equal(b[1],c32[1]) and
                  np.array_equal(edge_keys(arrays['disk']),edge_keys(g32)))
        if not same:raise ValueError('Changing cache size changed geometry/topology')
        extra['disk32']=dict(result=r32,exact_same_map=same)
    live=np.load(Path(json.loads((REVIEW/'protocol.json').read_text())['source'])/'live_map/final.npz')
    baseline_exact=bool(np.array_equal(live['history_points'],arrays['bounded']['points']) and
                        np.array_equal(live['history_edges'],arrays['bounded']['edges']))
    if not baseline_exact:raise ValueError('Bounded baseline no longer reproduces live history')
    scan=np.load(REVIEW/'snapshots/0248.npz')['history_points']
    roi=(scan[:,0]>=.25)&(scan[:,0]<=.85)&(scan[:,1]>=-.45)&(scan[:,1]<=.45)&(abs(scan[:,2])<=.05)
    scores={}
    for name,g in arrays.items():
        d=cKDTree(g['points']).query(scan,workers=1)[0]
        scores[name]=dict(full_reference_retained_within20mm=float(np.mean(d<=.02)),
            table_roi_retained_within20mm=float(np.mean(d[roi]<=.02)),table_roi_median_distance_mm=float(np.median(d[roi])*1000),
            table_roi_fraction_by_radius_mm={str(mm):float(np.mean(d[roi]<=mm/1000)) for mm in (1,2,5,10,20)})
    result=dict(baseline_exact_live=baseline_exact,disk_ram_geometry_exact=parity,disk_ram_edge_keys_exact=edge_parity,
        reference_scan_s=131.272254,reference_nodes=len(scan),reference_roi_nodes=int(roi.sum()),
        retention=scores,runs=runs,cache_ablation=extra,
        timing_caution='Single-run exploratory timing, not a repeated controlled benchmark. The initial 4 MiB pilot overlapped build/diagnostic activity; do not report its exact latency as an isolated benchmark.',
        claim='Storage ablation only; retaining more observations is NOT demonstrated geometric accuracy or dynamic robustness.',
        metric='Fixed 40k-node baseline scan reference. Fraction with a final neighbour <=20 mm; not semantic area/GT coverage.')
    write(root/'comparison.json',result)
    evidence=root/'source_evidence';evidence.mkdir(exist_ok=True)
    files=[repo_file(n) for n in ('archive_retention_study.py','render_archive_study.py','test_archive_study.py')]
    files += [ROOT/'cpp_surface'/n for n in ('include/observed_archive.hpp','src/observed_archive.cpp',
              'src/archive_study_api.cpp','tests/observed_archive_tests.cpp','CMakeLists.txt')]
    for source in files:
        dest=evidence/source.name
        if dest.exists() and sha(dest)!=sha(source):raise ValueError('Frozen study source changed; use a new evidence folder')
        if not dest.exists():shutil.copy2(source,dest)
    write(root/'source_manifest.json',{str(p.relative_to(ROOT)):sha(p) for p in files})
    print(json.dumps(dict(retention=scores,baseline_exact_live=baseline_exact,disk_ram_geometry_exact=parity),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='cmd',required=True)
    q=sub.add_parser('prepare');q.add_argument('output',type=Path)
    q=sub.add_parser('run');q.add_argument('pack',type=Path);q.add_argument('mode',choices=MODES);q.add_argument('output',type=Path);q.add_argument('--exports',action='store_true')
    q.add_argument('--dll',type=Path,default=DLL);q.add_argument('--cache-kib',type=int,default=4096)
    q=sub.add_parser('analyze');q.add_argument('root',type=Path)
    args=p.parse_args()
    if args.cmd=='prepare':prepare(args.output.resolve())
    elif args.cmd=='run':run(args.pack.resolve(),args.mode,args.output.resolve(),args.exports,args.dll.resolve(),args.cache_kib)
    else:analyze(args.root.resolve())
