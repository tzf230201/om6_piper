"""File-only regression and evidence checks for the regional read-back study."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import numpy as np
from regional_read_study import ROOT, SOURCE, DLL, WRITER_DLL, sha, write, rich_parity
from workspace_paths import repo_file, workspace_file


def verify(root):
    target=root/'delivery_verification.json'
    if target.exists():raise ValueError('Keep the previous verification; choose a fresh study root')
    suites=[]
    commands=[
        ('regional_reader_current', [sys.executable,'-m','unittest','discover','-s','om6_piper/tests','-p','test_regional_reader.py','-v']),
        ('block_current_writer', [sys.executable,'-c',
            "import sys,unittest,functools;sys.path[:0]=['om6_piper/src','om6_piper/tests'];import test_block_archive as t;"
            "from regional_read_study import WRITER_DLL;t.Block=functools.partial(t.Block,dll=WRITER_DLL);"
            "r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(t));sys.exit(not r.wasSuccessful())"]),
        ('delta_current_writer_legacy_crash_child', [sys.executable,'-c',
            "import sys,unittest;sys.path[:0]=['om6_piper/src','om6_piper/tests'];import test_delta_archive as t;"
            "from regional_read_study import WRITER_DLL;t.DELTA_DLL=WRITER_DLL;"
            "r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(t));sys.exit(not r.wasSuccessful())"])]
    for name,command in commands:
        p=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=90)
        suites.append(dict(suite=name,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr))
        print(name,p.returncode,flush=True)
    write(root/'test_results.json',suites)
    if any(s['returncode'] for s in suites):raise RuntimeError('Regression test failed; see test_results.json')

    protocol=json.loads((root/'protocol.json').read_text())
    assert sha(DLL)==protocol['reader_dll_sha256']
    assert sha(WRITER_DLL)==protocol['writer_dll_sha256']
    assert sha(SOURCE/'archive.sqlite')==protocol['archive_sha256']
    # The original loop-based job is intentionally retained in the initial source freeze.
    for name,h in protocol['source_hashes'].items():
        if name!='om6_piper/regional_read_job.py':assert sha(workspace_file(name))==h,name
    optimized=json.loads((root/'jobs_vectorized/measurements.json').read_text())
    for name,h in optimized['source_hashes'].items():assert sha(repo_file(name))==h,name
    previous=json.loads((root/'jobs/measurements.json').read_text())
    old_rows={(r['width'],r['index']):r for r in previous['trace']}
    matched=0
    for row in optimized['trace']:
        old=old_rows[row['width'],row['index']]
        assert row['complete']==old['complete'] and row['error']==old['error']
        assert row['revision']==old['revision']
        for step in row['steps']:
            s=step['stats']
            assert s['requested_regions']==1 and s['raw_bytes']<=8192*1024 and s['compressed_bytes']<=8192*1024
            assert (s['durable_batch'],s['durable_frames'],s['durable_source'])==tuple(row['revision'])
        if not row['complete']:continue
        assert row['retained_array_bytes']<=32*1024**2
        filename=f"{row['width']}_{row['index']:03d}.npz"
        rich_parity(dict(np.load(root/'jobs/snapshots'/filename)),dict(np.load(root/'jobs_vectorized/snapshots'/filename)))
        matched+=1
    assert matched==89

    prior=json.loads((ROOT/'output/delta_archive_20261009_v1/delivery_verification.json').read_text())
    preserved=dict(prior['preserved_artifact_hashes'])
    preserved['output/delta_archive_20261009_v1/video/demo.mp4']=prior['video_sha256']
    preserved['cpp_surface/build_delta_archive_20261009_v2/Release/block_archive_api.dll']='a7ada4f52231b583395eda7ecb85c5c93f6fb2661d9379e511602af1da842c0f'
    for name,h in preserved.items():assert sha(ROOT/name)==h,name

    video=json.loads((root/'video/verification.json').read_text())
    assert sha(root/'video/demo.mp4')==video['sha256']
    assert sha(repo_file('render_regional_read.py'))==video['renderer_sha256']
    assert video['archive_sha256']==protocol['archive_sha256'] and video['full_decode_passed']
    assert len(video['timeline'])==16 and video['ffprobe']['streams'][0]['nb_read_frames']=='2076'
    assert (root/'video/visual_review.json').exists(), 'Finish encoded-frame visual review first'

    evidence=root/'final_source_evidence';evidence.mkdir()
    paths=list(protocol['source_hashes'])+['om6_piper/regional_job_study.py','om6_piper/render_regional_read.py',
        'om6_piper/verify_regional_delivery.py','om6_piper/README.md','om6_piper/render_live_scan_review.py']
    hashes={}
    for name in paths:
        source=workspace_file(name);dest=evidence/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,dest);hashes[name]=sha(source)
    write(target,dict(suites=suites,python_native_binding_tests=18,
        test_scope='7 regional + 5 block tests on new native binaries; 6 delta tests use the new writer except the abrupt-exit child uses the preserved delta DLL and recovery uses the new DLL.',
        reader_dll_sha256=sha(DLL),writer_dll_sha256=sha(WRITER_DLL),source_hashes=hashes,
        original_and_vectorized_jobs_exact=matched,job_outcomes_and_revisions_unchanged=True,
        archive_unchanged=True,preserved_artifact_hashes=preserved,video=video,
        hardware_access=False,live_pipeline_changed=False,manuscript_changed=False,
        limitations=['One development recording, not independent validation or a camera-to-display benchmark.',
            'Counts and raw/retained-array budgets are not hard execution-time or total RSS bounds.',
            'Single concurrent unit fixture does not prove sustained writer/reader throughput.',
            'Native C++ retrieval; job orchestration and vectorized assembly remain Python/NumPy.',
            'SQLite 3.51.1 system runtime is not the upstream WAL-reset-fix version; reader is read-only, one writer connection.']))
    print('VERIFIED',matched,'identical completed jobs; 18 tests; preserved baselines and demo',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);a=p.parse_args();verify(a.root.resolve())
