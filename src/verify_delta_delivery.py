"""File-only delivery checks for the delta-archive experiment and its video."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from block_archive_study import ROOT, write
from replay_robot_mapping import sha
from workspace_paths import repo_file


def main(root):
    checks=[]
    for pattern in ('test_block_archive.py','test_delta_archive.py'):
        p=subprocess.run([sys.executable,'-m','unittest','discover','-s','om6_piper/tests','-p',pattern,'-v'],
                         cwd=ROOT,capture_output=True,text=True,timeout=90,
                         creationflags=subprocess.CREATE_NO_WINDOW)
        checks.append(dict(suite=pattern,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr))
    write(root/'test_results.json',checks)
    if any(c['returncode'] for c in checks):raise RuntimeError('Tests failed; inspect test_results.json')

    preserved={
        'cpp_surface/build_robot_replay_20261007/Release/camera_local_api.dll':
            '6369adb42e2f2c3addb6daa3b8764bb0f6bbdd9f25c0e13ca5e95104e3ee1b6e',
        'cpp_surface/build_block_archive_20261009_v3/Release/block_archive_api.dll':
            '13fcb61182b26afd270e202e9cd1650622941a83e4bfe7e3e7f1a3003f78e264',
        'output/block_archive_20261009_v1/video/demo.mp4':
            'e3e1a96d0cfc08aee7a2ee5871657ac4847eb1f9e230228d58d5162fbb556fc5'}
    for path,expected in preserved.items():
        if sha(ROOT/path)!=expected:raise ValueError('Preserved artifact changed: '+path)
    comparison=json.loads((root/'comparison.json').read_text())
    for name,expected in comparison['analysis_video_source_hashes'].items():
        if sha(root/comparison['source_evidence_directory']/name)!=expected or sha(repo_file(name))!=expected:
            raise ValueError('Analysis/video source changed after freezing: '+name)
    verification=json.loads((root/'video/verification.json').read_text())
    if not verification['full_ffmpeg_decode_passed'] or not verification['chronology_assertions_passed']:
        raise ValueError('Video verification incomplete')
    for entry in json.loads((root/'video/causality.json').read_text()):
        for key in ('rgbd_source_s','status_wall_s','durable_source_s'):
            stamp=entry[key]
            if stamp is not None and stamp>entry['replay_wall_s']+1e-8:raise ValueError('Future video timestamp')
    video=root/'video/demo.mp4'
    write(root/'delivery_verification.json',dict(
        suites=checks,python_native_contract_tests=11,video_sha256=sha(video),video_bytes=video.stat().st_size,
        preserved_artifact_hashes=preserved,video_checks=verification,
        hardware_access=False,live_pipeline_changed=False,manuscript_changed=False,
        scope='Six delta and five baseline binding tests. Not a standalone native executable or hardware power-loss test.',
        measurement_limits=['One development recording; repeat3 reuses it with synthetic monotonic IDs/times.',
            'The 0.5 s coalescing window is checked on new submissions, not an idle-flush timer.',
            'Payload, sampled memory and sampled file sizes are not physical I/O or instantaneous peaks.',
            '32 MiB/64-region compaction limits are work budgets, not hard realtime or RSS guarantees.',
            'Reading the full graph is offline; regional live retrieval and changed-object evaluation remain.']))
    print('PASS: 11 Python/native-binding tests, preserved baseline/live binaries and video, source hashes, encoded video chronology.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    main(p.parse_args().root.resolve())
