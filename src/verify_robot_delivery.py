"""Read-only audit of replay inputs, structural snapshots and encoded video."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import re
import cv2
import numpy as np

from workspace_paths import WORKSPACE_ROOT as ROOT, repo_file


def digest(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def verify(root, video_directory):
    report = dict(replays={})
    for name in ('original','slow'):
        folder = root/name
        data = json.loads((folder/'result.json').read_text())
        protocol = json.loads((folder/'protocol.json').read_text())
        assert data['rgb_display_uses_recorded_log_pair']
        for filename, expected in protocol['input_sha256'].items():
            assert digest(Path(protocol['source'])/filename) == expected, filename
        trace = data['trace']
        assert len(trace) == data['unique_frames']
        assert all(b['source_s'] > a['source_s'] for a,b in zip(trace,trace[1:]))
        assert len({r['depth_number'] for r in trace}) == len(trace)
        by_index = {r['log_index']:r for r in trace}
        peak_nodes = 0
        for r in data['captures']:
            assert r['source_s'] == by_index[r['log_index']]['source_s']
            with np.load(folder/'snapshots'/f'{r["stem"]}.npz') as a:
                assert a['depth'].shape == (480,640)
                assert a['pose'].shape == (4,4)
                for prefix, count_prefix in (('','recent_'),('history_','history_')):
                    p,e = a[prefix+'points'],a[prefix+'edges']
                    assert len(p) <= 40000 and len(e) <= 80000
                    assert len(p) == r[count_prefix+'nodes'] and len(e) == r[count_prefix+'edges']
                    assert np.isfinite(p).all()
                    assert not len(e) or (e.min() >= 0 and e.max() < len(p) and np.all(e[:,0] != e[:,1]))
                    peak_nodes = max(peak_nodes,len(p))
        report['replays'][name] = dict(unique_depth_frames=len(trace), snapshots=len(data['captures']),
            original_inputs_unchanged=True, chronological_snapshots=True, structural_graph_checks=True,
            peak_snapshot_nodes=peak_nodes, playback_rgb_regroupings=len(data['playback_rgb_regroupings']),
            caveat='No ground-truth accuracy, pose synchronization, or complete witness audit.')
    video = video_directory/'icra2027_demo.mp4'
    storyboard = json.loads((video_directory/'storyboard.json').read_text())
    probe = json.loads(subprocess.check_output([str(ROOT/'tmp/video_tools/ffprobe.exe'),'-v','error',
        '-show_entries','stream=codec_name,width,height,avg_frame_rate,nb_frames,pix_fmt,field_order:format=duration,size','-of','json',str(video)],text=True))
    stream = probe['streams'][0]
    assert stream['codec_name']=='h264' and stream['pix_fmt']=='yuv420p' and stream['field_order']=='progressive'
    assert (stream['width'],stream['height'],stream['avg_frame_rate']) == (1280,720,'30/1')
    assert float(probe['format']['duration']) == storyboard['duration_s'] <= 180
    assert video.stat().st_size < 19000000
    cap = cv2.VideoCapture(str(video))
    count = 0
    qa = video_directory/'encoded_qa'
    qa.mkdir(exist_ok=True)
    picks={int((r['start_s']+r['duration_s']*.55)*30) for r in storyboard['timeline']}
    # Inspect the late front-view phase too, not only the middle/orbit phase.
    picks.update(int((r['start_s']+r['duration_s']*.9)*30)
                 for r in storyboard['timeline'] if r['scene'].endswith('_frozen'))
    while True:
        ok,frame=cap.read()
        if not ok:
            break
        assert frame.shape == (720,1280,3)
        if count in picks:
            assert cv2.imwrite(str(qa/f'{count:04d}.png'),frame)
        count+=1
    cap.release()
    assert count == int(storyboard['duration_s']*30)
    report.update(video=probe, complete_video_decode_frames=count,
        paper_sha256=digest(ROOT/'output/pdf/icra2027_deferred_surface_mapping.pdf'),
        video_sha256=digest(video), paper_source_sha256=digest(ROOT/'paper_icra2027/bonn_revision/main.tex'))
    paper_root=ROOT/'paper_icra2027/bonn_revision'
    report['paper_inputs_sha256']={str(p.relative_to(paper_root)):digest(p)
        for p in paper_root.rglob('*') if p.is_file() and p.suffix in ('.tex','.bib','.cls','.bst')
        or (p.is_file() and p.parent==paper_root/'figures' and p.suffix=='.pdf')}
    pdf=ROOT/'output/pdf/icra2027_deferred_surface_mapping.pdf'
    poppler=Path('C:/Users/teuku/.cache/codex-runtimes/codex-primary-runtime/dependencies/native/poppler/Library/bin')
    info=subprocess.check_output([str(poppler/'pdfinfo.exe'),str(pdf)],text=True)
    pages=int(re.search(r'Pages:\s+(\d+)',info)[1])
    assert pages==8 and pdf.stat().st_size<6000000
    runtime_python=Path('C:/Users/teuku/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe')
    content=subprocess.check_output([str(runtime_python),'-X','utf8','-c',
        "from pypdf import PdfReader; import sys; print('\\n'.join(p.extract_text() for p in PdfReader(sys.argv[1]).pages))",
        str(pdf)],text=True,encoding='utf-8')
    assert '??' not in content and 'Anonymous Authors' in content
    assert all(x in content for x in ('19.84','18.33','29.33','28.35','312,767','717,955'))
    report['pdf']=dict(pages=pages,bytes=pdf.stat().st_size,anonymous=True,
                       checked_robot_table_values=True,unresolved_question_marks=False)
    report['render_sources_sha256']={str(p.relative_to(ROOT)):digest(p) for p in (
        ROOT/'paper_icra2027/bonn_revision/render_demo.py',
        ROOT/'paper_icra2027/bonn_revision/build_assets.py',
        repo_file('render_robot_mapping.py'))}
    (root/'verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('video_directory',type=Path)
    a=p.parse_args();verify(a.root,a.video_directory)
