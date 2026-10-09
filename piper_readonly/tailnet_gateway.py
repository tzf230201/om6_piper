"""Loopback-only read-only adapter behind user-authorized Tailscale TCP Serve.

Fixed backend, explicit Host/Origin allowlist, no motor/device access, no writes.
Tailscale ACLs govern remote access; this process is not a public/LAN listener.
"""
import argparse
import json
from pathlib import Path
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

ALLOWED_HOSTS = {'100.93.232.22:8765', 'msi.tail455172.ts.net:8765', 'msi:8765'}
ROUTES = {'/', '/style.css', '/viewer.bundle.js', '/viewer.bundle.js.LEGAL.txt',
          '/api/state', '/api/model', '/api/fgng'}
from app_paths import WORKSPACE_ROOT

PAPER_PATH = WORKSPACE_ROOT / 'output/pdf/icra2027_anonymous_draft.pdf'
EXPERIMENT_PATH = WORKSPACE_ROOT / 'output/pdf/icra2027_experiment_extension.pdf'
DELIVERY_ROOT = WORKSPACE_ROOT / 'output'
CURRENT_ARTIFACTS = {
    '/camera/review': (DELIVERY_ROOT / 'stationary_camera_20261005_v2/review.html', 'text/html; charset=utf-8'),
    '/camera/demo.mp4': (DELIVERY_ROOT / 'stationary_camera_20261005_v2/demo.mp4', 'video/mp4'),
    '/camera/status.json': (DELIVERY_ROOT / 'stationary_camera_20261005_v2/summary.json', 'application/json'),
    '/stream2/review': (DELIVERY_ROOT / 'stream_reserve_20261003_v1/review.html', 'text/html; charset=utf-8'),
    '/stream2/demo.mp4': (DELIVERY_ROOT / 'stream_reserve_20261003_v1/mapping_demo.mp4', 'video/mp4'),
    '/stream2/faults.mp4': (DELIVERY_ROOT / 'stream_reserve_20261003_v1/fault_demo.mp4', 'video/mp4'),
    '/stream2/analysis.json': (DELIVERY_ROOT / 'stream_reserve_20261003_v1/analysis.json', 'application/json'),
    '/stream2/protocol.json': (DELIVERY_ROOT / 'stream_reserve_20261003_v1/protocol.json', 'application/json'),
    '/stream2/audit.json': (DELIVERY_ROOT / 'stream_reserve_20261003_v1/provenance_audit.json', 'application/json'),
    '/stream2/source.zip': (DELIVERY_ROOT / 'stream_reserve_20261003_v1/source.zip', 'application/zip'),
    '/stream/review': (DELIVERY_ROOT / 'stream_consumer_20261002_v1/review.html', 'text/html; charset=utf-8'),
    '/stream/demo.mp4': (DELIVERY_ROOT / 'stream_consumer_20261002_v1/stream_demo.mp4', 'video/mp4'),
    '/stream/analysis.json': (DELIVERY_ROOT / 'stream_consumer_20261002_v1/analysis.json', 'application/json'),
    '/stream/protocol.json': (DELIVERY_ROOT / 'stream_consumer_20261002_v1/protocol.json', 'application/json'),
    '/stream/audit.json': (DELIVERY_ROOT / 'stream_consumer_20261002_v1/provenance_audit.json', 'application/json'),
    '/stream/source.zip': (DELIVERY_ROOT / 'stream_consumer_20261002_v1/source.zip', 'application/zip'),
    '/realtime/review': (DELIVERY_ROOT / 'realtime_o2_20261002_v1/review.html', 'text/html; charset=utf-8'),
    '/realtime/demo.mp4': (DELIVERY_ROOT / 'realtime_o2_20261002_v1/realtime_demo.mp4', 'video/mp4'),
    '/realtime/analysis.json': (DELIVERY_ROOT / 'realtime_o2_20261002_v1/analysis.json', 'application/json'),
    '/realtime/protocol.json': (DELIVERY_ROOT / 'realtime_o2_20261002_v1/protocol.json', 'application/json'),
    '/realtime/audit.json': (DELIVERY_ROOT / 'realtime_o2_20261002_v1/provenance_audit.json', 'application/json'),
    '/realtime/build.json': (DELIVERY_ROOT / 'realtime_o2_20261002_v1/build_audit.json', 'application/json'),
    '/realtime/tests.json': (DELIVERY_ROOT / 'realtime_o2_20261002_v1/unit_test_status.json', 'application/json'),
    '/realtime/reading-checklist.txt': (DELIVERY_ROOT.parent / 'paper_icra2027/SOURCES.md', 'text/plain; charset=utf-8'),
    # Historical blocker and V1 evidence remain accessible; not current status.
    '/realtime/blocker.json': (DELIVERY_ROOT / 'realtime_graph_20261002_v2/application_control_evidence.json', 'application/json'),
    '/realtime/source.zip': (DELIVERY_ROOT / 'realtime_o2_20261002_v1/source.zip', 'application/zip'),
    '/realtime-v1/demo.mp4': (DELIVERY_ROOT / 'realtime_graph_20261002_v1/realtime_demo.mp4', 'video/mp4'),
    '/realtime-v1/analysis.json': (DELIVERY_ROOT / 'realtime_graph_20261002_v1/analysis.json', 'application/json'),
    '/realtime-v1/source.zip': (DELIVERY_ROOT / 'realtime_graph_20261002_v1/source.zip', 'application/zip'),
    '/consensus/review': (DELIVERY_ROOT / 'pose_consensus_20261002_v1/review.html', 'text/html; charset=utf-8'),
    '/consensus/demo.mp4': (DELIVERY_ROOT / 'pose_consensus_20261002_v1/consensus_demo.mp4', 'video/mp4'),
    '/consensus/analysis.json': (DELIVERY_ROOT / 'pose_consensus_20261002_v1/analysis.json', 'application/json'),
    '/consensus/protocol.json': (DELIVERY_ROOT / 'pose_consensus_20261002_v1/protocol.json', 'application/json'),
    '/consensus/audit.json': (DELIVERY_ROOT / 'pose_consensus_20261002_v1/verification.json', 'application/json'),
    '/consensus/source.zip': (DELIVERY_ROOT / 'pose_consensus_20261002_v1/source.zip', 'application/zip'),
    '/pose-v2/review': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v2/review.html', 'text/html; charset=utf-8'),
    '/pose-v2/demo.mp4': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v2/pose_v2_demo.mp4', 'video/mp4'),
    '/pose-v2/comparison.json': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v2/comparison.json', 'application/json'),
    '/pose-v2/protocol.json': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v2/protocol.json', 'application/json'),
    '/pose-v2/audit.json': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v2/completion_audit.json', 'application/json'),
    '/pose-v2/status.json': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v2/run_status.json', 'application/json'),
    '/pose-v2/source.zip': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v2/source.zip', 'application/zip'),
    '/pose/review': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v1/review.html', 'text/html; charset=utf-8'),
    '/pose/demo.mp4': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v1/pose_demo.mp4', 'video/mp4'),
    '/pose/analysis.json': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v1/analysis.json', 'application/json'),
    '/pose/protocol.json': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v1/protocol.json', 'application/json'),
    '/pose/verification.json': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v1/verification.json', 'application/json'),
    '/pose/blocker.json': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v2/blocker.json', 'application/json'),
    '/pose/source.zip': (DELIVERY_ROOT / 'pose_diagnostic_20261002_v1/source.zip', 'application/zip'),
    '/refresh/review': (DELIVERY_ROOT / 'local_refresh_20261001_v1/review.html', 'text/html; charset=utf-8'),
    '/refresh/demo.mp4': (DELIVERY_ROOT / 'local_refresh_20261001_v1/refresh_demo.mp4', 'video/mp4'),
    '/refresh/analysis.json': (DELIVERY_ROOT / 'local_refresh_20261001_v1/analysis.json', 'application/json'),
    '/refresh/protocol.json': (DELIVERY_ROOT / 'local_refresh_20261001_v1/protocol.json', 'application/json'),
    '/refresh/verification.json': (DELIVERY_ROOT / 'local_refresh_20261001_v1/verification.json', 'application/json'),
    '/refresh/source.zip': (DELIVERY_ROOT / 'local_refresh_20261001_v1/source.zip', 'application/zip'),
    '/reference/review': (DELIVERY_ROOT / 'reference_guard_20261001_v1/review.html', 'text/html; charset=utf-8'),
    '/reference/demo.mp4': (DELIVERY_ROOT / 'reference_guard_20261001_v1/reference_demo.mp4', 'video/mp4'),
    '/reference/analysis.json': (DELIVERY_ROOT / 'reference_guard_20261001_v1/analysis.json', 'application/json'),
    '/reference/protocol.json': (DELIVERY_ROOT / 'reference_guard_20261001_v1/protocol.json', 'application/json'),
    '/reference/verification.json': (DELIVERY_ROOT / 'reference_guard_20261001_v1/verification.json', 'application/json'),
    '/reference/source.zip': (DELIVERY_ROOT / 'reference_guard_20261001_v1/source.zip', 'application/zip'),
    '/lifecycle/review': (DELIVERY_ROOT / 'lifecycle_20260930_v1/review.html', 'text/html; charset=utf-8'),
    '/lifecycle/demo.mp4': (DELIVERY_ROOT / 'lifecycle_20260930_v1/lifecycle_demo.mp4', 'video/mp4'),
    '/lifecycle/analysis.json': (DELIVERY_ROOT / 'lifecycle_20260930_v1/analysis.json', 'application/json'),
    '/lifecycle/protocol.json': (DELIVERY_ROOT / 'lifecycle_20260930_v1/protocol.json', 'application/json'),
    '/lifecycle/source.zip': (DELIVERY_ROOT / 'lifecycle_20260930_v1/source.zip', 'application/zip'),
    '/lifecycle/verification.json': (DELIVERY_ROOT / 'lifecycle_20260930_v1/verification.json', 'application/json'),
    '/multidata/review': (DELIVERY_ROOT / 'multidata_robotics_20260930_v2/review.html', 'text/html; charset=utf-8'),
    '/multidata/demo.mp4': (DELIVERY_ROOT / 'multidata_robotics_20260930_v2/multidata_demo_reviewed.mp4', 'video/mp4'),
    '/multidata/diagnosis.json': (DELIVERY_ROOT / 'multidata_robotics_20260930_v2/quality_failure_diagnosis.json', 'application/json'),
    '/multidata/analysis.json': (DELIVERY_ROOT / 'multidata_robotics_20260930_v2/analysis.json', 'application/json'),
    '/multidata/protocol.json': (DELIVERY_ROOT / 'multidata_robotics_20260930_v2/protocol.json', 'application/json'),
    '/multidata/source.zip': (DELIVERY_ROOT / 'multidata_robotics_20260930_v2/multidata_source.zip', 'application/zip'),
    '/paper/review': (DELIVERY_ROOT / 'icra2027_evidence_package/review.html', 'text/html; charset=utf-8'),
    '/paper/deferred-surface.pdf': (DELIVERY_ROOT / 'pdf/icra2027_deferred_surface_mapping.pdf', 'application/pdf'),
    '/paper/demo.mp4': (DELIVERY_ROOT / 'icra2027_evidence_package/video/icra2027_demo.mp4', 'video/mp4'),
    '/paper/source.zip': (DELIVERY_ROOT / 'icra2027_deferred_surface_source.zip', 'application/zip'),
    '/research/review': (DELIVERY_ROOT / 'research_20260930/review.html', 'text/html; charset=utf-8'),
    '/research/demo.mp4': (DELIVERY_ROOT / 'research_20260930/cross_dataset_failures_demo.mp4', 'video/mp4'),
    '/research/report.md': (DELIVERY_ROOT / 'research_20260930/REPORT.md', 'text/plain; charset=utf-8'),
    '/research/latency.png': (DELIVERY_ROOT / 'research_20260930/latency.png', 'image/png'),
    '/research/phases.png': (DELIVERY_ROOT / 'research_20260930/phases.png', 'image/png'),
    '/research/source.zip': (DELIVERY_ROOT / 'research_20260930/research_source.zip', 'application/zip'),
    '/native/review': (DELIVERY_ROOT / 'cpp_research_20260930/review.html', 'text/html; charset=utf-8'),
    '/native/demo.mp4': (DELIVERY_ROOT / 'cpp_research_20260930/cpp_surface_demo.mp4', 'video/mp4'),
    '/native/timing.png': (DELIVERY_ROOT / 'cpp_research_20260930/timing.png', 'image/png'),
    '/native/source.zip': (DELIVERY_ROOT / 'cpp_research_20260930/native_source.zip', 'application/zip'),
    '/regional/review': (DELIVERY_ROOT / 'regional_20260930_v1/review.html', 'text/html; charset=utf-8'),
    '/regional/demo.mp4': (DELIVERY_ROOT / 'regional_20260930_v1/regional_demo.mp4', 'video/mp4'),
    '/regional/preview.png': (DELIVERY_ROOT / 'regional_20260930_v1/preview_intro.png', 'image/png'),
    '/regional/analysis.json': (DELIVERY_ROOT / 'regional_20260930_v1/analysis.json', 'application/json'),
    '/regional/source.zip': (DELIVERY_ROOT / 'regional_20260930_v1/regional_source.zip', 'application/zip'),
    '/visible/review': (DELIVERY_ROOT / 'visible_regional_20260930_v2/review.html', 'text/html; charset=utf-8'),
    '/visible/demo.mp4': (DELIVERY_ROOT / 'visible_regional_20260930_v2/visible_demo.mp4', 'video/mp4'),
    '/visible/preview.png': (DELIVERY_ROOT / 'visible_regional_20260930_v2/preview_intro.png', 'image/png'),
    '/visible/analysis.json': (DELIVERY_ROOT / 'visible_regional_20260930_v2/analysis.json', 'application/json'),
    '/visible/source.zip': (DELIVERY_ROOT / 'visible_regional_20260930_v2/visible_source.zip', 'application/zip'),
}

def cad_preview_routes(directory=None, prefix='/camera/cad'):
    """Load this offline export's exact files only; never a directory server."""
    directory = (directory or DELIVERY_ROOT / 'cad_preview_20261005_v1').resolve()
    manifest = directory / 'routes.json'
    if not manifest.is_file():
        return {}
    result = {}
    for url, (relative, mime) in json.loads(manifest.read_text()).items():
        document = (directory / relative).resolve()
        if (url not in {prefix, prefix+'/'} and not url.startswith(prefix+'/')) or '..' in url or '?' in url:
            raise ValueError('Invalid CAD preview route')
        if not document.is_relative_to(directory) or mime not in {
            'application/json', 'application/octet-stream', 'text/html; charset=utf-8',
            'text/javascript', 'text/plain', 'text/css', 'image/png', 'video/mp4'}:
            raise ValueError('Invalid CAD preview artifact')
        result[url] = (document, mime)
    return result


CURRENT_ARTIFACTS.update(cad_preview_routes())
CURRENT_ARTIFACTS.update(cad_preview_routes(DELIVERY_ROOT/'trajectory_wrist_20261005_v1', '/trajectory'))
CURRENT_ARTIFACTS.update(cad_preview_routes(DELIVERY_ROOT/'trajectory_scan_20261005_v2', '/trajectory/scan'))


def make_handler(backend_port=8765):
    class Handler(BaseHTTPRequestHandler):
        def do_HEAD(self):
            self.do_GET()

        def log_message(self, fmt, *args):
            if len(args)>1 and str(args[1])=='200':return
            super().log_message(fmt,*args)

        def serve_artifact(self, document, mime):
            """Fixed artifact allowlist with bounded streaming and mobile byte ranges."""
            try:
                stream = document.open('rb')
            except OSError:
                return self.send_error(404, 'Artifact unavailable')
            with stream:
                stream.seek(0, 2)
                size = stream.tell()
                start, end, status = 0, size - 1, 200
                requested = self.headers.get('Range')
                if requested:
                    try:
                        if not requested.startswith('bytes=') or ',' in requested:
                            raise ValueError('Only one byte range is supported')
                        first, last = requested[6:].split('-', 1)
                        if first:
                            start = int(first)
                            end = min(int(last), size - 1) if last else size - 1
                        else:
                            suffix = int(last)
                            if suffix <= 0: raise ValueError('Invalid suffix')
                            start = max(0, size - suffix)
                        if start < 0 or start >= size or end < start:
                            raise ValueError('Invalid range')
                        status = 206
                    except ValueError:
                        self.send_response(416)
                        self.send_header('Content-Range', f'bytes */{size}')
                        self.send_header('Content-Length', '0')
                        self.end_headers()
                        return
                length = max(0, end - start + 1)
                self.send_response(status)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(length))
                self.send_header('Accept-Ranges', 'bytes')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Content-Disposition', f'inline; filename="{document.name}"')
                if status == 206:
                    self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
                self.end_headers()
                if self.command == 'HEAD': return
                stream.seek(start)
                try:
                    while length:
                        block = stream.read(min(256 * 1024, length))
                        if not block: break
                        self.wfile.write(block)
                        length -= len(block)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass

        def do_GET(self):
            hosts=self.headers.get_all('Host',[])
            origins=self.headers.get_all('Origin',[])
            if len(hosts)!=1 or hosts[0].lower() not in ALLOWED_HOSTS:
                return self.send_error(403)
            if len(origins)>1 or (origins and origins[0] not in {'http://'+h for h in ALLOWED_HOSTS}):
                return self.send_error(403)
            url=urlsplit(self.path)
            if url.scheme or url.netloc or not self.path.startswith('/'):
                return self.send_error(400)
            if url.path in CURRENT_ARTIFACTS:
                return self.serve_artifact(*CURRENT_ARTIFACTS[url.path])
            if url.path in ('/paper/icra2027.pdf', '/paper/experiments.pdf'):
                document = PAPER_PATH if url.path == '/paper/icra2027.pdf' else EXPERIMENT_PATH
                try:
                    body = document.read_bytes()
                except OSError:
                    return self.send_error(404, 'Paper unavailable')
                self.send_response(200)
                self.send_header('Content-Type', 'application/pdf')
                self.send_header('Content-Disposition', f'inline; filename="{document.name}"')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                if self.command != 'HEAD':
                    try:
                        self.wfile.write(body)
                    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                        pass
                return
            if url.path not in ROUTES and not url.path.startswith('/mesh/'):
                return self.send_error(404)
            connection=HTTPConnection('127.0.0.1',backend_port,timeout=5)
            try:
                connection.request('GET',self.path,headers={'Host':f'127.0.0.1:{backend_port}'})
                response=connection.getresponse()
                body=response.read(16*1024*1024+1)
                if len(body)>16*1024*1024:return self.send_error(502)
                self.send_response(response.status)
                for name in ['Content-Type','Content-Security-Policy','Cross-Origin-Resource-Policy',
                             'X-Content-Type-Options']:
                    value=response.getheader(name)
                    if value:self.send_header(name,value)
                self.send_header('Content-Length',str(len(body)))
                self.send_header('Cache-Control','no-store')
                self.end_headers()
                if self.command != 'HEAD':
                    self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):
                pass
            except OSError:
                self.send_error(502,'Local viewer unavailable')
            finally:
                connection.close()
    return Handler

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8766)
    args=parser.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),make_handler())
    server.daemon_threads=True
    print(f'Tailnet read-only gateway on loopback:{args.port}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()

if __name__=='__main__':main()
