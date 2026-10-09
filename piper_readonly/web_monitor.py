"""Loopback-only RViz-style Piper monitor. No command/control HTTP endpoints."""
import argparse
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit
from urllib.request import urlopen
import webbrowser
from robot_model import load_model, ROOT
from app_paths import APP_ROOT

def envelope(data, source, source_age, error=None):
    age = max(0., source_age)
    # Reader.snapshot already checks the arrival time of each CAN fragment.
    # Also expire the whole sample when its publishing process stops.
    frame_ages = (data or {}).get('rx_age_s', {})
    joint_age = max(frame_ages.get(k, float('inf')) for k in ('0x2A5', '0x2A6', '0x2A7'))
    fresh = bool(data and data.get('all_joint_feedback_fresh') and age + joint_age < .5 and not error)
    return {'application': 'piper-readonly-viewer', 'data': data, 'source': source, 'sample_age_s': age,
            'live': fresh and source == 'hardware', 'snapshot': source == 'snapshot',
            'error': error, 'server_time': datetime.now().astimezone().isoformat()}

class State:
    def __init__(self, reader=None, snapshot=None):
        self.reader, self.data = reader, snapshot
        self.lock = threading.Lock()
        self.updated = time.monotonic()
        self.error = None
        self.stop = threading.Event()

    def collect(self):
        while not self.stop.is_set():
            try:
                data = self.reader.snapshot()
                with self.lock:
                    self.data, self.updated, self.error = data, time.monotonic(), None
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
            self.stop.wait(.05)

    def response(self):
        with self.lock:
            return envelope(self.data, 'hardware' if self.reader else 'snapshot',
                            time.monotonic() - self.updated, self.error)

def make_handler(state, model, mesh_assets, port, fgng=None):
    assets = {**mesh_assets, '/': APP_ROOT/'web/index.html',
        '/style.css': APP_ROOT/'web/style.css', '/viewer.bundle.js': APP_ROOT/'web/viewer.bundle.js',
        '/viewer.bundle.js.LEGAL.txt': APP_ROOT/'web/viewer.bundle.js.LEGAL.txt'}
    allowed_hosts = {f'127.0.0.1:{port}', f'localhost:{port}'}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            if args and str(args[0]).startswith(('GET /api/state','GET /api/fgng')):
                return
            super().log_message(fmt, *args)

        def do_GET(self):
            # Deny DNS rebinding and cross-origin browser reads of robot telemetry.
            if self.headers.get('Host', '') not in allowed_hosts:
                return self.send_error(403)
            request_origin = self.headers.get('Origin')
            if request_origin and request_origin not in {'http://' + h for h in allowed_hosts}:
                return self.send_error(403)
            path = urlsplit(self.path).path
            if path == '/api/state':
                body = json.dumps(state.response(), allow_nan=False).encode()
                mime = 'application/json'
            elif path == '/api/model':
                body = json.dumps(model).encode()
                mime = 'application/json'
            elif path == '/api/fgng':
                body = json.dumps(fgng.response() if fgng else {
                    'status':'disabled','live':False,'message':'F-GNG tidak aktif',
                    'nodes':[],'edges':[],'support':[]}, allow_nan=False).encode()
                mime = 'application/json'
            elif path in assets and assets[path].is_file():
                body = assets[path].read_bytes()
                mime = 'text/javascript' if path.endswith('.js') else (mimetypes.guess_type(str(assets[path]))[0] or 'application/octet-stream')
            else:
                return self.send_error(404)
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Cross-Origin-Resource-Policy', 'same-origin')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--snapshot', type=Path, help='Offline inspection only; clearly marked, never live')
    parser.add_argument('--open', action='store_true', help='Open local viewer in default browser')
    parser.add_argument('--no-fgng', action='store_true', help='Do not open RealSense or run wrist F-GNG')
    parser.add_argument('--low-latency', action='store_true', help='Process-local 1 ms receive scheduling; no freshness threshold change')
    args = parser.parse_args()
    if args.open and not args.snapshot:
        try:
            with urlopen(f'http://127.0.0.1:{args.port}/api/state', timeout=1) as response:
                existing = json.load(response)
            if existing.get('application') == 'piper-readonly-viewer':
                webbrowser.open(f'http://127.0.0.1:{args.port}')
                print('Opened existing Piper monitor; no second CAN connection.')
                return
        except (OSError, ValueError):
            pass
    # Reserve the port before opening USB so a second launcher cannot take CAN.
    server = ThreadingHTTPServer(('127.0.0.1', args.port), BaseHTTPRequestHandler)
    server.daemon_threads = True
    reader, state, collector, fgng = None, None, None, None
    try:
        if args.snapshot:
            snapshot = json.loads(args.snapshot.read_text(encoding='utf-8'))
        else:
            from read_robot import Reader
            reader = Reader(low_latency=args.low_latency)
            reader.query_firmware()
            snapshot = reader.snapshot()
        state = State(reader, snapshot)
        firmware = (snapshot.get('firmware') or {}).get('software_version')
        model, assets = load_model(firmware)
        if reader and not args.no_fgng:
            from wrist_fgng import WristFGNG
            fgng = WristFGNG(state, model)
        server.RequestHandlerClass = make_handler(state, model, assets, args.port, fgng)
        stop_file = ROOT/'output/web_monitor.stop'
        # Never remove a pre-existing shutdown request silently.
        if stop_file.exists():
            raise RuntimeError(f'Remove {stop_file} before restarting the monitor')
        if reader:
            collector = threading.Thread(target=state.collect, daemon=True)
            collector.start()
        if fgng:
            fgng.start()
        def watch_stop():
            while not state.stop.wait(.25):
                if stop_file.exists():
                    server.shutdown()
                    return
        threading.Thread(target=watch_stop, daemon=True).start()
        url = f'http://127.0.0.1:{args.port}'
        print(f'Piper READ ONLY viewer: {url} | {model["urdf"]}', flush=True)
        if args.open:
            webbrowser.open(url)
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if fgng and fgng.thread.is_alive():
            fgng.close()
        if state:
            state.stop.set()
        if collector:
            collector.join(timeout=2)
        server.server_close()
        if reader:
            reader.close()

if __name__ == '__main__':
    main()
