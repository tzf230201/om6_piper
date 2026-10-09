"""Offline D405 bag -> explicit CAD pose -> C++ diagnostic graph.

No hardware/control imports. This is not the deferred-evidence global ledger:
the recent buffer expires to unknown; the history buffer has no change policy.
SDK poses are unverified at exposure time. Never use outputs for robot control.
"""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time
import cv2
import numpy as np
import pyrealsense2 as rs

from workspace_paths import WORKSPACE_ROOT as ROOT
sys.path.insert(0, str(ROOT / 'piper_readonly'))
from camera_pose import camera_world_pose
from robot_model import load_model

DLL = ROOT / 'cpp_surface/build_robot_replay_20261007/Release/camera_local_api.dll'
PROFILE = ROOT / 'output/cad_optical_20261005_v1/fgng_config.cad_nominal.json'
DP = ct.POINTER(ct.c_double)
IP = ct.POINTER(ct.c_int)


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')


def summary(values):
    a = np.asarray(values, float)
    return dict(zip(('p50', 'p95', 'max'), np.quantile(a, [.5, .95, 1]).tolist())) if len(a) else None


class NativeGraph:
    def __init__(self, intr, ttl=.5):
        self.shape=(intr['height'],intr['width'])
        self.lib = ct.CDLL(str(DLL))
        self.lib.camera_local_error.restype = ct.c_char_p
        self.lib.camera_pose_create.argtypes = [ct.c_int, ct.c_int] + [ct.c_double]*4 + [ct.c_int, ct.c_double, ct.c_double]
        self.lib.camera_pose_create.restype = ct.c_void_p
        self.lib.camera_pose_update.argtypes = [ct.c_void_p, DP, ct.c_int, ct.c_int64, ct.c_double, DP, DP]
        self.lib.camera_local_copy.argtypes = [ct.c_void_p, DP, ct.c_int, IP, ct.c_int]
        self.lib.camera_local_destroy.argtypes = [ct.c_void_p]
        self.handle = self.lib.camera_pose_create(intr['width'], intr['height'], *[intr[k] for k in ('fx','fy','cx','cy')], 8, .01, ttl)
        if not self.handle:
            raise RuntimeError(self.lib.camera_local_error().decode())
        self.xyz = np.empty((40000, 3), np.float64)
        self.edges = np.empty((80000, 2), np.int32)
        self.stats = np.empty(3, np.float64)

    def update(self, depth, pose, frame_id, stamp):
        depth = np.ascontiguousarray(depth, dtype=np.float64)
        pose = np.ascontiguousarray(pose, dtype=np.float64)
        if depth.shape!=self.shape or pose.shape!=(4,4):
            raise ValueError('Expected exact depth HxW and pose 4x4 shapes')
        code = self.lib.camera_pose_update(self.handle, depth.ctypes.data_as(DP), depth.size, frame_id,
                                           stamp, pose.ctypes.data_as(DP), self.stats.ctypes.data_as(DP))
        if code:
            raise RuntimeError(self.lib.camera_local_error().decode())
        return self.stats.copy()

    def snapshot(self):
        if self.lib.camera_local_copy(self.handle, self.xyz.ctypes.data_as(DP), 40000,
                                      self.edges.ctypes.data_as(IP), 80000):
            raise RuntimeError(self.lib.camera_local_error().decode())
        p = self.xyz[:int(self.stats[0])].copy()
        e = self.edges[:int(self.stats[1])].copy()
        assert np.isfinite(p).all() and (not len(e) or (e.min() >= 0 and e.max() < len(p)))
        return p, e

    def close(self):
        if self.handle:
            self.lib.camera_local_destroy(self.handle)
            self.handle = None


def reprojection_residual(previous, current, old_pose, new_pose, intr):
    """Causal flat-patch depth residual; no ICP, optimization, or GT assumption."""
    y, x = np.mgrid[2:previous.shape[0]-2:8, 2:previous.shape[1]-2:8]
    z = previous[y, x]
    valid = np.isfinite(z) & (z >= .07) & (z <= 1.5)
    for dy, dx in ((-1,0),(1,0),(0,-1),(0,1)):
        zz = previous[y+dy, x+dx]
        valid &= (zz >= .07) & (zz <= 1.5) & (abs(zz-z) < .01)
    x, y, z = x[valid], y[valid], z[valid]
    p = np.column_stack(((x-intr['cx'])*z/intr['fx'], (y-intr['cy'])*z/intr['fy'], z))
    world = p @ old_pose[:3,:3].T + old_pose[:3,3]
    q = (world-new_pose[:3,3]) @ new_pose[:3,:3]
    good = (q[:,2] >= .07) & (q[:,2] <= 1.5)
    q = q[good]
    u = np.rint(intr['fx']*q[:,0]/q[:,2]+intr['cx']).astype(int)
    v = np.rint(intr['fy']*q[:,1]/q[:,2]+intr['cy']).astype(int)
    good = (u >= 1) & (u < current.shape[1]-1) & (v >= 1) & (v < current.shape[0]-1)
    u, v, q = u[good], v[good], q[good]
    observed = current[v,u]
    good = np.isfinite(observed) & (observed >= .07) & (observed <= 1.5)
    for dy, dx in ((-1,0),(1,0),(0,-1),(0,1)):
        zz = current[v+dy,u+dx]
        good &= (zz >= .07) & (zz <= 1.5) & (abs(zz-observed) < .01)
    return (observed-q[:,2])[good]


def run(source, out):
    source, out = Path(source).resolve(), Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out/'snapshots').mkdir()
    rows = [json.loads(line) for line in (source/'frames.jsonl').read_text().splitlines() if line.strip()]
    meta = json.loads((source/'metadata.json').read_text())
    intr = meta['streams']['depth']
    assert all(abs(c) < 1e-12 for c in intr['coeffs']), 'Rectification required'
    config = json.loads(PROFILE.read_text())
    model, _ = load_model()  # Existing viewer fallback; actual firmware unverified.
    poses = [camera_world_pose(model, row['joints_after']['q_rad'], config) for row in rows]
    before = [camera_world_pose(model, row['joints_before']['q_rad'], config) for row in rows]
    # Common display-world offset is applied exactly once by camera_world_pose.
    hashes = {p.name: sha(p) for p in source.iterdir() if p.name in ('rgbd.bag','rgb.mp4','frames.jsonl','metadata.json')}
    write(out/'protocol.json', dict(source=str(source), input_sha256=hashes, pose='Recorded joints_after + nominal CAD; NOT exposure synchronized',
        urdf=model['urdf'], firmware_verified=False, optical_profile=json.loads(PROFILE.read_text()),
        stride=8, cell_m=.01, near_m=.07, far_m=1.5, capacity_nodes=40000, capacity_edges=80000,
        recent_ttl_s=.5, history_ttl_s=180, snapshot_every=15,
        interpretation='Recent measured surface buffer and bounded history diagnostic, NOT full deferred ledger or planner map',
        residual='Previous saved depth warped into current saved depth; valid flat local patches, nearest-pixel sampling. Includes occlusion and calibration/timing error; not GT surface error.',
        rgb_display='Recorded rgb.mp4 frame at exact JSONL row index; bag playback may regroup RGB/depth',
        DLL_sha256=sha(DLL), script_sha256=sha(__file__), hardware_access=False))
    lookup = {}
    for i, row in enumerate(rows):
        lookup.setdefault(row['depth_number'], i)
    recent, history = NativeGraph(intr), NativeGraph(intr, 180)
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(str(source/'rgbd.bag'), False)
    recorded_rgb = cv2.VideoCapture(str(source/'rgb.mp4'))
    if not recorded_rgb.isOpened() or int(recorded_rgb.get(cv2.CAP_PROP_FRAME_COUNT)) != len(rows):
        raise RuntimeError('Recorded RGB video/log length mismatch')
    rgb_index = -1
    rgb_image = None
    regrouped = []
    started = False
    seen = set()
    trace, captures, probes = [], [], []
    residuals = []
    previous = None
    try:
        profile = pipe.start(cfg)
        started = True
        playback = profile.get_device().as_playback()
        playback.set_real_time(False)
        while True:
            try:
                frames = pipe.wait_for_frames(5000)
            except RuntimeError:
                if playback.current_status() == rs.playback_status.stopped:
                    break
                raise
            d, rgb = frames.get_depth_frame(), frames.get_color_frame()
            if not d or not rgb:
                continue
            number = d.get_frame_number()
            if number not in lookup or number in seen:
                continue
            i = lookup[number]
            # Bag and journal associations must agree, not just image dimensions.
            if abs(d.get_timestamp()-rows[i]['depth_timestamp_ms']) > .01:
                raise RuntimeError('Bag/log depth timestamp mismatch')
            if rgb.get_frame_number() != rows[i]['color_number']:
                regrouped.append(dict(log_index=i, depth_number=number,
                                      recorded_rgb_number=rows[i]['color_number'], replay_rgb_number=rgb.get_frame_number()))
            seen.add(number)
            stamp = (rows[i]['depth_timestamp_ms']-rows[0]['depth_timestamp_ms'])/1000
            raw = np.asanyarray(d.get_data()).copy()
            depth = raw.astype(np.float64)*meta['depth_scale']
            a = recent.update(depth, poses[i], number, stamp)
            b = history.update(depth, poses[i], number, stamp)
            # A 0.5 m camera-local probe exposes before/after pose disagreement.
            probe = np.array([0., 0., .5])
            shift = np.linalg.norm((poses[i][:3,:3]@probe+poses[i][:3,3])-(before[i][:3,:3]@probe+before[i][:3,3]))*1000
            trace.append(dict(log_index=i, depth_number=number, source_s=stamp, recent_nodes=int(a[0]), history_nodes=int(b[0]),
                              recent_native_ms=float(a[2]), history_native_ms=float(b[2]), bracket_probe_shift_mm=float(shift)))
            if (len(trace)-1) % 15 == 0 or number == max(lookup):
                while rgb_index < i:
                    ok, rgb_image = recorded_rgb.read()
                    if not ok:
                        raise RuntimeError('Recorded RGB video ended before logged frame')
                    rgb_index += 1
                p, e = recent.snapshot()
                hp, he = history.snapshot()
                stem = f'{len(captures):04d}'
                np.savez_compressed(out/'snapshots'/f'{stem}.npz', points=p, edges=e, history_points=hp, history_edges=he, pose=poses[i], depth=raw)
                assert cv2.imwrite(str(out/'snapshots'/f'{stem}.jpg'), rgb_image)
                record = dict(index=len(captures), stem=stem, source_s=stamp, log_index=i, depth_number=number,
                              rgb_offset_ms=rows[i]['color_timestamp_ms']-rows[i]['depth_timestamp_ms'],
                              recent_nodes=len(p), recent_edges=len(e), history_nodes=len(hp), history_edges=len(he))
                captures.append(record)
                if previous is not None:
                    old_depth, old_pose, old_stamp = previous
                    r = reprojection_residual(old_depth, depth, old_pose, poses[i], intr)
                    residuals.extend(r.tolist())
                    probes.append(dict(source_s=stamp, previous_s=old_stamp, samples=len(r),
                        absolute_mm=summary(abs(r)*1000), within_20mm=float(np.mean(abs(r) <= .02)) if len(r) else None))
                previous = (depth, poses[i], stamp)
            if len(trace) % 500 == 0:
                print(source.name, len(trace), 'frames', flush=True)
    finally:
        if started:
            pipe.stop()
        recent.close()
        history.close()
        recorded_rgb.release()
    assert seen == set(lookup), f'Incomplete playback: {len(seen)}/{len(lookup)}'
    result = dict(source=source.name, unique_frames=len(trace), source_duration_s=trace[-1]['source_s'],
                  recent_native_ms=summary([r['recent_native_ms'] for r in trace]),
                  history_native_ms=summary([r['history_native_ms'] for r in trace]),
                  bracket_probe_shift_mm=summary([r['bracket_probe_shift_mm'] for r in trace]),
                  reprojected_samples=len(residuals), residual_absolute_mm=summary(abs(np.asarray(residuals))*1000),
                  residual_within_20mm=float(np.mean(abs(np.asarray(residuals)) <= .02)),
                  final_recent_nodes=trace[-1]['recent_nodes'], final_history_nodes=trace[-1]['history_nodes'],
                  playback_rgb_regroupings=regrouped, rgb_display_uses_recorded_log_pair=True,
                  scope='Offline C++ component timing with capture overhead outside timer; NOT end-to-end live latency or accuracy',
                  trace=trace, captures=captures, probes=probes, input_sha256=hashes)
    write(out/'result.json', result)
    shutil.copy2(__file__, out/'replay_robot_mapping.py')
    print(json.dumps({k:v for k,v in result.items() if k not in ('trace','captures','probes','input_sha256')}, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.source, args.output)
