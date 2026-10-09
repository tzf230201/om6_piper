"""Print six Piper joint angles at a nominal 50 Hz. READ ONLY.

Default source: existing local viewer (no USB/CAN connection).
--source can: direct passive pyAgxArm reader; requires exclusive CAN access.
Viewer snapshots may update slower than 50 Hz: repeated samples are labelled
CACHED. This program never claims that printing creates new measurements.
"""
import argparse
import math
from pathlib import Path
import sys
import time
import json
from urllib.request import urlopen

PERIOD_S = 1 / 50
JOINT_IDS = ('0x2A5', '0x2A6', '0x2A7')
MAX_AGE_S = .15  # Local diagnostic threshold, not a manufacturer guarantee.


def joint_values(snapshot, extra_age=0):
    control = snapshot.get('frozen_control') or {}
    values = control.get('joint_millidegrees')
    if control.get('valid_payloads') is not True or not isinstance(values, list) or len(values) != 6:
        raise ValueError('Joint feedback missing/invalid')
    if any(type(v) is not int for v in values):
        raise ValueError('Invalid joint values')
    ages = snapshot.get('rx_age_s', {})
    if type(extra_age) not in (int, float) or not math.isfinite(extra_age) or extra_age < 0:
        raise ValueError('Invalid source age')
    for ident in JOINT_IDS:
        age = ages.get(ident)
        if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age + extra_age <= MAX_AGE_S:
            raise ValueError('Joint feedback stale/missing')
    counts = snapshot.get('rx_ids', {})
    token = tuple(counts.get(ident) for ident in JOINT_IDS)
    if any(type(count) is not int or count < 0 for count in token):
        raise ValueError('Invalid receive counters')
    return [v / 1000 for v in values], token


class ViewerSource:
    def __init__(self, url):
        self.url = url.rstrip('/') + '/api/state'

    def read(self):
        # HTTP latency counts toward the source's reported sample age.
        started = time.perf_counter()
        with urlopen(self.url, timeout=.5) as response:
            view = json.load(response)
        if view.get('application') != 'piper-readonly-viewer' or view.get('source') != 'hardware' or view.get('live') is not True:
            raise ValueError('Viewer does not have live hardware feedback')
        return joint_values(view.get('data') or {},
                            view.get('sample_age_s', -1) + time.perf_counter() - started)

    def close(self):
        pass


class CanSource:
    def __init__(self):
        # Existing implementation blocks ALL application CAN TX, even queries.
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'piper_readonly'))
        from read_robot import Reader
        self.reader = Reader(low_latency=True, buffered_rx=False, passive=True)

    def read(self):
        return joint_values(self.reader.snapshot())

    def close(self):
        self.reader.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=('viewer', 'can'), default='viewer')
    parser.add_argument('--url', default='http://127.0.0.1:8765', help='Existing viewer base URL')
    parser.add_argument('--unit', choices=('deg', 'rad'), default='deg')
    parser.add_argument('--seconds', type=float, default=0, help='0: until Ctrl+C; positive: bounded run')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('--seconds must be finite and nonnegative')
    source = None
    timing = None
    start = time.perf_counter()
    next_tick = start
    lines = new_samples = missed_ticks = 0
    previous = None
    try:
        # Scoped 1 ms Windows timer; restores its setting on exit. No USB import.
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'piper_readonly'))
        from feedback_timing import ReceiverTiming
        timing = ReceiverTiming(enabled=True)
        timing.start()
        source = CanSource() if args.source == 'can' else ViewerSource(args.url)
        start = next_tick = time.perf_counter()
        print(f'READ ONLY | source={args.source} | nominal print rate=50 Hz | unit={args.unit}', flush=True)
        if args.source == 'viewer':
            print('CACHED = repeated viewer sample; print rate is not sensor acquisition rate.', flush=True)
        while args.seconds == 0 or time.perf_counter() - start < args.seconds:
            wait = next_tick - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            if args.seconds > 0 and time.perf_counter() - start >= args.seconds:
                break
            try:
                angles, token = source.read()
                if previous is not None and any(a < b for a, b in zip(token, previous)):
                    raise ValueError('Receive counters restarted; reconnect explicitly')
                changed = token != previous
                new_samples += int(changed)
                previous = token
                if args.unit == 'rad':
                    angles = [math.radians(v) for v in angles]
                text = ' '.join(f'J{i}={v:9.4f}' for i, v in enumerate(angles, 1))
                print(f'{time.perf_counter()-start:9.3f}s {"NEW   " if changed else "CACHED"} {text}', flush=True)
            except (OSError, ValueError, TypeError, KeyError) as exc:
                # Never present an old angle or artificial zero as fresh data.
                print(f'{time.perf_counter()-start:9.3f}s UNAVAILABLE: {exc}', flush=True)
            lines += 1
            next_tick += PERIOD_S
            now = time.perf_counter()
            if next_tick < now:
                skipped = int((now - next_tick) / PERIOD_S) + 1
                missed_ticks += skipped
                next_tick += skipped * PERIOD_S  # No catch-up output bursts.
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Cannot start reader: {exc}. CAN mode needs exclusive adapter access; do not run beside another CAN reader.', file=sys.stderr)
        return 2
    finally:
        elapsed = time.perf_counter() - start
        try:
            if source:
                source.close()
        finally:
            if timing:
                timing.close()
        if lines:
            print(f'Summary: {lines} lines, {new_samples} changed samples, actual output={lines/elapsed:.2f} Hz, missed ticks={missed_ticks}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
