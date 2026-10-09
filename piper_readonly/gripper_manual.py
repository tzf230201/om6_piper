"""One-cycle gripper helper. Default is OFFLINE; no hardware imports/connect.

--viewer reads the existing HTTP viewer for a dry-run plan only.
--execute is interlocked; it is NOT commissioned on the current robot.
Never hand-edit commissioning_interlock.json to release this program.
Windows/USB timing and the SDK force parameter are not safety-certified.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import struct
import time
from urllib.request import urlopen

from app_paths import LEGACY_ROOT

ROOT = LEGACY_ROOT  # One existing gate; never manufacture a new release.
GATE = ROOT / 'commissioning_interlock.json'
MAX_DELTA_MM = 1.0
MAX_FORCE_PARAMETER = 1.0
DEADLINE_S = .15


def plan_cycle(start_mm, delta_mm=1.0, force=1.0):
    values = (start_mm, delta_mm, force)
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('Finite numeric arguments required')
    if not 0 < delta_mm <= MAX_DELTA_MM or not .1 <= force <= MAX_FORCE_PARAMETER:
        raise ValueError('Local test limits: 0 < delta <= 1 mm; .1 <= SDK force <= 1')
    start_um = round(start_mm * 1000)
    delta_um = round(delta_mm * 1000)
    if delta_um < 1 or not 1000 <= start_um < start_um + delta_um <= 60000:
        raise ValueError('Local width window: 1..60 mm; this is not a measured hardware limit')
    force_raw = round(force * 1000)
    # SDK width-mode layout: signed int32 um, int16 force, enable flag, no zeroing.
    return [(0x159, struct.pack('>ihBB', width, effort, enabled, 0))
            for width, effort, enabled in (
                (start_um + delta_um, force_raw, 1),
                (start_um, force_raw, 1),
                (start_um, 0, 0))]


def check_release(path=GATE):
    gate = json.loads(path.read_text(encoding='utf-8'))
    if gate.get('control_tests_released') is not True:
        raise PermissionError('Hardware locked: ' + gate.get('reason', 'commissioning incomplete'))
    review = gate.get('gripper_manual_test') or {}
    # No CLI override, no reuse of the arm's consumed enable-stop permit.
    if (review.get('allowed') is not True or
        review.get('physical_gripper_stop_verified') is not True or
        review.get('procedure_validated') is not True):
        raise PermissionError('Separate reviewed gripper commissioning is missing')
    age = (datetime.now().astimezone() - datetime.fromisoformat(review['reviewed_at'])).total_seconds()
    digest = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    if not 0 <= age <= 3600 or review.get('program_sha256') != digest:
        raise PermissionError('Gripper review expired or program changed')
    if not review.get('adapter_serial') or not review.get('firmware_version'):
        raise PermissionError('Reviewed unit/firmware identity missing')
    return review


def gripper_sample(snapshot, *, expected_enabled=None):
    age = snapshot.get('rx_age_s', {}).get('0x2A8')
    if type(age) not in (float, int) or not math.isfinite(age) or not 0 <= age <= DEADLINE_S:
        raise RuntimeError('Gripper feedback stale/missing')
    raw = bytes.fromhex(snapshot.get('raw_latest', {}).get('0x2A8', ''))
    if len(raw) != 8:
        raise RuntimeError('Invalid gripper feedback payload')
    status = raw[6]
    if status & 0xBF:
        raise RuntimeError('Gripper fault, sensor or homing flag; no reset/clear attempted')
    enabled = bool(status & 0x40)
    if expected_enabled is not None and enabled != expected_enabled:
        raise RuntimeError('Unexpected gripper enable state')
    if (snapshot.get('gripper') or {}).get('mode') != 'width':
        raise RuntimeError('Width mode required; no mode conversion')
    counts = (snapshot.get('transport_health') or {}).get('counts', {})
    faults = ('usb_errors', 'malformed_frames', 'adapter_overflow_reports',
              'can_error_frames', 'hardware_clock_discontinuities',
              'host_queue_overflows', 'rx_pump_errors')
    if any(counts.get(k, 0) for k in faults):
        raise RuntimeError('Transport fault recorded; no control retry')
    width_mm = int.from_bytes(raw[:4], 'big', signed=True) / 1000
    return width_mm, enabled, snapshot.get('rx_ids', {}).get('0x2A8', 0)


def wait_observed(reader, target_mm, enabled, previous_count, timeout=2.0):
    until = time.monotonic() + timeout
    consecutive = 0
    while time.monotonic() < until:
        width, actual_enabled, count = gripper_sample(reader.snapshot())
        if abs(width - target_mm) > 2:
            raise RuntimeError('Gripper moved outside local test envelope')
        if count > previous_count:
            consecutive = consecutive + 1 if actual_enabled == enabled and abs(width-target_mm) <= .1 else 0
            previous_count = count
            if consecutive >= 3:
                return count
        time.sleep(.02)
    raise RuntimeError('Expected feedback not observed; no retry')


def execute_cycle(delta_mm, force):
    review = check_release()  # BEFORE hardware imports, USB connect or prompt.
    if input('Gripper empty, hands clear, operator at physical stop? Type GRIPPER SAFE: ').strip() != 'GRIPPER SAFE':
        raise PermissionError('Operator cancelled')
    # Imports below are reachable only after a separately reviewed release.
    from read_robot import Reader, plain
    from commission_piper import bounded_usb_write
    reader = None
    active_attempted = False
    disable_attempted = False
    packets = None
    try:
        reader = Reader(low_latency=True, buffered_rx=False)
        serial = reader.bus.gs_usb.gs_usb.serial_number
        firmware = reader.robot.get_firmware(timeout=2, min_interval=3)
        identity = plain(firmware.msg) if firmware is not None and hasattr(firmware, 'msg') else plain(firmware)
        if serial != review['adapter_serial'] or not isinstance(identity, dict) or identity.get('software_version') != review['firmware_version']:
            raise RuntimeError('Same-session unit/firmware identity not verified')
        time.sleep(.3)
        start, _, count = gripper_sample(reader.snapshot(), expected_enabled=False)
        packets = plan_cycle(start, delta_mm, force)
        print(f'Width {start:.3f} -> {start + delta_mm:.3f} -> {start:.3f} mm; one cycle only')
        for index, (ident, payload) in enumerate(packets):
            check_release()
            width, enabled, count = gripper_sample(reader.snapshot(), expected_enabled=index > 0)
            if not start-.2 <= width <= start+delta_mm+.2:
                raise RuntimeError('Width changed outside approved window')
            if index < 2:
                active_attempted = True  # Ambiguous USB failure still triggers one disable attempt.
            else:
                disable_attempted = True
            bounded_usb_write(reader.bus.gs_usb, ident, payload, echo_id=index+1)
            target_mm = int.from_bytes(payload[:4], 'big', signed=True)/1000
            count = wait_observed(reader, target_mm, index < 2, count)
        print('Width and disabled-state feedback observed; NOT safety-certified physical stop proof.')
    finally:
        if reader:
            try:
                if active_attempted and not disable_attempted:
                    disable_attempted = True
                    # Empty-gripper fallback only. Never send an arm stop, reset or enable.
                    ident, payload = packets[-1]
                    try:
                        bounded_usb_write(reader.bus.gs_usb, ident, payload, echo_id=4)
                        _, _, count = gripper_sample(reader.snapshot())
                        wait_observed(reader, start, False, count)
                    except Exception as exc:
                        print(f'GRIPPER STOP UNCONFIRMED: {exc}. Use physical emergency stop if unsafe.')
            finally:
                reader.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start-mm', type=float, default=39.06, help='OFFLINE example only; execute always reads actual width')
    parser.add_argument('--delta-mm', type=float, default=1.0)
    parser.add_argument('--force', type=float, default=1.0, help='SDK parameter, not a verified physical force limit')
    parser.add_argument('--viewer', action='store_true', help='Read current width through existing local viewer, no USB/CAN open')
    parser.add_argument('--execute', action='store_true', help='Requires existing interlock AND separately validated gripper release; currently blocked')
    args = parser.parse_args(argv)
    try:
        # Validate CLI values even if hardware gate is closed.
        plan_cycle(args.start_mm, args.delta_mm, args.force)
        if args.execute:
            if args.viewer:
                raise ValueError('--viewer is read-only planning, not an execution transport')
            execute_cycle(args.delta_mm, args.force)
            return 0
        start = args.start_mm
        if args.viewer:
            with urlopen('http://127.0.0.1:8765/api/state', timeout=3) as response:
                view = json.load(response)
            if view.get('live') is not True or view.get('source') != 'hardware':
                raise RuntimeError('Viewer is not live hardware')
            start, _, _ = gripper_sample(view['data'], expected_enabled=False)
        packets = plan_cycle(start, args.delta_mm, args.force)
        print('DRY RUN ONLY: no USB opened, no motor command sent.')
        for name, (ident, payload) in zip(('OPEN 1 SMALL STEP', 'RETURN', 'REQUEST DISABLE'), packets):
            print(f'{name}: width={int.from_bytes(payload[:4], "big", signed=True)/1000:.3f} mm, CAN=0x{ident:X}, payload={payload.hex()}')
        print('Real execution remains locked pending separate gripper commissioning; do not edit the interlock to bypass it.')
        return 0
    except (PermissionError, RuntimeError, ValueError, OSError, KeyError) as exc:
        print(f'NOT EXECUTED / CHECK FAILED: {exc}')
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
