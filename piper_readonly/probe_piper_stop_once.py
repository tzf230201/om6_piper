"""One electronic-stop probe from disabled Standby; no enable or target.

An acknowledgement here only tests the disabled controller. It is not proof
of braking an enabled/moving arm and never releases the motion interlock.
User's mechanical support/emergency availability confirmation is required.
One firmware query, one exact documented stop at most, then receive-only.
"""
import argparse
import json
from pathlib import Path
import threading
import time

import can
import numpy as np
from pyAgxArm import AgxArmFactory, create_agx_arm_config

from clear_piper_fault_once import write_json
from commission_piper import STOP, bounded_usb_write
from feedback_health import describe_feedback
from feedback_timing import ReceiverTiming
from read_robot import Reader
from readonly_bus import QueuedReadOnlyGsUsbBus, is_firmware_query
from reset_piper_disabled_once import ADAPTER_SERIAL, maintenance_reasons


def is_exact_stop(msg):
    return (msg.arbitration_id == STOP[0] and msg.dlc == 8 and bytes(msg.data) == STOP[1]
        and not any((msg.is_extended_id, msg.is_remote_frame, msg.is_error_frame, msg.is_fd)))


def stop_preflight_reasons(snapshot, reference=None):
    reasons = maintenance_reasons(snapshot, reference)
    if (snapshot.get('frozen_control') or {}).get('driver_codes') != [0]*6:
        reasons.append('Stop probe requires all six drivers disabled and fault-free')
    return reasons


class StopOnlyPermit:
    def __init__(self):
        self.attempted = False
        self.snapshot = self.reference = None
        self.deadline = 0

    def arm(self, snapshot, reference, *, operator_supported):
        if operator_supported is not True or self.attempted or self.snapshot is not None:
            raise PermissionError('Supported arm and unused stop permit required')
        reasons = stop_preflight_reasons(snapshot(), reference)
        if reasons: raise PermissionError('; '.join(reasons))
        self.snapshot = snapshot
        self.reference = np.asarray(reference, dtype=float).copy()
        self.deadline = time.monotonic() + 2

    def consume(self, msg):
        if not is_exact_stop(msg): raise PermissionError('Only exact electronic stop allowed')
        if self.attempted or self.snapshot is None or time.monotonic() > self.deadline:
            raise PermissionError('Stop permit absent, expired or consumed')
        self.attempted = True
        self.deadline = 0
        reasons = stop_preflight_reasons(self.snapshot(), self.reference)
        if reasons: raise PermissionError('; '.join(reasons))


class StopOnlyBus(QueuedReadOnlyGsUsbBus):
    def __init__(self, *args, **kwargs):
        self.permit = StopOnlyPermit()
        self.tx_closed = False
        self.tx_events = []
        self.tx_lock = threading.Lock()
        super().__init__(*args, **kwargs)

    def close_transmit(self):
        with self.tx_lock: self.tx_closed = True

    def send(self, msg, timeout=None):
        with self.tx_lock:
            try:
                if self.tx_closed: raise PermissionError('Receive-only latch closed')
                if is_firmware_query(msg):
                    if self.queries_sent or self.permit.snapshot is not None:
                        raise PermissionError('One preflight identity query only')
                    self.queries_sent += 1
                elif is_exact_stop(msg):
                    try: self.permit.consume(msg)
                    finally: self.tx_closed = True
                else: raise PermissionError('Stop-only probe blocks all other commands')
            except BaseException:
                self.blocked_commands += 1
                raise
            event = dict(id=f'0x{msg.arbitration_id:03X}', data=bytes(msg.data).hex(),
                started_perf_s=time.perf_counter(), usb_write_completed=False)
            self.tx_events.append(event)
            try:
                event.update(bounded_usb_write(self.gs_usb, msg.arbitration_id,
                    bytes(msg.data), echo_id=len(self.tx_events)))
                event.update(usb_write_completed=True, completed_perf_s=time.perf_counter())
            except BaseException as exc:
                self.tx_closed = True
                event['error'] = str(exc)
                raise


class StopOnlyReader(Reader):
    def __init__(self):
        can.interfaces.BACKENDS['gs_usb'] = ('probe_piper_stop_once', 'StopOnlyBus')
        self.channel, self.firmware_driver, self.firmware = 0, 'default', None
        self._receive_timing = ReceiverTiming(enabled=True)
        self.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper', firmeware_version='default', interface='gs_usb', channel=0,
            bitrate=1000000, enable_check_can=False, auto_connect=False, timeout=.02))
        try:
            self._receive_timing.start()
            self.robot.connect()
            comm = self.robot.get_context().get_comm()
            self.bus = comm.recv_bus
            if comm.send_bus is not self.bus or self.bus.gs_usb.serial_number != ADAPTER_SERIAL:
                raise RuntimeError('Unexpected shared bus or adapter identity')
            self.gripper = self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        except BaseException:
            self.close()
            raise

    def query_firmware(self):
        self.firmware = self.robot.get_firmware(timeout=2, min_interval=3)
        return self.firmware


def verify_disabled_stop(samples, completed_at, reference):
    if completed_at is None or len(samples) < 20: return False
    tail = samples[-20:]
    if tail[-1]['snapshot_perf_s'] - tail[0]['snapshot_perf_s'] < .5: return False
    previous_counts = None
    for s in tail:
        health = describe_feedback(s)
        if any(not v['fresh'] for v in health['groups'].values()) or health['transport_faults_since_connect']:
            return False
        c = s.get('frozen_control') or {}
        if (not c.get('valid_payloads') or c.get('arm_status') != 1 or c.get('err_code') != 0
            or c.get('driver_codes') != [0]*6 or c.get('gripper_status_code') != 0
            or c.get('motion_status') != 0 or c.get('teach_status') != 0):
            return False
        if any(s['snapshot_perf_s']-s['rx_age_s'][ident] <= completed_at for ident in (
                '0x2A1', '0x2A5', '0x2A6', '0x2A7')): return False
        counts = s.get('rx_ids', {})
        if previous_counts is not None and any(counts.get(ident, 0) <= previous_counts.get(ident, 0)
                for ident in ('0x2A1', '0x2A5', '0x2A6', '0x2A7')): return False
        previous_counts = counts
        q = np.asarray(c.get('joint_millidegrees', []), dtype=float)/1000
        if q.shape != (6,) or not np.isfinite(q).all() or np.max(np.abs(q-reference)) > .1:
            return False
    return True


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute-once', action='store_true')
    p.add_argument('--operator-supported', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.execute_once and not args.operator_supported:
        p.error('Supported-arm confirmation required')
    args.output.mkdir(parents=True, exist_ok=False)
    reader = None
    result = dict(execution_allowed=False, enabled_stop_verified=False,
        motion_readiness_verified=False, stop_api_called=False, disabled_stop_ack_observed=False)
    try:
        reader = StopOnlyReader()
        time.sleep(.8)
        reader.query_firmware()
        before = reader.snapshot()
        write_json(args.output/'before.json', before)
        reasons = stop_preflight_reasons(before)
        if reasons: raise RuntimeError('Preflight rejected: ' + '; '.join(reasons))
        reference_rad = np.asarray(before['joint_angles_rad'], dtype=float)
        pre = []
        for _ in range(20):
            time.sleep(.05)
            s = reader.snapshot()
            pre.append(s)
            reasons = stop_preflight_reasons(s, reference_rad)
            if reasons: raise RuntimeError('Stationary preflight rejected: ' + '; '.join(reasons))
        write_json(args.output/'pre_observations.json', pre)
        if not args.execute_once:
            result['outcome'] = 'preflight_only_no_stop'
        else:
            reader.bus.permit.arm(reader.snapshot, reference_rad,
                                  operator_supported=args.operator_supported)
            write_json(args.output/'intent.json', dict(api='electronic_emergency_stop()',
                can_id='0x150', data=STOP[1].hex(), max_attempts=1,
                operator_supported=True, motor_enable_allowed=False, motion_targets_allowed=False))
            result['stop_api_called'] = True
            try: reader.robot.electronic_emergency_stop()
            except Exception as exc: result['stop_write_error'] = str(exc)
            finally: reader.bus.close_transmit()
            observations = []
            end = time.perf_counter()+12
            try:
                while time.perf_counter() < end:
                    time.sleep(.05)
                    observations.append(reader.snapshot())
            finally:
                write_json(args.output/'post_stop_observations.json', observations)
                if observations: write_json(args.output/'after.json', observations[-1])
            event = next((e for e in reader.bus.tx_events if e['id']=='0x150'), {})
            result['disabled_stop_ack_observed'] = verify_disabled_stop(
                observations, event.get('completed_perf_s'), np.rad2deg(reference_rad))
            raw = observations[-1].get('frozen_control') or {}
            result['final_control'] = raw
            result['final_transport'] = observations[-1].get('transport_health')
            result['outcome'] = ('stop_delivery_ambiguous_no_retry' if result.get('stop_write_error') else
                'disabled_stop_ack_observed_no_motion_release' if result['disabled_stop_ack_observed'] else
                'disabled_stop_unconfirmed_no_retry')
    except BaseException as exc:
        result.update(outcome='aborted_no_further_write', error=f'{type(exc).__name__}: {exc}')
    finally:
        if reader:
            reader.bus.close_transmit()
            result.update(tx_events=reader.bus.tx_events, stop_permit_consumed=reader.bus.permit.attempted,
                          firmware_queries_attempted=reader.bus.queries_sent)
            try: reader.close()
            except Exception as exc: result['close_error'] = str(exc)
        write_json(args.output/'result.json', result)
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result.get('outcome') in ('preflight_only_no_stop',
        'disabled_stop_ack_observed_no_motion_release') else 2


if __name__ == '__main__': raise SystemExit(main())
