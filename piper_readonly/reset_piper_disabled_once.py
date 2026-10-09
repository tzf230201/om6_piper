"""Explicitly authorized reset-only maintenance; never enable or command motion.

Reset cuts motor power and may cause a supported arm to settle. This is NOT a
non-moving or safety-certified procedure. Requires mechanically supported,
fresh, stationary, disabled Standby and this unit's same-session identity.
The legacy Teaching recovery tool and the motion interlock are unchanged.
One firmware query, at most one reset, then irrevocably receive-only. No clear,
mode switch, retry, zeroing, protection change, enable or target is permitted.
"""
import argparse
import json
from pathlib import Path
import threading
import time

import can
import numpy as np
from pyAgxArm import AgxArmFactory, create_agx_arm_config

from clear_piper_fault_once import disabled_idle_reasons, write_json
from commission_piper import bounded_usb_write
from feedback_timing import ReceiverTiming
from read_robot import Reader
from readonly_bus import QueuedReadOnlyGsUsbBus, is_firmware_query
from recover_piper_state_once import is_exact_reset, RESET_BYTES

ADAPTER_SERIAL = '0033001A5246571120393733'
IDENTITY = dict(hardware_version='H-V1.2-1', software_version='S-V1.5-7',
                node_type='ARM_MC', production_date='250108', node_number='15',
                motor_ratio_and_batch='10')


def maintenance_reasons(snapshot, reference=None):
    reasons = disabled_idle_reasons(snapshot, reference)
    if any((snapshot.get('firmware') or {}).get(k) != v for k, v in IDENTITY.items()):
        reasons.append('Same-session unit identity mismatch')
    raw = snapshot.get('frozen_control') or {}
    if not raw.get('valid_payloads'):
        reasons.append('Missing frozen control payloads')
        return reasons
    for key in ('ctrl_mode', 'arm_status', 'teach_status', 'motion_status',
                'trajectory_num', 'err_code', 'gripper_status_code'):
        if type(raw.get(key)) is not int or raw[key] != 0:
            reasons.append(f'Frozen control is not disabled idle Standby: {key}')
    codes = raw.get('driver_codes', [])
    if len(codes) != 6 or any(type(c) is not int or c not in (0, 16, 32, 48) for c in codes):
        reasons.append('Frozen driver states enabled, invalid, or additional fault')
    if codes != [(d or {}).get('foc_status_code') for d in snapshot.get('drivers', [])]:
        reasons.append('Frozen and SDK driver state disagree')
    q = np.asarray(raw.get('joint_millidegrees', []), dtype=float) / 1000
    if q.shape != (6,) or not np.isfinite(q).all():
        reasons.append('Invalid frozen joint positions')
    elif reference is not None and np.max(np.abs(q - np.rad2deg(reference))) > .2:
        reasons.append('Frozen joint movement exceeds 0.2 degree allowance')
    return reasons


class StandbyResetPermit:
    """New, reset-only maintenance scope; not an override for motion control."""
    def __init__(self):
        self.attempted = False
        self.snapshot = None
        self.reference = None
        self.deadline = 0

    def arm(self, snapshot, reference, *, operator_supported):
        if operator_supported is not True or self.attempted or self.snapshot is not None:
            raise PermissionError('Explicit support confirmation and unused permit required')
        reasons = maintenance_reasons(snapshot(), reference)
        if reasons:
            raise PermissionError('; '.join(reasons))
        self.snapshot = snapshot
        self.reference = np.asarray(reference, dtype=float).copy()
        self.deadline = time.monotonic() + 2

    def consume(self, msg):
        if not is_exact_reset(msg):
            raise PermissionError('Only the exact documented reset packet is allowed')
        if self.attempted or self.snapshot is None or time.monotonic() > self.deadline:
            raise PermissionError('Reset permit absent, expired or already consumed')
        self.attempted = True  # Consumed even if recheck or native write fails.
        self.deadline = 0
        reasons = maintenance_reasons(self.snapshot(), self.reference)
        if reasons:
            raise PermissionError('; '.join(reasons))


class ResetOnlyBus(QueuedReadOnlyGsUsbBus):
    def __init__(self, *args, **kwargs):
        self.permit = StandbyResetPermit()
        self.tx_closed = False
        self.tx_lock = threading.Lock()
        self.tx_events = []
        super().__init__(*args, **kwargs)

    def close_transmit(self):
        with self.tx_lock:
            self.tx_closed = True

    def send(self, msg, timeout=None):
        with self.tx_lock:
            try:
                if self.tx_closed:
                    raise PermissionError('Receive-only latch closed; no further writes')
                if is_firmware_query(msg):
                    if self.queries_sent or self.permit.snapshot is not None:
                        raise PermissionError('Only one pre-reset identity query; no retry')
                    self.queries_sent += 1
                elif is_exact_reset(msg):
                    try:
                        self.permit.consume(msg)
                    finally:
                        self.tx_closed = True
                else:
                    raise PermissionError('Reset-only maintenance rejects this command')
            except BaseException:
                self.blocked_commands += 1
                raise
            event = dict(id=f'0x{msg.arbitration_id:03X}', data=bytes(msg.data).hex(),
                         started_perf_s=time.perf_counter(), usb_write_completed=False)
            self.tx_events.append(event)
            try:
                event.update(bounded_usb_write(self.gs_usb, msg.arbitration_id, bytes(msg.data),
                                               echo_id=len(self.tx_events)))
                event['usb_write_completed'] = True
                event['completed_perf_s'] = time.perf_counter()
            except BaseException as exc:
                self.tx_closed = True
                event['error'] = str(exc)
                raise  # An ambiguous USB write is never retried.


class ResetOnlyReader(Reader):
    def __init__(self):
        can.interfaces.BACKENDS['gs_usb'] = ('reset_piper_disabled_once', 'ResetOnlyBus')
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
        return self.firmware  # No Reader retry and no cached identity substitution.


def passive_followup(reader, output, result, reference, *, seconds=35):
    reader.bus.close_transmit()
    result['post_reset_tx_latched_closed'] = True
    samples = []
    start = time.perf_counter()
    next_progress = start
    try:
        while time.perf_counter() - start < seconds:
            time.sleep(.05)
            snapshot = reader.snapshot()
            samples.append(snapshot)
            if time.perf_counter() >= next_progress:
                raw = snapshot.get('frozen_control') or {}
                print(json.dumps(dict(phase='passive_after_reset',
                    elapsed_s=round(time.perf_counter()-start, 1),
                    driver_codes=raw.get('driver_codes'), arm_status=raw.get('arm_status'),
                    err_code=raw.get('err_code'), execution_allowed=False)), flush=True)
                next_progress = time.perf_counter() + 5
        tail = samples[-20:]
        # Report late state separately; never erase early faults, pose shifts,
        # transport errors or ambiguous writes, and never authorize motion.
        result['late_fault_free_disabled_standby_observed'] = len(tail) == 20 and all(
            not maintenance_reasons(s) and s['frozen_control']['driver_codes'] == [0]*6
            for s in tail)
        deltas = []
        for s in samples:
            raw = s.get('frozen_control') or {}
            q = np.asarray(raw.get('joint_millidegrees', []), dtype=float) / 1000
            if raw.get('valid_payloads') and q.shape == (6,) and np.isfinite(q).all():
                deltas.append(float(np.max(np.abs(q - np.rad2deg(reference)))))
        result['max_recorded_joint_delta_deg'] = max(deltas) if deltas else None
        result['pose_change_over_allowance'] = bool(deltas and max(deltas) > .2)
        result['observation_duration_s'] = time.perf_counter() - start
    finally:
        write_json(output / 'post_reset_observations.json', samples)
        if samples:
            write_json(output / 'after_latest.json', samples[-1])


def perform_reset_only(reader, output, result, *, execute, operator_supported):
    time.sleep(.8)
    reader.query_firmware()
    before = reader.snapshot()
    write_json(output / 'before.json', before)
    reasons = maintenance_reasons(before)
    if reasons:
        raise RuntimeError('Preflight rejected: ' + '; '.join(reasons))
    reference = np.asarray(before['joint_angles_rad'], dtype=float)
    samples = []
    try:
        for _ in range(20):
            time.sleep(.05)
            s = reader.snapshot()
            samples.append(s)
            reasons = maintenance_reasons(s, reference)
            if reasons:
                raise RuntimeError('Stationary preflight rejected: ' + '; '.join(reasons))
    finally:
        write_json(output / 'pre_observations.json', samples)
    if not execute:
        result['outcome'] = 'preflight_only_no_reset'
        return
    reader.bus.permit.arm(reader.snapshot, reference, operator_supported=operator_supported)
    write_json(output / 'reset_intent.json', dict(api='reset()', can_id='0x150',
        data=RESET_BYTES.hex(), maximum_attempts=1, operator_supported=True,
        motors_may_lose_support=True, automatic_enable_or_motion=False))
    result['reset_api_called'] = True
    try:
        reader.robot.reset()
    except Exception as exc:
        result['reset_error'] = str(exc)
    finally:
        reader.bus.close_transmit()
    # Even an ambiguous reset gets passive diagnostics, NEVER another write.
    passive_followup(reader, output, result, reference)
    result['outcome'] = ('reset_delivery_ambiguous_no_retry' if result.get('reset_error') else
        'late_fault_free_observed_motion_still_locked' if result.get(
            'late_fault_free_disabled_standby_observed') else 'recovery_not_verified_no_retry')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-once', action='store_true')
    parser.add_argument('--operator-supported', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.execute_once and not args.operator_supported:
        parser.error('Explicit current mechanical-support confirmation is required')
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(execution_allowed=False, motion_readiness_verified=False,
                  reset_api_called=False, late_fault_free_disabled_standby_observed=False)
    reader = None
    try:
        reader = ResetOnlyReader()
        perform_reset_only(reader, args.output, result, execute=args.execute_once,
                           operator_supported=args.operator_supported)
    except BaseException as exc:
        result.update(outcome='aborted_no_further_write', error=f'{type(exc).__name__}: {exc}')
    finally:
        if reader is not None:
            reader.bus.close_transmit()
            result['tx_events'] = reader.bus.tx_events
            result['firmware_queries_attempted'] = reader.bus.queries_sent
            result['reset_permit_consumed'] = reader.bus.permit.attempted
            result['blocked_commands'] = reader.bus.blocked_commands
            try:
                reader.close()
            except Exception as exc:
                result['close_error'] = str(exc)
        write_json(args.output / 'result.json', result)
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result.get('outcome') in ('preflight_only_no_reset',
        'late_fault_free_observed_motion_still_locked') else 2


if __name__ == '__main__':
    raise SystemExit(main())
