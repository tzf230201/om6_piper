"""Bounded, operator-authorized recovery of this disabled Piper; no motion.

Official reset() cuts power: the operator must confirm mechanical stability.
Only one exact reset is allowed, from fresh stationary disabled Teaching state.
Only after verified Standby may one official clear_joint_error(255) be sent.
An unchanged or unexpected state stops this run, without automatic retries.
The normal read-only backend remains unchanged. No enable/target/config writes.
"""
import argparse
from pathlib import Path
import time

import can
from can.interfaces.gs_usb import GsUsbBus
import numpy as np
from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile

from clear_piper_fault_once import (
    CLEAR_BYTES, FaultClearGsUsbBus, disabled_idle_reasons,
    disabled_maintenance_reasons, write_json,
)
from read_robot import Reader

RESET_BYTES = bytes.fromhex('0200000000000000')


def is_exact_reset(msg):
    return (msg.arbitration_id == 0x150 and msg.dlc == 8
            and bytes(msg.data) == RESET_BYTES and not msg.is_extended_id
            and not msg.is_remote_frame and not msg.is_error_frame and not msg.is_fd)


class ResetOncePolicy:
    def __init__(self):
        self.attempted = False
        self.deadline = 0
        self.snapshot = None
        self.reference = None

    def arm(self, snapshot, reference, *, operator_stable):
        if operator_stable is not True:
            raise PermissionError('Operator must confirm safe mechanical support')
        if self.attempted:
            raise PermissionError('No repeat of reset attempt')
        reasons = disabled_maintenance_reasons(snapshot(), reference, expected_ctrl_mode=2)
        if reasons:
            raise PermissionError('; '.join(reasons))
        self.snapshot = snapshot
        self.reference = np.asarray(reference, dtype=float).copy()
        self.deadline = time.monotonic() + 2

    def consume(self, msg):
        if not is_exact_reset(msg):
            raise PermissionError('Only the exact official reset packet is allowed')
        if self.attempted or self.snapshot is None or time.monotonic() > self.deadline:
            raise PermissionError('Reset permit absent, expired, or already used')
        # Consume BEFORE USB IO. An ambiguous send must not be retried.
        self.attempted = True
        self.deadline = 0
        reasons = disabled_maintenance_reasons(
            self.snapshot(), self.reference, expected_ctrl_mode=2)
        if reasons:
            raise PermissionError('; '.join(reasons))


class RecoveryGsUsbBus(FaultClearGsUsbBus):
    def __init__(self, *args, **kwargs):
        self.reset_policy = ResetOncePolicy()
        self.reset_tx_successful = 0
        self.reset_frames = []
        super().__init__(*args, **kwargs)

    def send(self, msg, timeout=None):
        if not is_exact_reset(msg):
            # Parent only permits firmware query and separately armed clear.
            return super().send(msg, timeout)
        try:
            self.reset_policy.consume(msg)
        except Exception:
            self.blocked_commands += 1
            raise
        self.reset_frames.append(dict(id='0x150', data=bytes(msg.data).hex(),
                                      attempted_monotonic=time.monotonic()))
        result = GsUsbBus.send(self, msg, timeout)
        self.reset_tx_successful += 1
        return result


class RecoveryReader(Reader):
    def __init__(self):
        can.interfaces.BACKENDS['gs_usb'] = ('recover_piper_state_once', 'RecoveryGsUsbBus')
        self.channel = 0
        self.firmware_driver = 'default'
        self.firmware = None
        self.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper', firmeware_version='default', interface='gs_usb', channel=0,
            bitrate=1000000, enable_check_can=False, auto_connect=False, timeout=.02))
        self.robot.connect()
        self.bus = self.robot.get_context().get_comm().recv_bus
        if self.robot.get_context().get_comm().send_bus is not self.bus:
            self.robot.disconnect()
            raise RuntimeError('Expected shared guarded RX/TX bus')
        self.gripper = self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)


def observe(reader, output, phase, reference, allowed_modes, count):
    samples = []
    try:
        for _ in range(count):
            time.sleep(.05)
            s = reader.snapshot()
            samples.append(s)
            write_json(output / 'after_latest.json', s)
            mode = (s.get('arm_status') or {}).get('ctrl_mode')
            if mode not in allowed_modes:
                raise RuntimeError(f'{phase}: unexpected control mode {mode}; no further writes')
            reasons = disabled_maintenance_reasons(s, reference, expected_ctrl_mode=mode)
            if reasons:
                raise RuntimeError(f'{phase}: ' + '; '.join(reasons))
    finally:
        write_json(output / f'{phase}.json', samples)
    return samples


def perform_recovery(reader, output, result, *, execute, operator_stable):
    time.sleep(.8)  # Warm up receive FPS before the firmware query.
    reader.query_firmware()
    before = reader.snapshot()
    write_json(output / 'before.json', before)
    mode = (before.get('arm_status') or {}).get('ctrl_mode')
    if mode not in (0, 2):
        raise RuntimeError(f'Preflight: unsupported controller mode {mode}')
    reasons = disabled_maintenance_reasons(before, expected_ctrl_mode=mode)
    if reasons:
        raise RuntimeError('Preflight: ' + '; '.join(reasons))
    if resolve_firmware_profile('piper', before['firmware']['software_version']) != 'default':
        raise RuntimeError('SDK profile mismatch')
    reference = np.asarray(before['joint_angles_rad'])
    samples = observe(reader, output, 'pre_observations', reference, (mode,), 20)
    if not execute:
        result['outcome'] = 'dry_run_no_write'
        return
    if operator_stable is not True:
        raise RuntimeError('Missing operator mechanical-stability confirmation')
    if mode == 2:
        write_json(output / 'reset_intent.json', dict(
            api='reset()', expected_can_id='0x150', expected_data_hex=RESET_BYTES.hex(),
            maximum_attempts=1, operator_confirmed_mechanical_stability=True,
            motor_enable_forbidden=True, automatic_motion_forbidden=True))
        reader.bus.reset_policy.arm(reader.snapshot, reference, operator_stable=operator_stable)
        result['reset_api_called'] = True
        reader.robot.reset()
        samples = observe(reader, output, 'post_reset_observations', reference, (0, 2), 60)
        if not all(s['arm_status']['ctrl_mode'] == 0 for s in samples[-20:]):
            result['outcome'] = 'standby_not_verified_no_clear_no_retry'
            return
    result['standby_verified'] = True
    last = samples[-1]
    # The dedicated clear policy independently rechecks Standby at TX time.
    reasons = disabled_idle_reasons(last, reference)
    if reasons:
        raise RuntimeError('Pre-clear: ' + '; '.join(reasons))
    if any(d['foc_status_code'] for d in last['drivers']):
        write_json(output / 'clear_intent.json', dict(
            api='clear_joint_error(255)', expected_can_id='0x475',
            expected_data_hex=CLEAR_BYTES.hex(), maximum_attempts=1,
            motor_enable_forbidden=True, automatic_motion_forbidden=True))
        reader.bus.clear_policy.arm(reader.snapshot, reference)
        result['clear_api_called'] = True
        result['clear_ack_received'] = bool(reader.robot.clear_joint_error(joint_index=255, timeout=1.0))
    samples = observe(reader, output, 'final_observations', reference, (0,), 60)
    result['fault_clear_verified'] = all(
        all(d['foc_status_code'] == 0 for d in s['drivers']) for s in samples[-20:])
    result['outcome'] = ('standby_clear_motors_disabled' if result['fault_clear_verified']
                         else 'standby_but_flags_persist_no_retry')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-once', action='store_true')
    parser.add_argument('--operator-stable', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.execute_once and not args.operator_stable:
        parser.error('--execute-once requires operator confirmation via --operator-stable')
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(execute_requested=args.execute_once, reset_api_called=False,
                  clear_api_called=False, standby_verified=False, fault_clear_verified=False,
                  clear_ack_received=None, motor_enable_commands_sent=0, motion_commands_sent=0,
                  protection_changes_sent=0, calibration_commands_sent=0)
    reader = None
    try:
        reader = RecoveryReader()
        perform_recovery(reader, args.output, result, execute=args.execute_once,
                         operator_stable=args.operator_stable)
    except Exception as exc:
        result['outcome'] = 'aborted_no_further_write'
        result['error'] = str(exc)
    finally:
        if reader:
            try:
                final = reader.snapshot()
                write_json(args.output / 'final_snapshot.json', final)
                result['final_captured_at'] = final.get('captured_at')
                result['final_arm_status'] = final.get('arm_status')
                result['final_driver_codes'] = [(d or {}).get('foc_status_code')
                                                for d in final.get('drivers', [])]
                result['final_joint_angles_deg'] = final.get('joint_angles_deg')
            except Exception as exc:
                result['final_snapshot_error'] = str(exc)
            for key in ['reset_tx_successful', 'reset_frames', 'clear_tx_successful',
                        'clear_frames', 'blocked_commands']:
                result[key] = getattr(reader.bus, key)
            result['firmware_queries_sent'] = reader.bus.queries_sent
            try:
                reader.close()
            except Exception as exc:
                result['disconnect_error'] = str(exc)
        write_json(args.output / 'result.json', result)
        import json
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result['outcome'] in ('standby_clear_motors_disabled', 'dry_run_no_write') else 2


if __name__ == '__main__':
    raise SystemExit(main())
