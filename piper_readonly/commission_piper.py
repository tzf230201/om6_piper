"""One-shot Piper commissioning, separate from the read-only monitor.

Default: read-only preflight. Explicit --execute is required for enable-stop
or a <=2 degree joint jog at the SDK's 1% setting. That percentage is NOT a
certified physical speed. No reset, disable, gripper, MIT, limit change, or
automatic retry. Electronic stop uses damping and may lower a raised arm.
Windows, USB, and this watchdog are NOT a safety-rated real-time controller.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
import secrets
from pathlib import Path
import threading
import time

import can
from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP
from gs_usb.gs_usb_frame import GsUsbFrame
from pyAgxArm import AgxArmFactory, create_agx_arm_config

from readonly_bus import QueuedReadOnlyGsUsbBus, is_firmware_query
from read_piper_joint_limits import QueryBudget, is_limit_query, compare_limits
from read_robot import Reader, plain
from feedback_timing import ReceiverTiming
from feedback_health import GROUPS
from motion_supervisor import JogPlan, MotionSupervisor, DryRunTransport, compile_jog, finite_six
from app_paths import INTERLOCK_PATH, WORKSPACE_ROOT

ENABLES = tuple((0x471, bytes([j, 2, 0, 0, 0, 0, 0, 0])) for j in range(1, 7))
STOP = (0x150, bytes.fromhex('0100000000000000'))
TERMINAL = {'DONE', 'STOPPED', 'STOP_UNCONFIRMED', 'ABORTED'}
USB_TIMEOUT_MS = 20  # Requested native USB timeout, not a hard OS deadline.
INTERLOCK = INTERLOCK_PATH


def check_hardware_release(path=None, *, kind=None, consume_revalidation=False):
    """Keep targets locked; a reviewed recovery may permit ONE enable-stop test.

    There is intentionally no CLI override. A supported recovery procedure and
    code review are needed before authorizing revalidation. That narrow permit
    is consumed before motor IO, cannot authorize a jog, expires after ten
    minutes and requires the exact reviewed executor and recovery artifacts.
    """
    gate_path = path or INTERLOCK
    value = json.loads(gate_path.read_text(encoding='utf-8'))
    if value.get('control_tests_released') is True:
        return 'released'
    permit = value.get('enable_stop_revalidation') or {}
    if kind != 'enable-stop' or permit.get('allowed') is not True or permit.get('used') is not False:
        raise PermissionError('Physical commissioning locked: ' + value.get('reason', 'not released'))
    if (permit.get('operator_authorized') is not True or
        permit.get('mechanical_support_confirmed') is not True or
        permit.get('physical_emergency_available') is not True):
        raise PermissionError('Revalidation requires the recorded operator/support/emergency confirmations')
    age = (datetime.now().astimezone()-datetime.fromisoformat(permit['reviewed_at'])).total_seconds()
    if not 0 <= age <= 600 or permit.get('source_hashes') != source_hashes():
        raise PermissionError('Revalidation review expired or executor changed')
    root = WORKSPACE_ROOT
    claim_path = (root/permit.get('claim_path', '')).resolve()
    if root not in claim_path.parents or claim_path.exists() or not claim_path.parent.is_dir():
        raise PermissionError('Revalidation claim invalid or already consumed')
    for key in ('reset_result', 'reset_review', 'preflight_result'):
        spec = permit.get(key) or {}
        artifact = (root/spec.get('path', '')).resolve()
        if root not in artifact.parents or not artifact.is_file():
            raise PermissionError('Missing recovery/preflight evidence')
        if hashlib.sha256(artifact.read_bytes()).hexdigest() != spec.get('sha256'):
            raise PermissionError('Recovery/preflight evidence changed')
    reset = json.loads((root/permit['reset_result']['path']).read_text(encoding='utf-8'))
    review = json.loads((root/permit['reset_review']['path']).read_text(encoding='utf-8'))
    preflight = json.loads((root/permit['preflight_result']['path']).read_text(encoding='utf-8'))
    if (reset.get('late_fault_free_disabled_standby_observed') is not True
        or reset.get('reset_error') or reset.get('close_error')
        or review.get('final_driver_codes') != [0]*6
        or review.get('enable_commands') != 0 or review.get('motion_target_commands') != 0
        or preflight.get('state') != 'PREFLIGHT_PASSED_NO_MOTOR_WRITES'
        or preflight.get('command_packets_attempted') != 0):
        raise PermissionError('Recovery/preflight evidence does not support an enable-stop retest')
    if consume_revalidation:
        # Exclusive creation prevents two processes consuming the same permit.
        # A claim remains consumed even if updating the gate or USB IO fails.
        with claim_path.open('x', encoding='utf-8') as claim:
            json.dump(dict(kind='enable-stop', consumed_at=datetime.now().astimezone().isoformat(),
                           motion_targets_allowed=False), claim)
        permit.update(used=True, used_at=datetime.now().astimezone().isoformat(),
                      status='Consumed before motor IO; no automatic retry')
        write_json(gate_path, value)
    return 'enable-stop-revalidation-only'


def enable_only_plan(start):
    q = finite_six(start)
    return JogPlan(q, q, ENABLES, (), (STOP,))


def bounded_usb_write(device, ident, data, *, echo_id=0):
    """Same gs_usb wire format as python-can, with explicit USB timeout.

    A completed USB write does NOT acknowledge robot execution. Never retry
    an ambiguous/short write. Do not mutate the caller's CAN payload.
    """
    if type(ident) is not int or not 0 <= ident <= 0x7ff or not 1 <= len(data) <= 8:
        raise ValueError('Standard classic CAN frames only')
    frame = GsUsbFrame(ident, bytes(data))
    if type(echo_id) is not int or not 0 <= echo_id < 0xFFFFFFFF:
        raise ValueError('Invalid gs_usb echo cookie')
    frame.echo_id = echo_id  # USB-only cookie: does not change CAN ID/payload.
    packet = frame.pack(bool(device.device_flags & GS_CAN_MODE_HW_TIMESTAMP))
    written = device.gs_usb.write(0x02, packet, timeout=USB_TIMEOUT_MS)
    if written != len(packet):
        raise can.CanOperationError('Short USB write: delivery is ambiguous, no retry')
    return dict(echo_id=echo_id)


class CommissionBus(QueuedReadOnlyGsUsbBus):
    def __init__(self, *args, **kwargs):
        self.query_budget = QueryBudget()
        self.query_frames = []
        self.session = None
        self._echo_cookie = secrets.randbelow(0xFFFFFF00) + 1
        self._echo_lock = threading.Lock()
        super().__init__(*args, **kwargs)

    def send(self, msg, timeout=None):
        # SDK callers can NEVER transmit control, even during commissioning.
        if self.session is not None:
            self.blocked_commands += 1
            raise PermissionError('No SDK transmissions after session starts')
        if is_firmware_query(msg):
            self.queries_sent += 1
        elif is_limit_query(msg):
            self.query_budget.consume(msg)
            self.query_frames.append(dict(id='0x472', data=bytes(msg.data).hex()))
        else:
            self.blocked_commands += 1
            raise PermissionError('Only firmware/individually permitted limit READs')
        self._write_tracked(msg.arbitration_id, bytes(msg.data))

    def _write_tracked(self, ident, data):
        with self._echo_lock:
            cookie = self._echo_cookie
            self._echo_cookie = cookie + 1
            if self._echo_cookie >= 0xFFFFFFFF:
                self._echo_cookie = 1
        try:
            return bounded_usb_write(self.gs_usb, ident, data, echo_id=cookie)
        except BaseException as exc:
            # Keep correlation available even if USB completion is ambiguous.
            exc.piper_echo_id = cookie
            raise

    def start_session(self, plan, kind, snapshot):
        if self.session is not None:
            raise PermissionError('One session per connection; no re-arm')
        session = CommissionSession(plan, kind, snapshot, self._write_tracked)
        # Validate the packet whitelist before consuming the diagnostic permit.
        check_hardware_release(kind=kind, consume_revalidation=True)
        self.session = session
        return self.session


class CommissionReader(Reader):
    def __init__(self):
        can.interfaces.BACKENDS['gs_usb'] = ('commission_piper', 'CommissionBus')
        self.channel = 0
        self.firmware_driver = 'default'
        self.firmware = None
        self._receive_timing = ReceiverTiming(enabled=True)
        self.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper', firmeware_version='default', interface='gs_usb', channel=0,
            bitrate=1000000, enable_check_can=False, auto_connect=False, timeout=.02))
        try:
            self._receive_timing.start()
            self.robot.connect()
            comm = self.robot.get_context().get_comm()
            self.bus = comm.recv_bus
            if comm.send_bus is not self.bus:
                raise RuntimeError('Expected one shared guarded bus')
            self.gripper = self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        except BaseException:
            self.close()
            raise

    def read_limits(self):
        limits = []
        for j in range(1, 7):
            self.bus.query_budget.allow(j)
            result = self.robot.get_joint_angle_vel_limits(j, timeout=1, min_interval=1)
            value = plain(result.msg) if result is not None else None
            limits.append(value)
        if not all(x['valid_limit_response'] for x in compare_limits(limits, None)):
            raise RuntimeError('Missing/invalid same-session controller limits')
        return limits


class CommissionSession:
    """Exact bounded command sequence. Writer injection enables offline tests.

    Uses the already-tested supervisor's feedback validator, but keeps the
    dry-run class dry-run only. Every live frame gets a NEW feedback check.
    stop() cancels remaining packets BEFORE acquiring the USB writer lock.
    """
    def __init__(self, plan, kind, snapshot, writer, clock=time.perf_counter):
        if kind not in ('enable-stop', 'jog'):
            raise ValueError('Unknown commissioning kind')
        if plan.enable_frames != ENABLES or plan.stop_frames != (STOP,):
            raise ValueError('Unexpected enable/stop packets')
        if kind == 'enable-stop' and plan.target_frames:
            raise ValueError('Enable-stop must not contain any motion target')
        if kind == 'jog':
            expected_mode = (0x151, bytes.fromhex('0101010000000000'))
            import struct
            expected_targets = tuple((0x155+i, struct.pack('>ii',
                round(plan.target_deg[2*i]*1000), round(plan.target_deg[2*i+1]*1000))) for i in range(3))
            if plan.target_frames != (expected_mode, *expected_targets):
                raise ValueError('Unexpected target packets or speed')
            if not .05 <= max(abs(a-b) for a,b in zip(finite_six(plan.start_deg), finite_six(plan.target_deg))) <= 2:
                raise ValueError('Jog exceeds commissioning displacement')
        self.plan, self.kind = plan, kind
        self.snapshot, self.writer, self.clock = snapshot, writer, clock
        self.validator = MotionSupervisor(plan, DryRunTransport())
        self.lock = threading.RLock()
        # Stop must not need the main state/snapshot lock. TX is serialized
        # separately, with a bounded wait if a native USB call is stuck.
        self.stop_lock = threading.RLock()
        self.tx_lock = threading.Lock()
        self.cancel = threading.Event()
        self.state = 'IDLE'
        self.reason = None
        self.events = []
        self.samples = []
        self.started = None
        self.last_heartbeat = clock()
        self.last_counts = {}
        self.observed_counts = {}
        self.stop_at = None
        self.target_at = None
        self.stable_since = None
        self.stable_q = None
        self.stable_n = 0
        self.control_attempted = False
        self.stop_attempted = False
        self._target_attempted = False
        self.enable_confirmed_stationary = False
        self.stop_write_completed_at = None

    def _advance_active(self, state):
        with self.stop_lock:
            if self.cancel.is_set():
                return False
            self.state = state
            return True

    def _observe_tx_echoes(self, s):
        # Matching cookies/payloads distinguish USB receipt from an adapter
        # TX echo. They never prove robot execution or release the interlock.
        echoes = (s.get('transport_health') or {}).get('recent_tx_echoes', [])
        for event in self.events:
            cookie = event.get('echo_id')
            if cookie is None:
                continue
            for echo in echoes:
                stamp = echo.get('received_perf_s')
                if (echo.get('echo_id') == cookie and echo.get('can_id') == event['can_id']
                    and echo.get('data_hex') == event['data_hex']
                    and not echo.get('extended') and not echo.get('remote')
                    and isinstance(stamp, (int,float)) and math.isfinite(stamp)
                    and event['time'] <= stamp <= self.clock()):
                    event['adapter_echo_observed'] = True
                    event['adapter_echo_perf_s'] = stamp

    def validate(self, s, now, stopping=False):
        self.validator.state = 'STOP_REQUESTED' if stopping else 'ENABLING'
        c, error = self.validator._feedback(s, now)
        if error:
            raise RuntimeError(error)
        if type(c.get('gripper_status_code')) is not int or c['gripper_status_code'] != 0:
            raise RuntimeError('Frozen gripper status must be disabled and normal')
        return c

    def check_start(self, s, now):
        c = self.validate(s, now)
        if c['ctrl_mode'] != 0 or c['motion_status'] != 0 or any(c['driver_codes']):
            raise RuntimeError('Start requires normal disabled standby')
        self.check_start_pose(c)
        return c

    def check_start_pose(self, c):
        if max(abs(x/1000-y) for x,y in zip(c['joint_millidegrees'], self.plan.start_deg)) > .2:
            raise RuntimeError('Start changed or unexpected movement while enabling')

    def _transmit(self, kind, packet):
        ident, data = packet
        event = dict(kind=kind, time=self.clock(), can_id=f'0x{ident:03X}', data_hex=data.hex(),
                     usb_write_started=False, usb_write_complete=False,
                     adapter_echo_observed=False, robot_execution_confirmed=False)
        self.events.append(event)  # Intent consumed before an ambiguous write.
        acquired = False
        try:
            acquired = self.tx_lock.acquire(timeout=.05)
            if not acquired:
                raise TimeoutError('TX lock unavailable; physical stop required, no retry')
            if kind != 'damped_stop' and self.cancel.is_set():
                raise RuntimeError('Cancelled before USB write; packet not sent')
            self.control_attempted = True
            event['usb_write_started'] = True
            metadata = self.writer(ident, data)
            if isinstance(metadata, dict):
                event['echo_id'] = metadata.get('echo_id')
            event['usb_write_complete'] = True
        except BaseException as exc:
            event['error'] = f'{type(exc).__name__}: {exc}'
            if hasattr(exc, 'piper_echo_id'):
                event['echo_id'] = exc.piper_echo_id
            raise
        finally:
            if acquired:
                self.tx_lock.release()
            event['write_duration_s'] = self.clock() - event['time']

    def _batch(self, kind, frames):
        for packet in frames:
            with self.lock:
                if self.cancel.is_set():
                    raise RuntimeError('Session cancelled; remaining commands discarded')
                now = self.clock()
                if now-self.last_heartbeat > .15:
                    raise RuntimeError('Local supervisor deadline missed before TX')
                s = self.snapshot()
                c = self.validate(s, self.clock())
                self.observed_counts = dict(s['rx_ids'])
                self.check_start_pose(c)
                if c['motion_status'] != 0:
                    raise RuntimeError('Unexpected movement before target completes')
                if kind == 'enable' and c['ctrl_mode'] != 0:
                    raise RuntimeError('Standby lost while enabling')
                if kind == 'target' and any(v != 0x40 for v in c['driver_codes']):
                    raise RuntimeError('Joint enable lost before target')
                self._transmit(kind, packet)

    def begin(self):
        with self.lock:
            if self.state != 'IDLE' or self.cancel.is_set():
                raise RuntimeError('No automatic retry or resume')
            s = self.snapshot()
            self.check_start(s, self.clock())
            self.started = self.last_heartbeat = self.clock()
            self.last_counts = dict(s['rx_ids'])
            self.observed_counts = dict(s['rx_ids'])
            if not self._advance_active('ENABLING'):
                raise RuntimeError('Cancelled during start validation')
        try:
            self._batch('enable', self.plan.enable_frames)
        except BaseException:
            self.stop('enable transmission failed')
            raise

    def stop(self, reason):
        self.cancel.set()  # Independent of writer lock: cancel later packets.
        with self.stop_lock:
            if self.state in TERMINAL or self.stop_attempted:
                return
            self.reason = reason
            if not self.control_attempted:
                self.state = 'ABORTED'
                return
            self.stop_attempted = True
            self.state = 'STOP_REQUESTED'
            self.stop_at = self.clock()
            self.stable_since = None
            self.stable_n = 0
            # Never fetch a snapshot here: a stuck SDK getter must not block
            # the stop attempt. Verification also checks post-write RX times.
            self.last_counts = self.observed_counts.copy()
            try:
                self._transmit('damped_stop', STOP)
                self.stop_write_completed_at = self.clock()
            except BaseException as exc:
                self.state = 'STOP_UNCONFIRMED'
                self.reason += f'; stop write failed: {type(exc).__name__}: {exc}'

    def _stable(self, s, c, now, ids):
        if not MotionSupervisor._new_counts(s, self.last_counts, ids):
            return False
        self.last_counts = dict(s['rx_ids'])
        q = tuple(x/1000 for x in c['joint_millidegrees'])
        if self.stable_since is None or max(abs(x-y) for x,y in zip(q, self.stable_q)) > .1:
            self.stable_since, self.stable_q, self.stable_n = now, q, 1
        else:
            self.stable_n += 1
        return now-self.stable_since >= .5 and self.stable_n >= 5

    def poll(self):
        with self.lock:
            now = self.clock()
            if self.state in TERMINAL:
                return self.state
            if self.state == 'IDLE':
                raise RuntimeError('Session not started')
            if now-self.last_heartbeat > .15 or now < self.last_heartbeat:
                self.stop('Local supervisor deadline missed')
            self.last_heartbeat = now
            s = self.snapshot()
            self.samples.append(s)
            self._observe_tx_echoes(s)
            self.observed_counts = dict(s.get('rx_ids', {}))
            # A watchdog may have terminated this session while snapshot()
            # was blocked. Do not overwrite that decision with a late result.
            now = self.clock()
            if self.state in TERMINAL:
                return self.state
            if self.state == 'STOP_REQUESTED':
                if now-self.stop_at > 3:
                    self.state = 'STOP_UNCONFIRMED'
                    return self.state
                try:
                    c = self.validate(s, self.clock(), stopping=True)
                except Exception:
                    self.stable_since = None
                    return self.state
                if c['arm_status'] != 1 or c['err_code'] or c['teach_status'] or any(x & ~0x40 for x in c['driver_codes']):
                    self.stable_since = None
                    return self.state
                # Cached payloads that were received before the USB stop
                # write completed cannot serve as a post-stop observation.
                if self.stop_write_completed_at is None or any(
                    s['snapshot_perf_s'] - s['rx_age_s'][f'0x{i:03X}'] <= self.stop_write_completed_at
                    for i in (*GROUPS['joint_position'], 0x2A1)):
                    self.stable_since = None
                    return self.state
                if self._stable(s, c, now, (*GROUPS['joint_position'], 0x2A1)):
                    with self.stop_lock:
                        if self.state == 'STOP_REQUESTED' and self.clock()-self.stop_at <= 3:
                            self.state = 'STOPPED'
                            for event in self.events:
                                if event['kind'] == 'damped_stop':
                                    event['controller_stop_status_observed'] = True
                                    event['stationary_window_observed_s'] = now-self.stable_since
                return self.state
            try:
                c = self.validate(s, self.clock())
                if now-self.started > 15:
                    raise RuntimeError('Session time limit reached')
                if self.state == 'ENABLING':
                    self.check_start_pose(c)
                    if c['ctrl_mode'] != 0 or c['motion_status'] != 0:
                        raise RuntimeError('Unexpected mode/movement while enabling')
                    if now-self.started > 2:
                        raise RuntimeError('Enable feedback timeout')
                    if all(x == 0x40 for x in c['driver_codes']) and self._stable(s, c, now,
                            (*GROUPS['driver_status'], *GROUPS['joint_position'], 0x2A1)):
                        self.enable_confirmed_stationary = True
                        if self.kind == 'enable-stop':
                            self.stop('Planned enable-stop verification; no target sent')
                        else:
                            if self._target_attempted:
                                raise RuntimeError('Target batch already consumed')
                            self._target_attempted = True
                            self._batch('target', self.plan.target_frames)
                            self.target_at = self.clock()
                            self.last_counts = dict(self.snapshot()['rx_ids'])
                            self.stable_since = None
                            self._advance_active('RUNNING')
                    elif not all(x == 0x40 for x in c['driver_codes']):
                        self.stable_since = None
                elif self.state == 'RUNNING':
                    if any(x != 0x40 for x in c['driver_codes']):
                        raise RuntimeError('Joint enable lost')
                    if (c['ctrl_mode'], c['motion_mode']) != (1, 1):
                        if now-self.target_at > .1:
                            raise RuntimeError('MOVE J position mode not acknowledged')
                        return self.state
                    q = tuple(x/1000 for x in c['joint_millidegrees'])
                    if any(x < min(a,b)-.25 or x > max(a,b)+.25 for x,a,b in zip(q,self.plan.start_deg,self.plan.target_deg)):
                        raise RuntimeError('Joint envelope exceeded')
                    if c['motion_status'] == 0 and max(abs(x-y) for x,y in zip(q,self.plan.target_deg)) <= .15:
                        if self._stable(s, c, now, (*GROUPS['joint_position'], 0x2A1)):
                            self._advance_active('DONE')  # Leave holding; NEVER disable/reset.
                    else:
                        self.stable_since = None
            except Exception as exc:
                self.stop(str(exc))
            return self.state

    def watchdog_check(self):
        now = self.clock()
        if self.state == 'STOP_REQUESTED' and (not math.isfinite(now) or now-self.stop_at > 3):
            with self.stop_lock:
                if self.state == 'STOP_REQUESTED':
                    self.state = 'STOP_UNCONFIRMED'
                    self.reason += '; stop feedback deadline expired'
            return
        if self.state not in TERMINAL | {'IDLE', 'STOP_REQUESTED'} and (
                not math.isfinite(now) or now < self.last_heartbeat or now-self.last_heartbeat > .15 or now-self.started > 17):
            self.stop('Watchdog: main loop missed local deadline')


class Watchdog:
    def __init__(self, session):
        self.session = session
        self.finished = threading.Event()
        self.thread = threading.Thread(target=self.run, name='piper-stop-watchdog', daemon=True)

    def run(self):
        while not self.finished.wait(.01):
            self.session.watchdog_check()

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *args):
        self.finished.set()
        self.thread.join(1)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2), encoding='utf-8')


def source_hashes():
    here = Path(__file__).resolve().parent
    return {name: hashlib.sha256((here/name).read_bytes()).hexdigest() for name in (
        'commission_piper.py', 'motion_supervisor.py', 'readonly_bus.py',
        'usb_receive.py', 'feedback_health.py', 'read_robot.py')}


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['preflight', 'enable-stop', 'jog'], default='preflight')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--operator-ready', action='store_true')
    p.add_argument('--arm-supported-for-damped-stop', action='store_true')
    p.add_argument('--target-deg', nargs=6, type=float)
    p.add_argument('--stop-test-report', type=Path)
    return p


def check_options(args):
    live = args.mode != 'preflight'
    if live and not (args.execute and args.operator_ready and args.arm_supported_for_damped_stop):
        raise ValueError('Physical test requires --execute, --operator-ready, --arm-supported-for-damped-stop')
    if not live and args.execute:
        raise ValueError('Preflight never executes motor commands')
    if args.mode == 'enable-stop' and args.target_deg:
        raise ValueError('Enable-stop cannot accept a target')
    if args.mode == 'jog':
        if args.target_deg is None or args.stop_test_report is None:
            raise ValueError('Jog requires explicit target and verified physical stop-test report')
        proof = json.loads(args.stop_test_report.read_text(encoding='utf-8'))
        if proof.get('mode') != 'enable-stop' or proof.get('state') != 'STOPPED' or not proof.get('execute'):
            raise ValueError('No successful physical enable-stop test')
        if proof.get('source_hashes') != source_hashes():
            raise ValueError('Executor changed since stop test; revalidate it')
        age = (datetime.now().astimezone()-datetime.fromisoformat(proof['captured_at'])).total_seconds()
        if not 0 <= age <= 3600:
            raise ValueError('Physical stop test is not from the last hour')
        if proof.get('motion_target_packets_attempted') != 0:
            raise ValueError('Invalid stop-test provenance')
        if not proof.get('enable_confirmed_stationary') or proof.get('enable_packets_attempted') != 6:
            raise ValueError('Stop test did not confirm all six joints enabled and stationary')
        return proof
    return None


def main(argv=None):
    args = parser().parse_args(argv)
    if args.mode != 'preflight':
        check_hardware_release(kind=args.mode)  # Before USB opens.
    proof = check_options(args)
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(mode=args.mode, execute=args.execute, state='NOT_STARTED',
                  captured_at=datetime.now().astimezone().isoformat(), source_hashes=source_hashes(),
                  hard_realtime=False, collision_check_performed=False,
                  deadline_is_local_policy=True, physical_motion_readiness_verified=False)
    reader = session = None
    try:
        reader = CommissionReader()
        time.sleep(.8)
        reader.query_firmware()
        if reader.firmware is None:
            raise RuntimeError('Firmware identity unavailable')
        limits = reader.read_limits()
        reader.bus.usb_receiver.begin_observation()
        before = reader.snapshot()
        write_json(args.output/'before.json', before)
        report['firmware'] = before['firmware']
        report['limits'] = limits
        if proof is not None and proof.get('firmware') != before['firmware']:
            raise RuntimeError('Device differs from stop-test identity')
        q = tuple(x/1000 for x in before['frozen_control']['joint_millidegrees'])
        plan = compile_jog(q, args.target_deg, limits) if args.target_deg else enable_only_plan(q)
        check = CommissionSession(plan, 'jog' if args.target_deg else 'enable-stop',
                                  reader.snapshot, lambda *a: (_ for _ in ()).throw(AssertionError('READ ONLY')))
        # Require a continuous half-second healthy/stationary disabled window.
        end = time.perf_counter()+.7
        reference = None
        preflight = []
        while time.perf_counter() < end:
            s = reader.snapshot()
            c = check.check_start(s, time.perf_counter())
            if reference is None:
                reference = c['joint_millidegrees']
            if max(abs(a-b) for a,b in zip(reference,c['joint_millidegrees'])) > 100:
                raise RuntimeError('Robot not stationary during preflight')
            preflight.append(s)
            time.sleep(.02)
        write_json(args.output/'preflight.json', preflight)
        report['start_deg'] = list(q)
        report['target_deg'] = list(plan.target_deg) if args.target_deg else None
        if args.mode == 'preflight':
            report['state'] = 'PREFLIGHT_PASSED_NO_MOTOR_WRITES'
        else:
            session = reader.bus.start_session(plan, args.mode, reader.snapshot)
            with Watchdog(session):
                try:
                    session.begin()
                    while session.state not in TERMINAL:
                        session.poll()
                        time.sleep(.02)
                except BaseException as exc:
                    report['run_error'] = f'{type(exc).__name__}: {exc}'
                    session.stop('Operator interrupt or execution exception')
                finally:
                    # Even after an exception, attempt to OBSERVE the stop;
                    # never retry its packet or resume the target sequence.
                    until = time.perf_counter()+3.5
                    while session.state == 'STOP_REQUESTED' and time.perf_counter() < until:
                        try:
                            session.poll()
                        except BaseException:
                            pass
                        time.sleep(.02)
                    if session.state == 'STOP_REQUESTED':
                        session.state = 'STOP_UNCONFIRMED'
            report['state'], report['reason'] = session.state, session.reason
    except BaseException as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        if session is not None:
            session.stop('Unhandled exception')
            report['state'] = session.state
        else:
            report['state'] = 'PREFLIGHT_REJECTED_NO_MOTOR_WRITES'
    finally:
        events = session.events if session else []
        report['command_packets_attempted'] = len(events)
        report['motion_target_packets_attempted'] = sum(x['kind']=='target' for x in events)
        report['enable_packets_attempted'] = sum(x['kind']=='enable' for x in events)
        report['stop_packets_attempted'] = sum(x['kind']=='damped_stop' for x in events)
        report['enable_confirmed_stationary'] = session.enable_confirmed_stationary if session else False
        if session and session.state == 'STOP_UNCONFIRMED':
            # Preserve the operator reports and recovery history when reclosing.
            value = json.loads(INTERLOCK.read_text(encoding='utf-8'))
            value.update(control_tests_released=False,
                reason='Physical stop not confirmed; no re-enable or motion until reviewed recovery',
                evidence=str((args.output/'result.json').resolve()),
                captured_at=datetime.now().astimezone().isoformat())
            write_json(INTERLOCK, value)
        if session:
            write_json(args.output/'commands.json', events)
            write_json(args.output/'samples.json', session.samples)
        if reader:
            try:
                write_json(args.output/'after.json', reader.snapshot())
                report['firmware_queries_sent'] = reader.bus.queries_sent
                report['limit_read_packets'] = reader.bus.query_frames
                report['blocked_commands'] = reader.bus.blocked_commands
            finally:
                try:
                    reader.close()
                except BaseException as exc:
                    report['close_error'] = str(exc)
        write_json(args.output/'result.json', report)
        print(json.dumps(report, indent=2), flush=True)
    return 0 if report['state'] in ('PREFLIGHT_PASSED_NO_MOTOR_WRITES', 'STOPPED', 'DONE') else 2


if __name__ == '__main__':
    raise SystemExit(main())
