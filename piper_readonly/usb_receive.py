"""Observable gs_usb RX, without changing adapter mode or robot TX permissions.

Unlike upstream GsUsb.read(), only a USB timeout is treated as no data. Other
USB failures propagate. Adapter overflow is recorded, never hidden by a fresh
host timestamp. Uses the same classic-CAN frame format as upstream gs_usb.
"""
from array import array
from collections import Counter, deque
from threading import Lock
from threading import Event, Thread
from queue import Queue, Empty, Full
import time

import can
import usb.core
from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP
from gs_usb.gs_usb_frame import GsUsbFrame, GS_USB_NONE_ECHO_ID


class UsbReceiver:
    def __init__(self, device, channel):
        self.device = device
        self.channel = channel
        self.hw_timestamp = bool(device.device_flags & GS_CAN_MODE_HW_TIMESTAMP)
        self.buffer = array('B', [0]) * (24 if self.hw_timestamp else 20)
        self.lock = Lock()
        self.counts = Counter()
        self.last_error = None
        self.max_read_s = 0.0
        self.hw_latest = {}
        self.hw_latest_perf = {}
        self.hw_max_gap_s = {}
        self.startup_hw_max_gap_s = {}
        self.observation_epoch = 0
        self.clock_discontinuities = deque(maxlen=32)
        self.long_gaps = deque(maxlen=128)
        self.last_received_perf = None
        self.tx_echoes = deque(maxlen=128)
        self.error_frames = deque(maxlen=16)

    def begin_observation(self):
        """Start gap statistics after identity/warmup; preserve every error count."""
        with self.lock:
            self.startup_hw_max_gap_s = self.hw_max_gap_s.copy()
            self.hw_latest.clear()
            self.hw_latest_perf.clear()
            self.hw_max_gap_s.clear()
            self.observation_epoch += 1

    def diagnostics(self):
        with self.lock:
            return dict(counts=dict(self.counts), last_error=self.last_error,
                        max_usb_read_s=self.max_read_s,
                        hardware_timestamps_available=self.hw_timestamp,
                        hardware_max_gap_s={f'0x{k:03X}': v for k, v in self.hw_max_gap_s.items()},
                        startup_hardware_max_gap_s={f'0x{k:03X}':v for k,v in self.startup_hw_max_gap_s.items()},
                        recent_tx_echoes=[x.copy() for x in self.tx_echoes],
                        recent_can_errors=[x.copy() for x in self.error_frames],
                        recent_hardware_clock_discontinuities=[x.copy() for x in self.clock_discontinuities],
                        recent_long_frame_gaps=[x.copy() for x in self.long_gaps],
                        long_gap_threshold_s=.1,
                        hardware_clock_event_capacity=self.clock_discontinuities.maxlen,
                        observation_epoch=self.observation_epoch,
                        hardware_clock_event_note='Diagnostic timestamp pairs only; counters/interlocks unchanged. Epoch 0 is startup. History survives begin_observation; bounded to latest events.',
                        tx_echo_note='Adapter echo only; not robot execution or physical stopping evidence',
                        hardware_timestamp_note='Adapter capture time, not robot sample time; no absolute host synchronization')

    def read(self, timeout):
        started = time.perf_counter()
        try:
            # PyUSB supports a caller-owned array, avoiding a new buffer per RX.
            size = self.device.gs_usb.read(0x81, self.buffer,
                                           max(1, round((timeout or .001) * 1000)))
            self.last_received_perf = time.perf_counter()
        except usb.core.USBTimeoutError:
            with self.lock:
                self.counts['usb_timeouts'] += 1
            return None, False
        except usb.core.USBError as exc:
            with self.lock:
                self.counts['usb_errors'] += 1
                self.last_error = str(exc)
            raise can.CanOperationError('USB receive failed; feedback unavailable') from exc
        finally:
            elapsed = time.perf_counter() - started
            with self.lock:
                self.counts['usb_read_calls'] += 1
                self.max_read_s = max(self.max_read_s, elapsed)
        if size != len(self.buffer):
            with self.lock:
                self.counts['malformed_frames'] += 1
            raise can.CanOperationError(f'Invalid gs_usb frame length: {size}')
        frame = GsUsbFrame()
        GsUsbFrame.unpack_into(frame, self.buffer, self.hw_timestamp)
        with self.lock:
            self.counts['usb_frames'] += 1
            # Linux gs_usb.c defines GS_CAN_FLAG_OVERFLOW = BIT(0).
            if frame.flags & 1:
                self.counts['adapter_overflow_reports'] += 1
            if frame.can_dlc > 8 or frame.flags & ~1 or frame.channel != 0:
                self.counts['malformed_frames'] += 1
                raise can.CanOperationError('Unsupported gs_usb frame format/channel')
            if frame.is_error_frame:
                self.counts['can_error_frames'] += 1
                self.error_frames.append(dict(can_id=f'0x{frame.arbitration_id:X}',
                    data_hex=bytes(frame.data[:frame.can_dlc]).hex(),
                    received_perf_s=self.last_received_perf))
            elif frame.echo_id != GS_USB_NONE_ECHO_ID:
                self.counts['tx_echo_frames'] += 1
                self.tx_echoes.append(dict(echo_id=frame.echo_id,
                    can_id=f'0x{frame.arbitration_id:03X}',
                    data_hex=bytes(frame.data[:frame.can_dlc]).hex(),
                    received_perf_s=self.last_received_perf,
                    adapter_timestamp_us=frame.timestamp_us if self.hw_timestamp else None,
                    extended=frame.is_extended_id, remote=frame.is_remote_frame))
            if frame.echo_id == GS_USB_NONE_ECHO_ID and not frame.is_error_frame and self.hw_timestamp:
                ident = frame.arbitration_id
                old = self.hw_latest.get(ident)
                if old is not None:
                    gap = ((frame.timestamp_us - old) & 0xFFFFFFFF) / 1e6
                    if gap < 2147:  # exclude backwards/reset timestamps
                        self.hw_max_gap_s[ident] = max(self.hw_max_gap_s.get(ident, 0), gap)
                        host_gap = self.last_received_perf - self.hw_latest_perf[ident]
                        if gap > .1 or host_gap > .1:
                            self.long_gaps.append(dict(can_id=f'0x{ident:03X}',
                                adapter_gap_s=gap, host_gap_s=host_gap,
                                received_perf_s=self.last_received_perf,
                                observation_epoch=self.observation_epoch))
                    else:
                        self.counts['hardware_clock_discontinuities'] += 1
                        self.clock_discontinuities.append(dict(
                            can_id=f'0x{ident:03X}',
                            previous_adapter_timestamp_us=old,
                            adapter_timestamp_us=frame.timestamp_us,
                            signed_numeric_delta_us=frame.timestamp_us - old,
                            unsigned_modulo_gap_us=(frame.timestamp_us - old) & 0xFFFFFFFF,
                            previous_received_perf_s=self.hw_latest_perf.get(ident),
                            received_perf_s=self.last_received_perf,
                            observation_epoch=self.observation_epoch))
                self.hw_latest[ident] = frame.timestamp_us
                self.hw_latest_perf[ident] = self.last_received_perf
        return can.Message(timestamp=frame.timestamp, arbitration_id=frame.arbitration_id,
                           is_extended_id=frame.is_extended_id, is_remote_frame=frame.is_remote_frame,
                           is_error_frame=frame.is_error_frame, channel=self.channel,
                           dlc=frame.can_dlc, data=bytes(frame.data[:frame.can_dlc]),
                           is_rx=frame.echo_id == GS_USB_NONE_ECHO_ID), False


class QueuedUsbReceiver:
    """Optional bounded RX pump: USB reads do not wait for SDK packet decoding.

    FIFO ordering and original receive times are preserved. Overflow is fatal,
    not silently dropped. The queue is not a substitute for a motion watchdog.
    """
    def __init__(self, base):
        self.base=base;self.lock=base.lock;self.counts=base.counts
        self.queue=Queue(maxsize=256);self.stopping=Event();self.error=None
        self.high_water=0;self.max_queue_age_s=0.;self.last_received_perf=None
        self.worker=Thread(target=self._pump,name='piper-usb-rx',daemon=True)
        self.worker.start()

    def _pump(self):
        try:
            while not self.stopping.is_set():
                result=self.base.read(.02)
                if result[0] is None:continue
                try:self.queue.put_nowait((result,self.base.last_received_perf))
                except Full:
                    with self.lock:self.counts['host_queue_overflows']+=1
                    raise can.CanOperationError('Host USB receive queue overflow')
                self.high_water=max(self.high_water,self.queue.qsize())
        except Exception as exc:
            with self.lock:self.counts['rx_pump_errors']+=1
            self.error=exc;self.stopping.set()

    def read(self,timeout):
        if self.error is not None:raise self.error
        try:result,stamp=self.queue.get(timeout=max(.001,timeout or .001))
        except Empty:
            if self.error is not None:raise self.error
            return None,False
        if self.error is not None:raise self.error
        self.last_received_perf=stamp
        self.max_queue_age_s=max(self.max_queue_age_s,time.perf_counter()-stamp)
        return result

    def begin_observation(self):self.base.begin_observation()

    def diagnostics(self):
        return {**self.base.diagnostics(),'buffered_rx':True,
                'queue_high_water':self.high_water,'queue_capacity':256,
                'max_queue_age_s':self.max_queue_age_s,
                'pump_error':str(self.error) if self.error else None}

    def close(self):
        self.stopping.set();self.worker.join(timeout=1)
        if self.worker.is_alive():raise RuntimeError('USB RX worker did not stop')
