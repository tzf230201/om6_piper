"""python-can gs_usb backend with an explicit robot-command deny policy.

The only permitted outbound CAN frame is the documented firmware read query.
Normal USB configuration/bitrate and CAN acknowledgement are not motor commands.
"""
import time
from threading import Lock
from collections import Counter
import can
import libusb_package
import usb.backend.libusb1
from can.interfaces.gs_usb import GsUsbBus
from usb_receive import UsbReceiver, QueuedUsbReceiver

EIGHT_BYTE_FEEDBACK_IDS = frozenset((*range(0x251,0x257),*range(0x261,0x267),*range(0x2A1,0x2A9),0x4AF))

def is_firmware_query(msg):
    return (msg.arbitration_id == 0x4AF and bytes(msg.data) == b'\x01'
            and msg.dlc == 1 and not msg.is_extended_id
            and not msg.is_remote_frame and not msg.is_error_frame and not msg.is_fd)

class ReadOnlyGsUsbBus(GsUsbBus):
    def __init__(self, *args, **kwargs):
        self.received = Counter()
        self.last_received = {}
        self.queries_sent = 0
        self.blocked_commands = 0
        self.raw_latest = {}
        self.rx_lock = Lock()
        usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
        super().__init__(*args, **kwargs)
        self.usb_receiver = UsbReceiver(self.gs_usb, self.channel_info)

    def send(self, msg, timeout=None):
        if is_firmware_query(msg):
            self.queries_sent += 1
            return super().send(msg, timeout)
        self.blocked_commands += 1
        raise PermissionError(f"Read-only monitor blocked CAN TX 0x{msg.arbitration_id:X}")

    def _recv_internal(self, timeout):
        msg, filtered = self.usb_receiver.read(timeout)
        if msg is not None and (not msg.is_rx or msg.is_remote_frame or msg.is_extended_id):
            return None,False
        if msg is not None and msg.is_rx and not msg.is_error_frame and not msg.is_remote_frame and not msg.is_extended_id:
            ident = msg.arbitration_id
            required = ident in EIGHT_BYTE_FEEDBACK_IDS
            if required and (msg.dlc != 8 or len(msg.data) != 8):
                with self.usb_receiver.lock:
                    self.usb_receiver.counts['malformed_frames'] += 1
                return None, False
            with self.rx_lock:
                self.received[ident] += 1
                self.last_received[ident] = self.usb_receiver.last_received_perf
                self.raw_latest[ident] = bytes(msg.data).hex()
        return msg, filtered

    def capture_receive(self):
        """One coherent host receive cache; not a simultaneous robot sample."""
        with self.rx_lock:
            return self.last_received.copy(), self.raw_latest.copy(), self.received.copy()

class QueuedReadOnlyGsUsbBus(ReadOnlyGsUsbBus):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.usb_receiver=QueuedUsbReceiver(self.usb_receiver)

    def shutdown(self):
        if isinstance(getattr(self,'usb_receiver',None),QueuedUsbReceiver):
            self.usb_receiver.close()
        super().shutdown()


class PassiveReadOnlyGsUsbBus(ReadOnlyGsUsbBus):
    """No application CAN TX, including firmware reads; normal CAN ACK remains."""
    def send(self, msg, timeout=None):
        self.blocked_commands += 1
        raise PermissionError('Passive receiver: all application CAN frames blocked')


class QueuedPassiveReadOnlyGsUsbBus(QueuedReadOnlyGsUsbBus):
    send = PassiveReadOnlyGsUsbBus.send


def register_backend(buffered_rx=False, passive=False):
    # Replace backend resolution only inside this monitor process.
    if passive:
        name = 'QueuedPassiveReadOnlyGsUsbBus' if buffered_rx else 'PassiveReadOnlyGsUsbBus'
    else:
        name = 'QueuedReadOnlyGsUsbBus' if buffered_rx else 'ReadOnlyGsUsbBus'
    can.interfaces.BACKENDS['gs_usb'] = ('readonly_bus', name)
