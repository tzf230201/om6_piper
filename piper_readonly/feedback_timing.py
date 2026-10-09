"""Process-local receive scheduling; no robot commands or system persistence.

Shorter CPython GIL slices reduce contention with USB receive callbacks.
Windows timer requests are paired and restored when the last reader closes.
This is NOT a hard-realtime guarantee and never changes a freshness deadline.
"""
import ctypes
import sys
import threading

_lock=threading.Lock()
_users=0
_previous_switch=None
_winmm=None


class ReceiverTiming:
    def __init__(self, enabled=True):
        self.enabled=enabled
        self.active=False

    def start(self):
        global _users,_previous_switch,_winmm
        if not self.enabled or self.active:return self
        with _lock:
            if _users==0:
                previous=sys.getswitchinterval()
                timer=None
                if sys.platform=='win32':
                    candidate=ctypes.WinDLL('winmm')
                    if candidate.timeBeginPeriod(1)!=0:
                        raise RuntimeError('Unable to request Windows 1 ms timer')
                    timer=candidate
                try:sys.setswitchinterval(.001)
                except BaseException:
                    if timer is not None:timer.timeEndPeriod(1)
                    raise
                _previous_switch=previous
                _winmm=timer
            _users+=1
            self.active=True
        return self

    def close(self):
        global _users,_previous_switch,_winmm
        with _lock:
            if not self.active:return
            self.active=False
            _users-=1
            if _users==0:
                try:sys.setswitchinterval(_previous_switch)
                finally:
                    if _winmm is not None:_winmm.timeEndPeriod(1)
                    _previous_switch=None
                    _winmm=None

    def profile(self):
        return dict(enabled=self.enabled,active=self.active,
                    gil_switch_ms=1 if self.active else None,
                    windows_timer_ms=1 if self.active and sys.platform=='win32' else None,
                    hard_realtime_guarantee=False)
