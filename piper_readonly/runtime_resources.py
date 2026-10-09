"""Small explicit process-resource sampler for reproducible mapper runs."""
from __future__ import annotations

from dataclasses import dataclass
import os
import time


@dataclass(frozen=True)
class ResourceSample:
    label: str
    elapsed_ms: float
    rss_bytes: int | None
    vms_bytes: int | None

    def as_dict(self):
        return dict(label=self.label, elapsed_ms=self.elapsed_ms, rss_bytes=self.rss_bytes,
                    vms_bytes=self.vms_bytes)


class ProcessResourceSampler:
    """Track process RSS/VMS, explicitly distinguishing unavailable counters."""

    def __init__(self):
        self.started = time.perf_counter()
        self.samples: list[ResourceSample] = []
        try:
            import psutil
            self._process = psutil.Process(os.getpid())
        except Exception:
            self._process = None

    def sample(self, label: str) -> ResourceSample:
        if not str(label):
            raise ValueError('Resource sample requires a label')
        rss = vms = None
        if self._process is not None:
            try:
                with self._process.oneshot():
                    memory = self._process.memory_info()
                rss, vms = int(memory.rss), int(memory.vms)
            except Exception:
                # Lack of an OS counter is reported, never silently converted
                # to zero bytes.
                rss = vms = None
        value = ResourceSample(str(label), (time.perf_counter()-self.started)*1000, rss, vms)
        self.samples.append(value)
        return value

    def summary(self):
        values = [sample.rss_bytes for sample in self.samples if sample.rss_bytes is not None]
        return dict(rss_available=bool(values), samples=[sample.as_dict() for sample in self.samples],
                    rss_start_bytes=values[0] if values else None,
                    rss_end_bytes=values[-1] if values else None,
                    rss_peak_bytes=max(values) if values else None,
                    elapsed_ms=(time.perf_counter()-self.started)*1000)
