"""Capture-time clock-domain adapter for the Isaac GUI ZED stream.

Locks one constant offset from fresh RGB arrivals; never re-stamps inference at
publication time. Network/inference delay after lock remains visible. This is an
estimated stream-clock alignment, not hardware time synchronization.
"""
from collections import deque
import math
import statistics


class CaptureClock:
    def __init__(self, enabled=False, samples=8, jitter_s=.20):
        self.enabled=enabled; self.samples=samples; self.jitter_s=jitter_s
        self.reset()

    def reset(self):
        self.offset=None; self.window=deque(maxlen=self.samples)
        self.last_raw=None; self.last_capture=None; self.reason='calibrating' if self.enabled else 'SDK clock'

    def observe(self, raw, clock):
        if not self.enabled:return raw
        if not all(math.isfinite(x) for x in (raw,clock)):return None
        if self.last_raw is not None and raw<=self.last_raw:
            if raw<self.last_raw-.1:self.reset()
            else:return self.convert(raw)
        self.last_raw=raw
        delta=clock-raw
        if self.offset is None:
            self.window.append((raw,delta))
            if len(self.window)<self.samples or self.window[-1][0]-self.window[0][0]<.2:return None
            offsets=sorted(d for _,d in self.window)
            # Require a stable offset; reject queue stalls and clock jumps.
            if offsets[-2]-offsets[1]>self.jitter_s:
                self.reason='unstable sensor/simulation clock offset';return None
            # Earliest-arrival envelope minimizes transport-delay contamination.
            # Keep synchronized clocks untouched instead of absorbing network delay.
            self.offset=offsets[1] if statistics.median(offsets)<-.25 else 0.
            self.reason='estimated fixed capture offset' if self.offset else 'SDK clock'
        if delta-self.offset<-.25:
            self.reset();self.window.append((raw,delta));self.last_raw=raw
            self.reason='sensor clock jumped; recalibrating';return None
        self.last_capture=raw+self.offset
        return self.last_capture

    def convert(self, raw):
        return raw if not self.enabled else (None if self.offset is None else raw+self.offset)
