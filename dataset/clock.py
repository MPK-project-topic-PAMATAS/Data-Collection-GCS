"""Host monotonic -> wall (UTC epoch) clock conversion.

Every sensor's timestamp_ns is host CLOCK_MONOTONIC (time.monotonic_ns()) —
an arbitrary epoch (usually system boot), not a real date. Anything that
turns a timestamp into a filename or a Parquet partition date (YYYYMMDD)
needs a real UTC epoch instead, or everything lands under a 1970-01-01
folder. The offset between the two clocks is effectively constant for the
life of a process, so it only needs to be captured once.
"""

from __future__ import annotations

import time


def capture_wall_offset_ns() -> int:
    """Capture the current mono->wall offset. Call once (e.g. at recorder
    start), not per-sample — time.time_ns()/time.monotonic_ns() each carry
    their own small jitter, so repeated captures would add noise for no
    benefit since the offset itself does not drift within a process.
    """
    return time.time_ns() - time.monotonic_ns()


def mono_to_wall_ns(mono_ns: int, wall_offset_ns: int) -> int:
    """Convert one host CLOCK_MONOTONIC timestamp to a UTC-epoch one."""
    return mono_ns + wall_offset_ns
