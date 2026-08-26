"""Small single-threaded FIFO for causally committed Cartesian segments.

This queue intentionally knows nothing about IK or joint motion.  Its only job
is to make the hand-off explicit: once a segment is finalized it is immutable,
ordered, and ready for a downstream executor.  A future live-follow policy may
decide what to do with *uncommitted* observations, but it must never rewrite an
entry already accepted by this queue.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .rolling_cartesian_planner import PlannedCartesianSegment


@dataclass(frozen=True)
class QueuedCartesianSegment:
    """One segment plus the replay instant where it became executable work."""

    segment: PlannedCartesianSegment
    enqueued_replay_t_ms: float

    def __post_init__(self) -> None:
        if not isinstance(self.segment, PlannedCartesianSegment):
            raise TypeError("segment must be a PlannedCartesianSegment.")
        if self.enqueued_replay_t_ms < self.segment.time_contract.available_replay_t_ms:
            raise ValueError("A segment cannot enter the queue before it is causally available.")


class PlannedSegmentQueue:
    """FIFO ownership boundary between Cartesian planning and later IK/execution."""

    def __init__(self) -> None:
        self._entries: deque[QueuedCartesianSegment] = deque()
        self._last_segment_index = -1

    @property
    def entries(self) -> tuple[QueuedCartesianSegment, ...]:
        """Read-only queue snapshot for diagnostics and deterministic tests."""
        return tuple(self._entries)

    @property
    def depth(self) -> int:
        return len(self._entries)

    def enqueue(self, segment: PlannedCartesianSegment, replay_t_ms: float) -> QueuedCartesianSegment:
        """Append one newly-finalized segment without reordering planner output."""
        if not isinstance(segment, PlannedCartesianSegment):
            raise TypeError("segment must be a PlannedCartesianSegment.")
        if segment.segment_index <= self._last_segment_index:
            raise ValueError("Queued segment_index must increase strictly.")
        entry = QueuedCartesianSegment(segment, float(replay_t_ms))
        self._entries.append(entry)
        self._last_segment_index = segment.segment_index
        return entry

    def pop_next(self) -> QueuedCartesianSegment | None:
        """Transfer ownership of the oldest committed segment to one consumer."""
        return None if not self._entries else self._entries.popleft()
