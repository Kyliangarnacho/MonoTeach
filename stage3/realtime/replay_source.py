"""Caller-driven offline replay of immutable workspace trajectories."""

from __future__ import annotations

import math
from numbers import Real

from stage2.workspace_trajectory import WorkspaceTrajectory2D

from .realtime_sample import RealtimeWorkspaceSample


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite non-negative number.")
    numeric_value = float(value)
    if not math.isfinite(numeric_value) or numeric_value < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number.")
    return numeric_value


def _positive_finite(value: object, name: str) -> float:
    numeric_value = _finite_nonnegative(value, name)
    if numeric_value == 0.0:
        raise ValueError(f"{name} must be positive.")
    return numeric_value


class ReplaySource:
    """Release workspace samples only after their scaled replay due time.

    Time is supplied by the caller via :meth:`advance_to`; no wall clock,
    sleeping, thread, or mutation of the source trajectory is involved.
    """

    def __init__(
        self,
        trajectory: WorkspaceTrajectory2D,
        *,
        playback_speed: float = 1.0,
    ) -> None:
        if not isinstance(trajectory, WorkspaceTrajectory2D):
            raise TypeError("trajectory must be a WorkspaceTrajectory2D.")
        self._playback_speed = _positive_finite(playback_speed, "playback_speed")
        self._events = self._build_events(trajectory)
        self._next_index = 0
        self._replay_elapsed_ms = 0.0

    @property
    def playback_speed(self) -> float:
        return self._playback_speed

    @property
    def replay_elapsed_ms(self) -> float:
        return self._replay_elapsed_ms

    @property
    def is_finished(self) -> bool:
        return self._next_index == len(self._events)

    def advance_to(self, replay_elapsed_ms: float) -> tuple[RealtimeWorkspaceSample, ...]:
        """Advance virtual replay time and return only newly due samples."""
        elapsed_ms = _finite_nonnegative(replay_elapsed_ms, "replay_elapsed_ms")
        if elapsed_ms < self._replay_elapsed_ms:
            raise ValueError("replay_elapsed_ms must not move backwards; call reset().")
        self._replay_elapsed_ms = elapsed_ms
        first_new_index = self._next_index
        while (
            self._next_index < len(self._events)
            and self._events[self._next_index].replay_time_ms <= elapsed_ms
        ):
            self._next_index += 1
        return self._events[first_new_index : self._next_index]

    def reset(self) -> None:
        """Return the virtual source to its initial, unreleased state."""
        self._next_index = 0
        self._replay_elapsed_ms = 0.0

    def _build_events(
        self,
        trajectory: WorkspaceTrajectory2D,
    ) -> tuple[RealtimeWorkspaceSample, ...]:
        previous_t_ms: float | None = None
        events: list[RealtimeWorkspaceSample] = []
        for source_index, sample in enumerate(trajectory.samples):
            if previous_t_ms is not None and sample.t_ms <= previous_t_ms:
                raise ValueError("WorkspaceTrajectory samples must have strictly increasing t_ms.")
            events.append(
                RealtimeWorkspaceSample(
                    source_index=source_index,
                    replay_time_ms=sample.t_ms / self._playback_speed,
                    workspace_sample=sample,
                )
            )
            previous_t_ms = sample.t_ms
        return tuple(events)
