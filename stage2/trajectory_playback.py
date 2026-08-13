"""Hardware- and UI-independent trajectory playback timing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, Iterable, TypeVar

from .trajectory import Trajectory2D, TrajectorySample
from .trajectory_io import LoadedTrajectory
from .workspace_trajectory import WorkspaceTrajectory2D, WorkspaceTrajectorySample


PlaybackSample = TypeVar("PlaybackSample", TrajectorySample, WorkspaceTrajectorySample)


@dataclass(frozen=True)
class PlaybackEvent(Generic[PlaybackSample]):
    """One trajectory sample and its due time on the playback clock."""

    due_ms: float
    sample: PlaybackSample


@dataclass(frozen=True)
class PlaybackTimeline(Generic[PlaybackSample]):
    """Immutable, time-scaled playback schedule."""

    events: tuple[PlaybackEvent[PlaybackSample], ...]
    playback_speed: float
    source_duration_ms: float
    playback_duration_ms: float

    def samples_due(self, elapsed_playback_ms: float) -> tuple[PlaybackSample, ...]:
        """Return the ordered sample prefix due at the supplied playback time."""
        if elapsed_playback_ms < 0:
            raise ValueError("elapsed_playback_ms must be non-negative.")
        return tuple(
            event.sample for event in self.events if event.due_ms <= elapsed_playback_ms
        )


def build_playback_timeline(
    source: LoadedTrajectory | Trajectory2D | WorkspaceTrajectory2D,
    *,
    samples: Iterable[PlaybackSample] | None = None,
    playback_speed: float = 1.0,
) -> PlaybackTimeline[PlaybackSample]:
    """Build a schedule from raw or derived samples without mutating either."""
    if playback_speed <= 0:
        raise ValueError("playback_speed must be positive.")

    if isinstance(source, LoadedTrajectory):
        default_samples: tuple[TrajectorySample, ...] = source.trajectory.raw_samples
    elif isinstance(source, Trajectory2D):
        default_samples = source.raw_samples
    elif isinstance(source, WorkspaceTrajectory2D):
        default_samples = source.samples
    else:
        raise TypeError("source must be a loaded, image, or workspace trajectory.")
    scheduled_samples = default_samples if samples is None else tuple(samples)
    _validate_sample_timeline(scheduled_samples)

    events = tuple(
        PlaybackEvent(due_ms=sample.t_ms / playback_speed, sample=sample)
        for sample in scheduled_samples
    )
    source_duration_ms = scheduled_samples[-1].t_ms if scheduled_samples else 0.0
    return PlaybackTimeline(
        events=events,
        playback_speed=float(playback_speed),
        source_duration_ms=source_duration_ms,
        playback_duration_ms=source_duration_ms / playback_speed,
    )


def _validate_sample_timeline(samples: tuple[PlaybackSample, ...]) -> None:
    previous_t_ms: float | None = None
    for sample in samples:
        if previous_t_ms is not None and sample.t_ms <= previous_t_ms:
            raise ValueError("Playback sample t_ms values must be strictly increasing.")
        previous_t_ms = sample.t_ms
