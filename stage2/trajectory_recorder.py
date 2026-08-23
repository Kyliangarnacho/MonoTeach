"""Lightweight recording state machine for Stage 2.2 trajectories."""

from __future__ import annotations

from enum import Enum
from uuid import uuid4

from .hand_observation import HandObservation
from .trajectory import (
    Trajectory2D,
    TrajectoryMetadata,
    TrajectorySample,
    observation_to_trajectory_sample,
)


SCHEMA_VERSION = "1.0"


class RecorderState(Enum):
    """Lifecycle states of a trajectory recorder."""

    IDLE = "idle"
    RECORDING = "recording"
    READY = "ready"


class TrajectoryRecorder:
    """Collect ordered hand observations and seal them into a Trajectory2D."""

    def __init__(self) -> None:
        self._state = RecorderState.IDLE
        self._metadata: TrajectoryMetadata | None = None
        self._working_samples: list[TrajectorySample] = []
        self._finished_trajectory: Trajectory2D | None = None
        self._last_observation_timestamp_ms: float | None = None

    @property
    def state(self) -> RecorderState:
        return self._state

    @property
    def sample_count(self) -> int:
        if self._state is RecorderState.READY and self._finished_trajectory is not None:
            return len(self._finished_trajectory.raw_samples)
        return len(self._working_samples)

    @property
    def finished_trajectory(self) -> Trajectory2D | None:
        return self._finished_trajectory

    @property
    def current_trajectory(self) -> Trajectory2D | None:
        """Alias for the sealed trajectory available in READY state."""
        return self._finished_trajectory

    @property
    def samples(self) -> tuple[TrajectorySample, ...]:
        """Return a read-only snapshot of working or sealed raw samples."""
        if self._finished_trajectory is not None:
            return self._finished_trajectory.raw_samples
        return tuple(self._working_samples)

    def start(
        self,
        *,
        recording_start_timestamp_ms: float,
        frame_width: int,
        frame_height: int,
        camera_index: int,
    ) -> TrajectoryMetadata:
        """Begin a new recording and return its generated metadata."""
        if self._state is not RecorderState.IDLE:
            raise RuntimeError(
                f"Cannot start recording while recorder state is {self._state.name}; "
                "call reset() before starting a new recording."
            )

        metadata = TrajectoryMetadata(
            schema_version=SCHEMA_VERSION,
            trajectory_id=str(uuid4()),
            frame_width=frame_width,
            frame_height=frame_height,
            camera_index=camera_index,
            recording_start_timestamp_ms=recording_start_timestamp_ms,
        )
        self._metadata = metadata
        self._working_samples = []
        self._finished_trajectory = None
        self._last_observation_timestamp_ms = None
        self._state = RecorderState.RECORDING
        return metadata

    def record(
        self,
        observation: HandObservation,
        *,
        pen_state: str | None = None,
        stroke_id: int | None = None,
    ) -> TrajectorySample | None:
        """Record one observation while RECORDING; IDLE is an explicit no-op."""
        if self._state is RecorderState.IDLE:
            return None
        if self._state is RecorderState.READY:
            raise RuntimeError(
                "Cannot record while recorder state is READY; call reset() first."
            )
        if self._metadata is None:
            raise RuntimeError("Recorder metadata is unavailable while RECORDING.")

        previous_timestamp = self._last_observation_timestamp_ms
        if previous_timestamp is not None and observation.timestamp_ms <= previous_timestamp:
            raise ValueError(
                "Observation timestamps must be strictly increasing: "
                f"received {observation.timestamp_ms} after {previous_timestamp}."
            )

        sample = observation_to_trajectory_sample(
            observation,
            self._metadata.recording_start_timestamp_ms,
            pen_state=pen_state,
            stroke_id=stroke_id,
        )
        self._working_samples.append(sample)
        self._last_observation_timestamp_ms = observation.timestamp_ms
        return sample

    def record_pen_up_barrier(self, timestamp_ms: float) -> TrajectorySample | None:
        """Append one non-writing UP marker without recording fingertip motion."""
        if self._state is RecorderState.IDLE:
            return None
        if self._state is RecorderState.READY:
            raise RuntimeError(
                "Cannot record while recorder state is READY; call reset() first."
            )
        if self._metadata is None:
            raise RuntimeError("Recorder metadata is unavailable while RECORDING.")
        previous_timestamp = self._last_observation_timestamp_ms
        if previous_timestamp is not None and timestamp_ms <= previous_timestamp:
            raise ValueError(
                "Observation timestamps must be strictly increasing: "
                f"received {timestamp_ms} after {previous_timestamp}."
            )
        sample = TrajectorySample(
            t_ms=timestamp_ms - self._metadata.recording_start_timestamp_ms,
            valid=False,
            x_norm=None,
            y_norm=None,
            u=None,
            v=None,
            invalid_reason="pen_up",
            pen_state="UP",
            stroke_id=None,
        )
        self._working_samples.append(sample)
        self._last_observation_timestamp_ms = timestamp_ms
        return sample

    def stop(self) -> Trajectory2D:
        """Seal the mutable work buffer into immutable raw_samples."""
        if self._state is not RecorderState.RECORDING:
            raise RuntimeError(
                f"Cannot stop recording while recorder state is {self._state.name}."
            )
        if self._metadata is None:
            raise RuntimeError("Recorder metadata is unavailable while RECORDING.")

        trajectory = Trajectory2D(
            metadata=self._metadata,
            raw_samples=tuple(self._working_samples),
        )
        self._finished_trajectory = trajectory
        self._state = RecorderState.READY
        return trajectory

    def reset(self) -> None:
        """Discard all working and finished recording state and return to IDLE."""
        self._working_samples = []
        self._metadata = None
        self._finished_trajectory = None
        self._last_observation_timestamp_ms = None
        self._state = RecorderState.IDLE
