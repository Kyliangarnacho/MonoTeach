"""Stage 2.2 trajectory data contract and observation conversion."""

from __future__ import annotations

from dataclasses import dataclass

from .hand_observation import HandObservation


TRAJECTORY_SOURCE = "c920_mediapipe_index_tip"
COORDINATE_SPACE = "image_normalized"


@dataclass(frozen=True)
class TrajectorySample:
    """One raw fingertip sample relative to the start of a recording."""

    t_ms: float
    valid: bool
    x_norm: float | None
    y_norm: float | None
    u: int | None
    v: int | None
    invalid_reason: str | None = None

    def __post_init__(self) -> None:
        if self.t_ms < 0:
            raise ValueError("t_ms must be non-negative.")

        coordinates = (self.x_norm, self.y_norm, self.u, self.v)
        coordinates_present = tuple(value is not None for value in coordinates)

        if self.valid:
            if not all(coordinates_present):
                raise ValueError("A valid sample must include all coordinates.")
            if self.invalid_reason is not None:
                raise ValueError("A valid sample cannot have an invalid_reason.")
            return

        if self.invalid_reason is None:
            raise ValueError("An invalid sample must include an invalid_reason.")
        if any(coordinates_present) and not all(coordinates_present):
            raise ValueError(
                "An invalid sample must include either all coordinates or none."
            )


@dataclass(frozen=True)
class TrajectoryMetadata:
    """Context needed to interpret a two-dimensional fingertip trajectory."""

    schema_version: str
    trajectory_id: str
    frame_width: int
    frame_height: int
    camera_index: int
    recording_start_timestamp_ms: float
    source: str = TRAJECTORY_SOURCE
    coordinate_space: str = COORDINATE_SPACE

    def __post_init__(self) -> None:
        if not self.schema_version:
            raise ValueError("schema_version must not be empty.")
        if not self.trajectory_id:
            raise ValueError("trajectory_id must not be empty.")
        if self.frame_width <= 0 or self.frame_height <= 0:
            raise ValueError("frame_width and frame_height must be positive.")


@dataclass(frozen=True)
class Trajectory2D:
    """A trajectory whose immutable raw_samples are the canonical source of truth."""

    metadata: TrajectoryMetadata
    raw_samples: tuple[TrajectorySample, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.raw_samples, tuple):
            raise TypeError("raw_samples must be an immutable tuple.")


def observation_to_trajectory_sample(
    observation: HandObservation,
    recording_start_timestamp_ms: float,
) -> TrajectorySample:
    """Convert one hand observation into a raw trajectory sample."""
    t_ms = observation.timestamp_ms - recording_start_timestamp_ms
    if t_ms < 0:
        raise ValueError(
            "Observation timestamp cannot precede recording_start_timestamp_ms."
        )

    if not observation.detected:
        return TrajectorySample(
            t_ms=t_ms,
            valid=False,
            x_norm=None,
            y_norm=None,
            u=None,
            v=None,
            invalid_reason="no_hand",
        )

    if observation.index_tip_norm is None or observation.index_tip_px is None:
        raise ValueError(
            "A detected HandObservation must include index_tip_norm and index_tip_px."
        )

    x_norm, y_norm = observation.index_tip_norm
    u, v = observation.index_tip_px
    return TrajectorySample(
        t_ms=t_ms,
        valid=True,
        x_norm=x_norm,
        y_norm=y_norm,
        u=u,
        v=v,
        invalid_reason=None,
    )
