"""Hand-observation data contract for later Stage 2.1 integration."""

from __future__ import annotations

from dataclasses import dataclass


NormalizedPoint = tuple[float, float]
PixelPoint = tuple[int, int]
NormalizedLandmark = tuple[float, float, float]


@dataclass(frozen=True)
class HandObservation:
    """One frame's hand-detection result without any detector dependency."""

    timestamp_ms: float
    detected: bool
    handedness: str | None = None
    handedness_score: float | None = None
    landmarks_norm: tuple[NormalizedLandmark, ...] = ()
    index_tip_norm: NormalizedPoint | None = None
    index_tip_px: PixelPoint | None = None

    def __post_init__(self) -> None:
        if self.handedness_score is not None and not 0.0 <= self.handedness_score <= 1.0:
            raise ValueError("handedness_score must be between 0 and 1.")
        if not self.detected and any(
            value is not None
            for value in (
                self.handedness,
                self.handedness_score,
                self.index_tip_norm,
                self.index_tip_px,
            )
        ):
            raise ValueError("A no-hand observation cannot include hand-specific fields.")
        if not self.detected and self.landmarks_norm:
            raise ValueError("A no-hand observation cannot include landmarks.")
