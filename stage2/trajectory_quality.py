"""Pure Stage 2.2 visual trajectory quality gate V0."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from math import hypot

from .trajectory import TrajectorySample


@dataclass(frozen=True)
class QualityGateConfig:
    """Tunable V0 quality thresholds; these are not robot safety limits."""

    max_normalized_speed: float = 5.0

    def __post_init__(self) -> None:
        if self.max_normalized_speed <= 0:
            raise ValueError("max_normalized_speed must be positive.")


@dataclass(frozen=True)
class TrajectoryQualityResult:
    """Processed samples and compact quality-gate statistics."""

    processed_samples: tuple[TrajectorySample, ...]
    accepted_count: int
    rejected_speed_count: int
    rejected_time_count: int
    original_invalid_count: int


def _rejected_copy(sample: TrajectorySample, reason: str) -> TrajectorySample:
    """Return an invalid copy while preserving all original coordinates."""
    return TrajectorySample(
        t_ms=sample.t_ms,
        valid=False,
        x_norm=sample.x_norm,
        y_norm=sample.y_norm,
        u=sample.u,
        v=sample.v,
        invalid_reason=reason,
        pen_state=sample.pen_state,
        stroke_id=sample.stroke_id,
    )


def apply_quality_gate(
    raw_samples: Iterable[TrajectorySample],
    config: QualityGateConfig = QualityGateConfig(),
) -> TrajectoryQualityResult:
    """Apply time and normalized-speed checks without mutating raw samples."""
    processed: list[TrajectorySample] = []
    reference: TrajectorySample | None = None
    accepted_count = 0
    rejected_speed_count = 0
    rejected_time_count = 0
    original_invalid_count = 0

    for sample in raw_samples:
        if not sample.valid:
            processed.append(sample)
            original_invalid_count += 1
            reference = None
            continue

        if reference is None:
            processed.append(sample)
            accepted_count += 1
            reference = sample
            continue

        dt_ms = sample.t_ms - reference.t_ms
        if dt_ms <= 0:
            processed.append(_rejected_copy(sample, "invalid_time"))
            rejected_time_count += 1
            continue

        assert sample.x_norm is not None and sample.y_norm is not None
        assert reference.x_norm is not None and reference.y_norm is not None
        distance = hypot(
            sample.x_norm - reference.x_norm,
            sample.y_norm - reference.y_norm,
        )
        speed = distance / (dt_ms / 1_000.0)
        if speed > config.max_normalized_speed:
            processed.append(_rejected_copy(sample, "speed_gate"))
            rejected_speed_count += 1
            continue

        processed.append(sample)
        accepted_count += 1
        reference = sample

    return TrajectoryQualityResult(
        processed_samples=tuple(processed),
        accepted_count=accepted_count,
        rejected_speed_count=rejected_speed_count,
        rejected_time_count=rejected_time_count,
        original_invalid_count=original_invalid_count,
    )
