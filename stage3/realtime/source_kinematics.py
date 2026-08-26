"""Derivative estimates in the original demonstration clock.

These values describe human/source motion. A later joint planner must derive
its own execution-time qd/qdd after allocating robot execution time.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .stream_contract import RealtimeTaskEvent


Vector2 = tuple[float, float]


@dataclass(frozen=True)
class SourceKinematicEstimate:
    """Position and local derivatives evaluated at one source observation."""

    source_index: int
    source_t_ms: float
    position_mm: Vector2
    velocity_mm_s: Vector2
    acceleration_mm_s2: Vector2
    method: str = "nonuniform_quadratic_three_point_v1"

    def __post_init__(self) -> None:
        if not isinstance(self.source_index, int) or isinstance(self.source_index, bool) or self.source_index < 0:
            raise ValueError("source_index must be a non-negative integer.")
        if not isinstance(self.source_t_ms, (int, float)) or isinstance(self.source_t_ms, bool) or not math.isfinite(self.source_t_ms) or self.source_t_ms < 0:
            raise ValueError("source_t_ms must be a finite non-negative number.")
        for name in ("position_mm", "velocity_mm_s", "acceleration_mm_s2"):
            value = getattr(self, name)
            if not isinstance(value, tuple) or len(value) != 2 or not all(
                isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(item)
                for item in value
            ):
                raise ValueError(f"{name} must be a finite two-item tuple.")
        if self.method != "nonuniform_quadratic_three_point_v1":
            raise ValueError("method must identify the supported source derivative estimator.")


def estimate_source_start(
    first: RealtimeTaskEvent,
    second: RealtimeTaskEvent,
    third: RealtimeTaskEvent,
) -> SourceKinematicEstimate:
    """Estimate source derivatives at the first point of a three-point window."""
    return _estimate_triplet(first, second, third, "start")


def estimate_source_center(
    previous: RealtimeTaskEvent,
    center: RealtimeTaskEvent,
    following: RealtimeTaskEvent,
) -> SourceKinematicEstimate:
    """Estimate source velocity and acceleration at the center point."""
    return _estimate_triplet(previous, center, following, "center")


def estimate_source_end(
    first: RealtimeTaskEvent,
    second: RealtimeTaskEvent,
    third: RealtimeTaskEvent,
) -> SourceKinematicEstimate:
    """Estimate source derivatives at the final point of a three-point window."""
    return _estimate_triplet(first, second, third, "end")


def _estimate_triplet(
    first: RealtimeTaskEvent,
    second: RealtimeTaskEvent,
    third: RealtimeTaskEvent,
    evaluation: str,
) -> SourceKinematicEstimate:
    _validate_triplet(first, second, third)
    h0_s = (second.source_t_ms - first.source_t_ms) / 1_000.0
    h1_s = (third.source_t_ms - second.source_t_ms) / 1_000.0
    velocity_x, acceleration_x = _quadratic_derivatives(
        first.x_mm, second.x_mm, third.x_mm, h0_s, h1_s, evaluation
    )
    velocity_y, acceleration_y = _quadratic_derivatives(
        first.y_mm, second.y_mm, third.y_mm, h0_s, h1_s, evaluation
    )
    event = {"start": first, "center": second, "end": third}[evaluation]
    assert event.x_mm is not None and event.y_mm is not None
    return SourceKinematicEstimate(
        source_index=event.source_index,
        source_t_ms=event.source_t_ms,
        position_mm=(event.x_mm, event.y_mm),
        velocity_mm_s=(velocity_x, velocity_y),
        acceleration_mm_s2=(acceleration_x, acceleration_y),
    )


def _validate_triplet(
    first: RealtimeTaskEvent,
    second: RealtimeTaskEvent,
    third: RealtimeTaskEvent,
) -> None:
    if not all(isinstance(event, RealtimeTaskEvent) for event in (first, second, third)):
        raise TypeError("Source derivative estimation requires RealtimeTaskEvent values.")
    if not first.source_t_ms < second.source_t_ms < third.source_t_ms:
        raise ValueError("Source derivative estimation requires strictly increasing source_t_ms.")
    if not all(event.planning_eligible for event in (first, second, third)):
        raise ValueError("Source derivative estimation cannot cross an observation barrier.")
    if len({event.motion_mode for event in (first, second, third)}) != 1 or len(
        {event.stroke_id for event in (first, second, third)}
    ) != 1:
        raise ValueError("Source derivative estimation cannot cross a pen semantic transition.")


def _quadratic_derivatives(
    first: float | None,
    second: float | None,
    third: float | None,
    h0_s: float,
    h1_s: float,
    evaluation: str,
) -> tuple[float, float]:
    assert first is not None and second is not None and third is not None
    span_s = h0_s + h1_s
    if evaluation == "start":
        velocity = (
            -(2.0 * h0_s + h1_s) * first / (h0_s * span_s)
            + span_s * second / (h0_s * h1_s)
            - h0_s * third / (h1_s * span_s)
        )
    elif evaluation == "center":
        velocity = (
            -h1_s * first / (h0_s * span_s)
            + (h1_s - h0_s) * second / (h0_s * h1_s)
            + h0_s * third / (h1_s * span_s)
        )
    elif evaluation == "end":
        velocity = (
            h1_s * first / (h0_s * span_s)
            - span_s * second / (h0_s * h1_s)
            + (h0_s + 2.0 * h1_s) * third / (h1_s * span_s)
        )
    else:
        raise ValueError("evaluation must be start, center, or end.")
    acceleration = 2.0 * (
        first / (h0_s * span_s)
        - second / (h0_s * h1_s)
        + third / (h1_s * span_s)
    )
    return velocity, acceleration
