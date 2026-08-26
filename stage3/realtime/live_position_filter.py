"""Causal, auditable position smoothing for live workspace observations.

The camera/MediaPipe result remains the raw source evidence.  This module
creates a *new* derived position estimate for the start gate and later
planner.  It never rewrites a capture timestamp, calibration result or raw
workspace coordinate.

The baseline is the One Euro filter: a low-pass filter whose cutoff increases
when the hand is moving.  It reduces stationary landmark jitter without
forcing the same large lag during a deliberate stroke.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

from .live_observation import LiveWorkspaceObservation
from .stream_contract import RealtimeTaskEvent


def _positive_finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a positive finite number.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{name} must be a positive finite number.")
    return numeric


def _nonnegative_finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a non-negative finite number.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{name} must be a non-negative finite number.")
    return numeric


@dataclass(frozen=True)
class OneEuroPositionFilterConfig:
    """User-visible One Euro parameters expressed in real-time units.

    ``min_cutoff_hz`` controls still-hand stability.  ``beta`` raises the
    cutoff as estimated hand speed rises, reducing movement lag.  They must
    be tuned from a captured diagnostic trace; the defaults are only a safe
    first live baseline for an approximately 30 Hz C920 stream.
    """

    min_cutoff_hz: float = 1.5
    beta: float = 0.02
    derivative_cutoff_hz: float = 1.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "min_cutoff_hz", _positive_finite(self.min_cutoff_hz, "min_cutoff_hz"))
        object.__setattr__(self, "beta", _nonnegative_finite(self.beta, "beta"))
        object.__setattr__(
            self,
            "derivative_cutoff_hz",
            _positive_finite(self.derivative_cutoff_hz, "derivative_cutoff_hz"),
        )


@dataclass(frozen=True)
class FilteredWorkspaceEstimate:
    """One filtered view of a raw live observation.

    ``raw_observation`` is deliberately embedded rather than copied into new
    mutable fields.  An audit reader can always compare the exact measured
    point with the filtered point that downstream logic consumed.
    """

    raw_observation: LiveWorkspaceObservation
    filtered_x_mm: float | None
    filtered_y_mm: float | None
    filter_applied: bool
    reset_reason: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.raw_observation, LiveWorkspaceObservation):
            raise TypeError("raw_observation must be a LiveWorkspaceObservation.")
        if not isinstance(self.filter_applied, bool):
            raise TypeError("filter_applied must be a boolean.")
        if self.raw_observation.valid:
            if self.filtered_x_mm is None or self.filtered_y_mm is None:
                raise ValueError("A valid raw observation requires filtered x/y.")
            for name in ("filtered_x_mm", "filtered_y_mm"):
                value = float(getattr(self, name))
                if not math.isfinite(value):
                    raise ValueError(f"{name} must be finite.")
                object.__setattr__(self, name, value)
        elif self.filtered_x_mm is not None or self.filtered_y_mm is not None:
            raise ValueError("An invalid raw observation cannot carry filtered coordinates.")
        if self.reset_reason is not None and (not isinstance(self.reset_reason, str) or not self.reset_reason):
            raise ValueError("reset_reason must be a non-empty string or None.")

    @property
    def source_position_mm(self) -> tuple[float, float] | None:
        if not self.raw_observation.valid:
            return None
        assert self.raw_observation.x_mm is not None and self.raw_observation.y_mm is not None
        return (self.raw_observation.x_mm, self.raw_observation.y_mm)

    @property
    def filtered_position_mm(self) -> tuple[float, float] | None:
        if not self.raw_observation.valid:
            return None
        assert self.filtered_x_mm is not None and self.filtered_y_mm is not None
        return (self.filtered_x_mm, self.filtered_y_mm)

    def to_realtime_task_event(self, source_index: int, demo_t_ms: float) -> RealtimeTaskEvent:
        """Adapt the filtered *position* while retaining raw timing/semantics."""
        raw = self.raw_observation
        return RealtimeTaskEvent(
            source_index=source_index,
            source_t_ms=demo_t_ms,
            replay_t_ms=raw.available_t_ms,
            valid=raw.valid,
            inside_workspace=raw.inside_workspace,
            x_mm=self.filtered_x_mm if raw.valid else None,
            y_mm=self.filtered_y_mm if raw.valid else None,
            pen_state=raw.pen_state,
            stroke_id=raw.stroke_id,
            invalid_reason=raw.invalid_reason,
        )

    def to_dict(self) -> dict[str, object]:
        """JSON audit record; raw is referenced by frame index, not rewritten."""
        return {
            "frame_index": self.raw_observation.frame_index,
            "capture_t_ms": self.raw_observation.capture_t_ms,
            "filtered_x_mm": self.filtered_x_mm,
            "filtered_y_mm": self.filtered_y_mm,
            "filter_applied": self.filter_applied,
            "reset_reason": self.reset_reason,
        }


class _OneEuroScalar:
    """One scalar One Euro state; kept private to prevent accidental reuse."""

    def __init__(self, config: OneEuroPositionFilterConfig) -> None:
        self._config = config
        self._last_t_ms: float | None = None
        self._last_raw: float | None = None
        self._x_hat: float | None = None
        self._dx_hat: float | None = None

    @staticmethod
    def _alpha(cutoff_hz: float, dt_s: float) -> float:
        tau_s = 1.0 / (2.0 * math.pi * cutoff_hz)
        return 1.0 / (1.0 + tau_s / dt_s)

    def reset(self) -> None:
        self._last_t_ms = None
        self._last_raw = None
        self._x_hat = None
        self._dx_hat = None

    def push(self, value: float, capture_t_ms: float) -> tuple[float, bool]:
        """Return (filtered_value, was_already_initialized)."""
        if self._last_t_ms is None:
            self._last_t_ms = capture_t_ms
            self._last_raw = value
            self._x_hat = value
            self._dx_hat = 0.0
            return value, False

        dt_s = (capture_t_ms - self._last_t_ms) / 1_000.0
        if dt_s <= 0.0:
            raise ValueError("One Euro filter requires strictly increasing capture_t_ms.")
        assert self._last_raw is not None and self._x_hat is not None and self._dx_hat is not None
        raw_derivative = (value - self._last_raw) / dt_s
        derivative_alpha = self._alpha(self._config.derivative_cutoff_hz, dt_s)
        self._dx_hat = derivative_alpha * raw_derivative + (1.0 - derivative_alpha) * self._dx_hat
        cutoff_hz = self._config.min_cutoff_hz + self._config.beta * abs(self._dx_hat)
        position_alpha = self._alpha(cutoff_hz, dt_s)
        self._x_hat = position_alpha * value + (1.0 - position_alpha) * self._x_hat
        self._last_t_ms = capture_t_ms
        self._last_raw = value
        return self._x_hat, True


class LivePositionFilter:
    """Apply a One Euro baseline to uninterrupted in-workspace live positions.

    The state resets at every invalid/outside barrier.  This is not merely a
    numerical convenience: allowing an old filtered value to leak across a
    no-hand gap would invent an unobserved bridge for a future planner.
    """

    def __init__(self, config: OneEuroPositionFilterConfig = OneEuroPositionFilterConfig()) -> None:
        if not isinstance(config, OneEuroPositionFilterConfig):
            raise TypeError("config must be a OneEuroPositionFilterConfig.")
        self.config = config
        self._x = _OneEuroScalar(config)
        self._y = _OneEuroScalar(config)
        self._last_frame_index: int | None = None

    def reset(self) -> None:
        self._x.reset()
        self._y.reset()
        self._last_frame_index = None

    def push(self, observation: LiveWorkspaceObservation) -> FilteredWorkspaceEstimate:
        """Create one derived estimate; the caller still owns the raw observation."""
        if not isinstance(observation, LiveWorkspaceObservation):
            raise TypeError("observation must be a LiveWorkspaceObservation.")
        if self._last_frame_index is not None and observation.frame_index <= self._last_frame_index:
            raise ValueError("Live position filter requires strictly increasing frame_index.")
        self._last_frame_index = observation.frame_index

        if not observation.valid:
            self._x.reset()
            self._y.reset()
            return FilteredWorkspaceEstimate(observation, None, None, False, "invalid_observation")
        assert observation.x_mm is not None and observation.y_mm is not None
        if not observation.inside_workspace:
            self._x.reset()
            self._y.reset()
            # Keep an outside measurement visible for audit/UI, but do not
            # use it to seed the next in-workspace run.
            return FilteredWorkspaceEstimate(
                observation,
                observation.x_mm,
                observation.y_mm,
                False,
                "outside_workspace",
            )

        x_mm, prior_x = self._x.push(observation.x_mm, observation.capture_t_ms)
        y_mm, prior_y = self._y.push(observation.y_mm, observation.capture_t_ms)
        return FilteredWorkspaceEstimate(
            observation,
            x_mm,
            y_mm,
            filter_applied=prior_x and prior_y,
            reset_reason=None if prior_x and prior_y else "new_continuous_run",
        )
