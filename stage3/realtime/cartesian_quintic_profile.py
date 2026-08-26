"""Local Cartesian quintic profiles derived from two timed waypoints."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real


Vector2 = tuple[float, float]


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite number.")
    numeric_value = float(value)
    if not math.isfinite(numeric_value):
        raise ValueError(f"{name} must be a finite number.")
    return numeric_value


def _finite_nonnegative(value: object, name: str) -> float:
    numeric_value = _finite(value, name)
    if numeric_value < 0.0:
        raise ValueError(f"{name} must be non-negative.")
    return numeric_value


def _vector2_or_zero(value: object, name: str) -> Vector2:
    if value is None:
        return (0.0, 0.0)
    if not isinstance(value, tuple) or len(value) != 2:
        raise TypeError(f"{name} must be a two-item tuple or None.")
    return (_finite(value[0], f"{name}[0]"), _finite(value[1], f"{name}[1]"))


@dataclass(frozen=True)
class CartesianWaypoint:
    """A timed workspace-mm Cartesian waypoint with optional boundary derivatives."""

    t_ms: float
    x_mm: float
    y_mm: float
    velocity_mm_s: Vector2 | None = None
    acceleration_mm_s2: Vector2 | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "t_ms", _finite_nonnegative(self.t_ms, "t_ms"))
        object.__setattr__(self, "x_mm", _finite(self.x_mm, "x_mm"))
        object.__setattr__(self, "y_mm", _finite(self.y_mm, "y_mm"))
        object.__setattr__(
            self,
            "velocity_mm_s",
            _vector2_or_zero(self.velocity_mm_s, "velocity_mm_s"),
        )
        object.__setattr__(
            self,
            "acceleration_mm_s2",
            _vector2_or_zero(self.acceleration_mm_s2, "acceleration_mm_s2"),
        )



@dataclass(frozen=True)
class CartesianTrajectorySample:
    """One evaluated Cartesian profile state in mm, mm/s, and mm/s²."""

    t_ms: float
    x_mm: float
    y_mm: float
    velocity_mm_s: Vector2
    acceleration_mm_s2: Vector2

    def __post_init__(self) -> None:
        object.__setattr__(self, "t_ms", _finite_nonnegative(self.t_ms, "t_ms"))
        object.__setattr__(self, "x_mm", _finite(self.x_mm, "x_mm"))
        object.__setattr__(self, "y_mm", _finite(self.y_mm, "y_mm"))
        object.__setattr__(
            self,
            "velocity_mm_s",
            _vector2_or_zero(self.velocity_mm_s, "velocity_mm_s"),
        )
        object.__setattr__(
            self,
            "acceleration_mm_s2",
            _vector2_or_zero(self.acceleration_mm_s2, "acceleration_mm_s2"),
        )


@dataclass(frozen=True)
class CartesianQuinticProfile:
    """An immutable two-axis quintic segment evaluable at any in-range time."""

    start: CartesianWaypoint
    end: CartesianWaypoint
    coefficients_x: tuple[float, float, float, float, float, float]
    coefficients_y: tuple[float, float, float, float, float, float]

    @property
    def duration_ms(self) -> float:
        return self.end.t_ms - self.start.t_ms

    @property
    def duration_s(self) -> float:
        return self.duration_ms / 1_000.0

    def sample_at(self, t_ms: float) -> CartesianTrajectorySample:
        """Evaluate position, velocity, and acceleration without extrapolation."""
        sample_time_ms = _finite_nonnegative(t_ms, "t_ms")
        if sample_time_ms < self.start.t_ms or sample_time_ms > self.end.t_ms:
            raise ValueError("t_ms must be within the profile time interval.")
        elapsed_s = (sample_time_ms - self.start.t_ms) / 1_000.0
        x_mm, velocity_x, acceleration_x = _evaluate(self.coefficients_x, elapsed_s)
        y_mm, velocity_y, acceleration_y = _evaluate(self.coefficients_y, elapsed_s)
        return CartesianTrajectorySample(
            t_ms=sample_time_ms,
            x_mm=x_mm,
            y_mm=y_mm,
            velocity_mm_s=(velocity_x, velocity_y),
            acceleration_mm_s2=(acceleration_x, acceleration_y),
        )

    def sample_uniform(self, sample_rate_hz: float = 100.0) -> tuple[CartesianTrajectorySample, ...]:
        """Evaluate at a uniform rate, appending the exact endpoint if required."""
        rate = _finite_nonnegative(sample_rate_hz, "sample_rate_hz")
        if rate == 0.0:
            raise ValueError("sample_rate_hz must be positive.")
        period_ms = 1_000.0 / rate
        sample_count = int(math.floor(self.duration_ms / period_ms))
        times = [self.start.t_ms + index * period_ms for index in range(sample_count + 1)]
        if times[-1] < self.end.t_ms:
            times.append(self.end.t_ms)
        return tuple(self.sample_at(t_ms) for t_ms in times)


class CartesianQuinticProfileGenerator:
    """Create the minimal position/velocity/acceleration-bounded Cartesian segment."""

    def build(self, start: CartesianWaypoint, end: CartesianWaypoint) -> CartesianQuinticProfile:
        """Build one profile from explicit p/v/a endpoint states."""
        if not isinstance(start, CartesianWaypoint) or not isinstance(end, CartesianWaypoint):
            raise TypeError("start and end must be CartesianWaypoint instances.")
        start_waypoint = start
        end_waypoint = end
        if end_waypoint.t_ms <= start_waypoint.t_ms:
            raise ValueError("end.t_ms must be greater than start.t_ms.")
        duration_s = (end_waypoint.t_ms - start_waypoint.t_ms) / 1_000.0
        return CartesianQuinticProfile(
            start=start_waypoint,
            end=end_waypoint,
            coefficients_x=_quintic_coefficients(
                start_waypoint.x_mm,
                start_waypoint.velocity_mm_s[0],
                start_waypoint.acceleration_mm_s2[0],
                end_waypoint.x_mm,
                end_waypoint.velocity_mm_s[0],
                end_waypoint.acceleration_mm_s2[0],
                duration_s,
            ),
            coefficients_y=_quintic_coefficients(
                start_waypoint.y_mm,
                start_waypoint.velocity_mm_s[1],
                start_waypoint.acceleration_mm_s2[1],
                end_waypoint.y_mm,
                end_waypoint.velocity_mm_s[1],
                end_waypoint.acceleration_mm_s2[1],
                duration_s,
            ),
        )


def _quintic_coefficients(
    start_position: float,
    start_velocity: float,
    start_acceleration: float,
    end_position: float,
    end_velocity: float,
    end_acceleration: float,
    duration_s: float,
) -> tuple[float, float, float, float, float, float]:
    c0 = start_position
    c1 = start_velocity
    c2 = start_acceleration / 2.0
    position_residual = end_position - (c0 + c1 * duration_s + c2 * duration_s**2)
    velocity_residual = end_velocity - (c1 + 2.0 * c2 * duration_s)
    acceleration_residual = end_acceleration - 2.0 * c2
    c3 = (
        10.0 * position_residual
        - 4.0 * velocity_residual * duration_s
        + 0.5 * acceleration_residual * duration_s**2
    ) / duration_s**3
    c4 = (
        -15.0 * position_residual
        + 7.0 * velocity_residual * duration_s
        - acceleration_residual * duration_s**2
    ) / duration_s**4
    c5 = (
        6.0 * position_residual
        - 3.0 * velocity_residual * duration_s
        + 0.5 * acceleration_residual * duration_s**2
    ) / duration_s**5
    return (c0, c1, c2, c3, c4, c5)


def _evaluate(
    coefficients: tuple[float, float, float, float, float, float],
    elapsed_s: float,
) -> tuple[float, float, float]:
    c0, c1, c2, c3, c4, c5 = coefficients
    position = c0 + c1 * elapsed_s + c2 * elapsed_s**2 + c3 * elapsed_s**3 + c4 * elapsed_s**4 + c5 * elapsed_s**5
    velocity = c1 + 2.0 * c2 * elapsed_s + 3.0 * c3 * elapsed_s**2 + 4.0 * c4 * elapsed_s**3 + 5.0 * c5 * elapsed_s**4
    acceleration = 2.0 * c2 + 6.0 * c3 * elapsed_s + 12.0 * c4 * elapsed_s**2 + 20.0 * c5 * elapsed_s**3
    return position, velocity, acceleration
