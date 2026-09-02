"""Immutable Task 5 contracts from a semantic behavior to a nominal motion plan."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real

from .personality_contracts import BehaviorEvent


MOTION_PLAN_SCHEMA_VERSION = "banter_motion_plan_v1"


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite non-negative number.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number.")
    return numeric


def _positive_ms(value: object, name: str) -> float:
    numeric = _finite_nonnegative(value, name)
    if numeric <= 0.0:
        raise ValueError(f"{name} must be a finite positive duration in milliseconds.")
    return numeric


def _identifier(value: object, name: str) -> str:
    if not isinstance(value, str) or not value or not value.isidentifier() or value != value.upper():
        raise ValueError(f"{name} must be a non-empty uppercase identifier.")
    return value


class MotionStepKind(str, Enum):
    """The deliberately small serial vocabulary for an expressive macro."""

    MOVE_POSE = "MOVE_POSE"
    HOLD = "HOLD"
    RETURN_NEUTRAL = "RETURN_NEUTRAL"


@dataclass(frozen=True)
class MotionStep:
    """One named pose target and its nominal duration; never joint coordinates."""

    kind: MotionStepKind
    target_pose_name: str
    duration_ms: float

    def __post_init__(self) -> None:
        if not isinstance(self.kind, MotionStepKind):
            raise TypeError("kind must be a MotionStepKind.")
        object.__setattr__(self, "target_pose_name", _identifier(self.target_pose_name, "target_pose_name"))
        object.__setattr__(self, "duration_ms", _positive_ms(self.duration_ms, "duration_ms"))
        if self.kind is MotionStepKind.RETURN_NEUTRAL and self.target_pose_name != "NEUTRAL":
            raise ValueError("RETURN_NEUTRAL must target NEUTRAL.")

    def to_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind.value,
            "target_pose_name": self.target_pose_name,
            "duration_ms": self.duration_ms,
        }


@dataclass(frozen=True)
class MotionMacro:
    """The one named action that belongs to one Task 4 behavior name."""

    behavior_name: str
    macro_name: str
    steps: tuple[MotionStep, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "behavior_name", _identifier(self.behavior_name, "behavior_name"))
        object.__setattr__(self, "macro_name", _identifier(self.macro_name, "macro_name"))
        if not isinstance(self.steps, tuple) or not self.steps or not all(isinstance(step, MotionStep) for step in self.steps):
            raise ValueError("steps must be a non-empty tuple of MotionStep values.")
        if self.steps[-1].kind is not MotionStepKind.RETURN_NEUTRAL:
            raise ValueError("Every MotionMacro must end with RETURN_NEUTRAL.")


@dataclass(frozen=True)
class MotionPlan:
    """A JSON-safe immutable expansion of one behavior; no trajectory is embedded."""

    plan_id: int
    source_behavior: BehaviorEvent
    macro_name: str
    motion_steps: tuple[MotionStep, ...]
    available_t_ms: float
    schema_version: str = MOTION_PLAN_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if not isinstance(self.plan_id, int) or isinstance(self.plan_id, bool) or self.plan_id < 1:
            raise ValueError("plan_id must be a positive integer.")
        if not isinstance(self.source_behavior, BehaviorEvent):
            raise TypeError("source_behavior must be a BehaviorEvent.")
        object.__setattr__(self, "macro_name", _identifier(self.macro_name, "macro_name"))
        if not isinstance(self.motion_steps, tuple) or not self.motion_steps or not all(isinstance(step, MotionStep) for step in self.motion_steps):
            raise ValueError("motion_steps must be a non-empty tuple of MotionStep values.")
        if self.motion_steps[-1].kind is not MotionStepKind.RETURN_NEUTRAL:
            raise ValueError("MotionPlan must end with RETURN_NEUTRAL.")
        available = _finite_nonnegative(self.available_t_ms, "available_t_ms")
        if available < self.source_behavior.available_t_ms:
            raise ValueError("MotionPlan availability must not precede source BehaviorEvent availability.")
        if self.schema_version != MOTION_PLAN_SCHEMA_VERSION:
            raise ValueError(f"Unsupported MotionPlan schema_version: {self.schema_version!r}")
        object.__setattr__(self, "available_t_ms", available)

    @property
    def nominal_duration_ms(self) -> float:
        return sum(step.duration_ms for step in self.motion_steps)

    def to_dict(self) -> dict[str, object]:
        """Return the cross-language fixture form consumed by the MATLAB compiler."""
        return {
            "schema_version": self.schema_version,
            "plan_id": self.plan_id,
            "source_behavior": self.source_behavior.to_dict(),
            "macro_name": self.macro_name,
            "variant": self.source_behavior.variant,
            "motion_steps": [step.to_dict() for step in self.motion_steps],
            "nominal_duration_ms": self.nominal_duration_ms,
            "available_t_ms": self.available_t_ms,
        }
