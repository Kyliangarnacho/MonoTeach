"""Immutable Task 6 contracts for discrete motion styling.

Task 5 remains the canonical nominal action.  Task 6 derives a new plan with
the same action identity and pose sequence, changing only per-step timing
selected by an already-discrete variant.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

from .motion_contracts import MotionPlan, MotionStep, MotionStepKind


MOTION_STYLE_SCHEMA_VERSION = "banter_motion_style_v1"
STYLED_MOTION_PLAN_SCHEMA_VERSION = "banter_styled_motion_plan_v1"


def _identifier(value: object, name: str) -> str:
    if not isinstance(value, str) or not value or not value.isidentifier() or value != value.upper():
        raise ValueError(f"{name} must be a non-empty uppercase identifier.")
    return value


def _positive(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite positive number.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{name} must be a finite positive number.")
    return numeric


def _nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite non-negative number.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number.")
    return numeric


@dataclass(frozen=True)
class MotionStyleProfile:
    """One fixed discrete style; it is never a live continuous intensity."""

    variant: str
    move_duration_scale: float = 1.0
    hold_duration_scale: float = 1.0
    return_duration_scale: float = 1.0
    schema_version: str = MOTION_STYLE_SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(self, "variant", _identifier(self.variant, "variant"))
        for name in ("move_duration_scale", "hold_duration_scale", "return_duration_scale"):
            object.__setattr__(self, name, _positive(getattr(self, name), name))
        if self.schema_version != MOTION_STYLE_SCHEMA_VERSION:
            raise ValueError(f"Unsupported MotionStyleProfile schema_version: {self.schema_version!r}")

    def duration_scale_for(self, kind: MotionStepKind) -> float:
        if not isinstance(kind, MotionStepKind):
            raise TypeError("kind must be a MotionStepKind.")
        if kind is MotionStepKind.MOVE_POSE:
            return self.move_duration_scale
        if kind is MotionStepKind.HOLD:
            return self.hold_duration_scale
        return self.return_duration_scale

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "variant": self.variant,
            "move_duration_scale": self.move_duration_scale,
            "hold_duration_scale": self.hold_duration_scale,
            "return_duration_scale": self.return_duration_scale,
        }


@dataclass(frozen=True)
class MotionStyleOverride:
    """An explicit high-emotion override for one semantic behavior only."""

    behavior_name: str
    profile: MotionStyleProfile

    def __post_init__(self) -> None:
        object.__setattr__(self, "behavior_name", _identifier(self.behavior_name, "behavior_name"))
        if not isinstance(self.profile, MotionStyleProfile):
            raise TypeError("profile must be a MotionStyleProfile.")
        if self.profile.variant == "DEFAULT":
            raise ValueError("MotionStyleOverride is only for a non-DEFAULT variant.")

    @property
    def key(self) -> tuple[str, str]:
        return self.behavior_name, self.profile.variant


@dataclass(frozen=True)
class StyledMotionPlan:
    """A derived, provenance-preserving Task 6 plan with changed timings only."""

    styled_plan_id: int
    source_motion_plan: MotionPlan
    style: MotionStyleProfile
    motion_steps: tuple[MotionStep, ...]
    available_t_ms: float
    schema_version: str = STYLED_MOTION_PLAN_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if not isinstance(self.styled_plan_id, int) or isinstance(self.styled_plan_id, bool) or self.styled_plan_id < 1:
            raise ValueError("styled_plan_id must be a positive integer.")
        if not isinstance(self.source_motion_plan, MotionPlan):
            raise TypeError("source_motion_plan must be a MotionPlan.")
        if not isinstance(self.style, MotionStyleProfile):
            raise TypeError("style must be a MotionStyleProfile.")
        if self.style.variant != self.source_motion_plan.source_behavior.variant:
            raise ValueError("Style variant must match source MotionPlan behavior variant.")
        if (not isinstance(self.motion_steps, tuple) or not self.motion_steps
                or not all(isinstance(step, MotionStep) for step in self.motion_steps)):
            raise ValueError("motion_steps must be a non-empty tuple of MotionStep values.")
        if len(self.motion_steps) != len(self.source_motion_plan.motion_steps):
            raise ValueError("Styled MotionPlan must retain the source step count.")
        for source, styled in zip(self.source_motion_plan.motion_steps, self.motion_steps):
            if source.kind is not styled.kind or source.target_pose_name != styled.target_pose_name:
                raise ValueError("Task 6 may change only MotionStep durations, never Macro topology or pose names.")
            expected = source.duration_ms * self.style.duration_scale_for(source.kind)
            if not math.isclose(styled.duration_ms, expected, rel_tol=1.0e-12, abs_tol=1.0e-12):
                raise ValueError("Styled MotionStep duration must equal the selected fixed style scale.")
        if self.motion_steps[-1].kind is not MotionStepKind.RETURN_NEUTRAL:
            raise ValueError("Styled MotionPlan must end with RETURN_NEUTRAL.")
        available = _nonnegative(self.available_t_ms, "available_t_ms")
        if available < self.source_motion_plan.available_t_ms:
            raise ValueError("Styled MotionPlan availability must not precede its MotionPlan input.")
        if self.schema_version != STYLED_MOTION_PLAN_SCHEMA_VERSION:
            raise ValueError(f"Unsupported StyledMotionPlan schema_version: {self.schema_version!r}")
        object.__setattr__(self, "available_t_ms", available)

    @property
    def nominal_duration_ms(self) -> float:
        return sum(step.duration_ms for step in self.motion_steps)

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "styled_plan_id": self.styled_plan_id,
            "source_motion_plan": self.source_motion_plan.to_dict(),
            "macro_name": self.source_motion_plan.macro_name,
            "behavior_name": self.source_motion_plan.source_behavior.behavior_name,
            "variant": self.style.variant,
            "style": self.style.to_dict(),
            "motion_steps": [step.to_dict() for step in self.motion_steps],
            "nominal_duration_ms": self.nominal_duration_ms,
            "available_t_ms": self.available_t_ms,
        }
