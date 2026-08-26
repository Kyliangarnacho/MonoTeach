"""Semantic and clock contracts shared by future Stage 3.4 stream modules.

The source clock describes the demonstration. A later execution backend is
free to allocate its own clock, but it must keep the source clock as evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real

from .realtime_sample import RealtimeWorkspaceSample


class MotionMode(str, Enum):
    """Task meaning of an eligible target, independent from robot execution."""

    DRAW = "DRAW"
    LIFTED_FOLLOW = "LIFTED_FOLLOW"
    HOLD = "HOLD"
    TERMINAL_STOP = "TERMINAL_STOP"
    TRANSITION_REQUIRED = "TRANSITION_REQUIRED"


class BarrierType(str, Enum):
    """Reasons that must not be silently connected by a streaming planner."""

    OBSERVATION_INVALID = "OBSERVATION_INVALID"
    OUTSIDE_WORKSPACE = "OUTSIDE_WORKSPACE"
    STALE_INPUT = "STALE_INPUT"
    IK_FAILURE = "IK_FAILURE"
    JOINT_LIMIT = "JOINT_LIMIT"


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite non-negative number.")
    numeric_value = float(value)
    if not math.isfinite(numeric_value) or numeric_value < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number.")
    return numeric_value


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite number.")
    numeric_value = float(value)
    if not math.isfinite(numeric_value):
        raise ValueError(f"{name} must be a finite number.")
    return numeric_value


def _source_index(value: object) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError("source_index must be a non-negative integer.")
    return value


@dataclass(frozen=True)
class RealtimeTaskEvent:
    """One published observation with task semantics and separate source clocks.

    ``source_t_ms`` is the canonical demonstration timestamp. ``replay_t_ms``
    records when the virtual or live source made the event visible. Neither is
    an execution deadline for the robot.
    """

    source_index: int
    source_t_ms: float
    replay_t_ms: float
    valid: bool
    inside_workspace: bool
    x_mm: float | None
    y_mm: float | None
    pen_state: str = "DOWN"
    stroke_id: int | None = None
    invalid_reason: str | None = None
    # Kept last for positional compatibility with frozen Stage 3.4 callers.
    # It is expressed in the task-plane frame, never robot-base Z.
    z_task_mm: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "source_index", _source_index(self.source_index))
        object.__setattr__(self, "source_t_ms", _finite_nonnegative(self.source_t_ms, "source_t_ms"))
        object.__setattr__(self, "replay_t_ms", _finite_nonnegative(self.replay_t_ms, "replay_t_ms"))
        if not isinstance(self.valid, bool) or not isinstance(self.inside_workspace, bool):
            raise TypeError("valid and inside_workspace must be booleans.")
        if self.pen_state not in {"DOWN", "UP"}:
            raise ValueError("pen_state must be DOWN or UP.")
        object.__setattr__(self, "z_task_mm", _finite(self.z_task_mm, "z_task_mm"))
        if self.stroke_id is not None and (
            not isinstance(self.stroke_id, int)
            or isinstance(self.stroke_id, bool)
            or self.stroke_id < 1
        ):
            raise ValueError("stroke_id must be a positive integer or None.")
        if self.pen_state == "UP" and self.stroke_id is not None:
            raise ValueError("UP events must not carry stroke_id.")

        if self.valid:
            if self.x_mm is None or self.y_mm is None:
                raise ValueError("A valid task event must include x_mm and y_mm.")
            if self.invalid_reason is not None:
                raise ValueError("A valid task event cannot carry invalid_reason.")
            object.__setattr__(self, "x_mm", _finite(self.x_mm, "x_mm"))
            object.__setattr__(self, "y_mm", _finite(self.y_mm, "y_mm"))
        else:
            if self.x_mm is not None or self.y_mm is not None:
                raise ValueError("An invalid task event cannot include x_mm or y_mm.")
            if self.inside_workspace:
                raise ValueError("An invalid task event cannot be inside_workspace.")
            if not isinstance(self.invalid_reason, str) or not self.invalid_reason:
                raise ValueError("An invalid task event requires invalid_reason.")

    @classmethod
    def from_realtime_workspace_sample(
        cls,
        sample: RealtimeWorkspaceSample,
    ) -> "RealtimeTaskEvent":
        """Adapt an immutable Stage 3.4A arrival without changing its source."""
        if not isinstance(sample, RealtimeWorkspaceSample):
            raise TypeError("sample must be a RealtimeWorkspaceSample.")
        workspace = sample.workspace_sample
        return cls(
            source_index=sample.source_index,
            source_t_ms=sample.t_ms,
            replay_t_ms=sample.replay_time_ms,
            valid=workspace.valid,
            inside_workspace=workspace.inside_workspace,
            x_mm=workspace.x_mm,
            y_mm=workspace.y_mm,
            z_task_mm=0.0,
            pen_state=workspace.pen_state or "DOWN",
            stroke_id=workspace.stroke_id,
            invalid_reason=workspace.invalid_reason,
        )

    @property
    def planning_eligible(self) -> bool:
        """Only valid in-workspace observations may enter Cartesian planning."""
        return self.valid and self.inside_workspace

    @property
    def motion_mode(self) -> MotionMode:
        """Map pen semantics without treating PEN_UP as a perception barrier."""
        if not self.planning_eligible:
            return MotionMode.HOLD
        return MotionMode.DRAW if self.pen_state == "DOWN" else MotionMode.LIFTED_FOLLOW

    @property
    def observation_barrier_type(self) -> BarrierType | None:
        if not self.valid:
            return BarrierType.OBSERVATION_INVALID
        if not self.inside_workspace:
            return BarrierType.OUTSIDE_WORKSPACE
        return None


@dataclass(frozen=True)
class StreamBarrier:
    """An explicit non-motion event retained for later planner/backend handling."""

    barrier_type: BarrierType
    source_index: int | None
    source_t_ms: float | None
    replay_t_ms: float | None
    reason: str

    def __post_init__(self) -> None:
        if not isinstance(self.barrier_type, BarrierType):
            raise TypeError("barrier_type must be a BarrierType.")
        if self.source_index is not None:
            object.__setattr__(self, "source_index", _source_index(self.source_index))
        for name in ("source_t_ms", "replay_t_ms"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _finite_nonnegative(value, name))
        if not isinstance(self.reason, str) or not self.reason:
            raise ValueError("reason must be a non-empty string.")

    @classmethod
    def from_task_event(cls, event: RealtimeTaskEvent) -> "StreamBarrier":
        if not isinstance(event, RealtimeTaskEvent):
            raise TypeError("event must be a RealtimeTaskEvent.")
        barrier_type = event.observation_barrier_type
        if barrier_type is None:
            raise ValueError("Only an observation barrier event can create StreamBarrier.")
        reason = event.invalid_reason or "outside_workspace"
        return cls(
            barrier_type=barrier_type,
            source_index=event.source_index,
            source_t_ms=event.source_t_ms,
            replay_t_ms=event.replay_t_ms,
            reason=reason,
        )


@dataclass(frozen=True)
class SegmentTimeContract:
    """Keep source, planning availability, and execution clocks intentionally separate."""

    source_start_t_ms: float
    source_end_t_ms: float
    available_replay_t_ms: float
    execution_start_s: float | None = None
    execution_duration_s: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "source_start_t_ms", _finite_nonnegative(self.source_start_t_ms, "source_start_t_ms"))
        object.__setattr__(self, "source_end_t_ms", _finite_nonnegative(self.source_end_t_ms, "source_end_t_ms"))
        object.__setattr__(self, "available_replay_t_ms", _finite_nonnegative(self.available_replay_t_ms, "available_replay_t_ms"))
        if self.source_end_t_ms <= self.source_start_t_ms:
            raise ValueError("source_end_t_ms must be greater than source_start_t_ms.")
        if (self.execution_start_s is None) != (self.execution_duration_s is None):
            raise ValueError("execution_start_s and execution_duration_s must be supplied together.")
        if self.execution_start_s is not None:
            object.__setattr__(self, "execution_start_s", _finite_nonnegative(self.execution_start_s, "execution_start_s"))
            duration_s = _finite(self.execution_duration_s, "execution_duration_s")
            if duration_s <= 0.0:
                raise ValueError("execution_duration_s must be positive.")
            object.__setattr__(self, "execution_duration_s", duration_s)

    @property
    def execution_end_s(self) -> float | None:
        if self.execution_start_s is None or self.execution_duration_s is None:
            return None
        return self.execution_start_s + self.execution_duration_s
