"""Immutable evidence emitted by one real camera frame.

This module deliberately separates a camera frame's capture time from the
time at which its workspace result becomes available.  The former is the only
clock later allowed to describe demonstrated human motion; the latter is
latency evidence for an online system.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from numbers import Real

from stage2.workspace_trajectory import WorkspaceTrajectorySample

from .stream_contract import RealtimeTaskEvent


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite non-negative number.")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number.")
    return numeric


def _optional_pixel(value: object, name: str) -> tuple[float, float] | None:
    if value is None:
        return None
    if not isinstance(value, tuple) or len(value) != 2:
        raise TypeError(f"{name} must be a two-item tuple or None.")
    first, second = (float(component) for component in value)
    if not math.isfinite(first) or not math.isfinite(second):
        raise ValueError(f"{name} must contain finite values.")
    return (first, second)


@dataclass(frozen=True)
class LiveWorkspaceObservation:
    """One processed camera frame, before the formal demo-time boundary.

    ``capture_t_ms`` and ``available_t_ms`` are both relative to the current
    live session's monotonic-clock origin.  PREPARE observations use exactly
    this same contract; they are not secretly rewritten as tracking points.
    """

    frame_index: int
    capture_t_ms: float
    available_t_ms: float
    valid: bool
    x_mm: float | None
    y_mm: float | None
    inside_workspace: bool
    invalid_reason: str | None
    raw_pixel_xy: tuple[float, float] | None
    undistorted_pixel_xy: tuple[float, float] | None
    pen_state: str
    stroke_id: int | None
    control_gesture: str = "UNAVAILABLE"
    control_gesture_active: bool = False
    # True on the single frame that actually toggled the pen controller.  A
    # held L pose is not repeated as a spatial barrier every camera frame.
    control_gesture_toggled: bool = False
    # ``candidate`` starts before a command has passed its dwell gate.  It
    # lets the follow session protect the hand-shape transition from becoming
    # a spurious spatial barrier without treating it as a pen toggle yet.
    control_gesture_candidate: bool = False
    control_gesture_armed: bool = False
    control_gesture_candidate_elapsed_ms: float = 0.0
    control_gesture_dropout_grace_remaining_ms: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.frame_index, int) or isinstance(self.frame_index, bool) or self.frame_index < 0:
            raise ValueError("frame_index must be a non-negative integer.")
        object.__setattr__(self, "capture_t_ms", _finite_nonnegative(self.capture_t_ms, "capture_t_ms"))
        object.__setattr__(self, "available_t_ms", _finite_nonnegative(self.available_t_ms, "available_t_ms"))
        if self.available_t_ms < self.capture_t_ms:
            raise ValueError("available_t_ms must not precede capture_t_ms.")
        if not isinstance(self.valid, bool) or not isinstance(self.inside_workspace, bool):
            raise TypeError("valid and inside_workspace must be booleans.")
        if self.pen_state not in {"DOWN", "UP"}:
            raise ValueError("pen_state must be DOWN or UP.")
        if not isinstance(self.control_gesture, str) or not self.control_gesture:
            raise ValueError("control_gesture must be a non-empty string.")
        if not isinstance(self.control_gesture_active, bool):
            raise TypeError("control_gesture_active must be a boolean.")
        if not isinstance(self.control_gesture_candidate, bool) or not isinstance(self.control_gesture_armed, bool):
            raise TypeError("control_gesture_candidate and control_gesture_armed must be booleans.")
        for name in ("control_gesture_candidate_elapsed_ms", "control_gesture_dropout_grace_remaining_ms"):
            object.__setattr__(self, name, _finite_nonnegative(getattr(self, name), name))
        if not isinstance(self.control_gesture_toggled, bool):
            raise TypeError("control_gesture_toggled must be a boolean.")
        if self.pen_state == "UP" and self.stroke_id is not None:
            raise ValueError("UP observations must not carry stroke_id.")
        if self.stroke_id is not None and (
            not isinstance(self.stroke_id, int) or isinstance(self.stroke_id, bool) or self.stroke_id < 1
        ):
            raise ValueError("stroke_id must be a positive integer or None.")
        object.__setattr__(self, "raw_pixel_xy", _optional_pixel(self.raw_pixel_xy, "raw_pixel_xy"))
        object.__setattr__(
            self,
            "undistorted_pixel_xy",
            _optional_pixel(self.undistorted_pixel_xy, "undistorted_pixel_xy"),
        )
        if self.valid:
            if self.x_mm is None or self.y_mm is None or self.invalid_reason is not None:
                raise ValueError("A valid observation requires x/y and cannot carry invalid_reason.")
            if self.raw_pixel_xy is None or self.undistorted_pixel_xy is None:
                raise ValueError("A valid observation requires raw and undistorted pixels.")
            object.__setattr__(self, "x_mm", float(self.x_mm))
            object.__setattr__(self, "y_mm", float(self.y_mm))
            if not math.isfinite(self.x_mm) or not math.isfinite(self.y_mm):
                raise ValueError("x_mm and y_mm must be finite.")
        else:
            if any(value is not None for value in (self.x_mm, self.y_mm, self.raw_pixel_xy, self.undistorted_pixel_xy)):
                raise ValueError("An invalid observation cannot include workspace or pixel coordinates.")
            if self.inside_workspace:
                raise ValueError("An invalid observation cannot be inside_workspace.")
            if not isinstance(self.invalid_reason, str) or not self.invalid_reason:
                raise ValueError("An invalid observation requires invalid_reason.")

    @property
    def processing_latency_ms(self) -> float:
        """Wall-clock processing evidence; never use this as human-motion time."""
        return self.available_t_ms - self.capture_t_ms

    def to_workspace_sample(self, demo_t_ms: float) -> WorkspaceTrajectorySample:
        """Make a derived workspace sample while preserving live pen semantics.

        Stage 2's frozen WorkspaceTrajectory JSON can represent valid DOWN
        samples only.  Consequently this helper is intentionally limited to
        that legacy subset; the full UP/DOWN truth is retained in the new live
        session trace and in :meth:`to_realtime_task_event`.
        """
        if self.valid and self.pen_state != "DOWN":
            raise ValueError(
                "Stage 2 WorkspaceTrajectory cannot encode a valid UP sample; "
                "use the live session trace instead of discarding pen semantics."
            )
        return WorkspaceTrajectorySample(
            t_ms=demo_t_ms,
            valid=self.valid,
            x_mm=self.x_mm,
            y_mm=self.y_mm,
            inside_workspace=self.inside_workspace,
            invalid_reason=self.invalid_reason,
            pen_state=self.pen_state,
            stroke_id=self.stroke_id,
        )

    def to_realtime_task_event(self, source_index: int, demo_t_ms: float) -> RealtimeTaskEvent:
        """Adapt a formal tracking observation to the existing Stage 3.4 input.

        ``RealtimeTaskEvent.replay_t_ms`` is a historical name.  For a live
        source we store the *availability* instant there; source motion timing
        still comes exclusively from ``demo_t_ms``.
        """
        return RealtimeTaskEvent(
            source_index=source_index,
            source_t_ms=demo_t_ms,
            replay_t_ms=self.available_t_ms,
            valid=self.valid,
            inside_workspace=self.inside_workspace,
            x_mm=self.x_mm,
            y_mm=self.y_mm,
            pen_state=self.pen_state,
            stroke_id=self.stroke_id,
            invalid_reason=self.invalid_reason,
        )

    def to_dict(self) -> dict[str, object]:
        """Produce JSON-safe audit data without landmarks or raw image frames."""
        return asdict(self) | {"processing_latency_ms": self.processing_latency_ms}
