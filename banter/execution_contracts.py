"""Immutable Task 7 contracts for one non-preemptive Banter execution."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .motion_style_contracts import StyledMotionPlan
from .personality_contracts import BehaviorEvent


EXECUTION_SCHEMA_VERSION = "banter_execution_v1"


class ExecutionState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    READY = "READY"
    DISPATCHED = "DISPATCHED"
    EXECUTING = "EXECUTING"
    FAULTED = "FAULTED"
    SHUTTING_DOWN = "SHUTTING_DOWN"
    CLOSED = "CLOSED"


def _positive_id(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 1:
        raise ValueError(f"{name} must be a positive integer.")
    return value


@dataclass(frozen=True)
class ExecutionCommand:
    """One fully-provenanced command sent after Python has acquired busy."""

    execution_id: int
    behavior: BehaviorEvent
    styled_motion_plan: StyledMotionPlan
    schema_version: str = EXECUTION_SCHEMA_VERSION

    def __post_init__(self) -> None:
        _positive_id(self.execution_id, "execution_id")
        if not isinstance(self.behavior, BehaviorEvent):
            raise TypeError("behavior must be a BehaviorEvent.")
        if not isinstance(self.styled_motion_plan, StyledMotionPlan):
            raise TypeError("styled_motion_plan must be a StyledMotionPlan.")
        source = self.styled_motion_plan.source_motion_plan.source_behavior
        if source != self.behavior:
            raise ValueError("StyledMotionPlan provenance must match the execution BehaviorEvent.")
        if self.schema_version != EXECUTION_SCHEMA_VERSION:
            raise ValueError(f"Unsupported execution schema_version: {self.schema_version!r}")

    @property
    def session_id(self) -> str:
        return self.behavior.session_id

    def to_payload(self) -> dict[str, object]:
        plan = self.styled_motion_plan
        return {
            "execution_id": self.execution_id,
            "behavior_id": self.behavior.behavior_id,
            "plan_id": plan.source_motion_plan.plan_id,
            "styled_plan_id": plan.styled_plan_id,
            "behavior_name": self.behavior.behavior_name,
            "variant": self.behavior.variant,
            "macro_name": plan.source_motion_plan.macro_name,
            "styled_motion_plan": plan.to_dict(),
        }
