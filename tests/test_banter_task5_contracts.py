"""Contract tests for Task 5 semantic MotionPlans."""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from banter.motion_contracts import MotionMacro, MotionPlan, MotionStep, MotionStepKind
from banter.personality_contracts import BehaviorEvent


def _behavior(*, available_t_ms: float = 200.0) -> BehaviorEvent:
    return BehaviorEvent(1, "task5", 1, "FIST_COUNTER", "DEFAULT", (1,), (), 100.0, available_t_ms)


def test_motion_plan_is_immutable_json_safe_and_has_no_intensity() -> None:
    step = MotionStep(MotionStepKind.RETURN_NEUTRAL, "NEUTRAL", 300.0)
    plan = MotionPlan(1, _behavior(), "COUNTER_JAB", (step,), 250.0)
    payload = plan.to_dict()
    assert payload["variant"] == "DEFAULT"
    assert payload["source_behavior"]["behavior_name"] == "FIST_COUNTER"
    assert "intensity" not in payload
    with pytest.raises(FrozenInstanceError):
        plan.macro_name = "OTHER"  # type: ignore[misc]


def test_motion_contract_rejects_invalid_return_or_availability() -> None:
    with pytest.raises(ValueError, match="RETURN_NEUTRAL"):
        MotionStep(MotionStepKind.RETURN_NEUTRAL, "AIM", 100.0)
    with pytest.raises(ValueError, match="end with RETURN_NEUTRAL"):
        MotionMacro("FIST_COUNTER", "COUNTER_JAB", (MotionStep(MotionStepKind.MOVE_POSE, "AIM", 100.0),))
    with pytest.raises(ValueError, match="must not precede"):
        MotionPlan(1, _behavior(), "COUNTER_JAB", (MotionStep(MotionStepKind.RETURN_NEUTRAL, "NEUTRAL", 100.0),), 199.0)
