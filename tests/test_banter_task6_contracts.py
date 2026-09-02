"""Task 6 immutable, timing-only styling contracts."""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from banter.motion_contracts import MotionPlan, MotionStep, MotionStepKind
from banter.motion_style_contracts import MotionStyleProfile, StyledMotionPlan
from banter.personality_contracts import BehaviorEvent


def _plan() -> MotionPlan:
    behavior = BehaviorEvent(1, "task6-contract", 1, "FIST_COUNTER", "FIRM", (1,), (), 100.0, 200.0)
    return MotionPlan(
        1, behavior, "COUNTER_JAB",
        (MotionStep(MotionStepKind.MOVE_POSE, "RECOIL", 300.0),
         MotionStep(MotionStepKind.HOLD, "JAB_EXTEND", 250.0),
         MotionStep(MotionStepKind.RETURN_NEUTRAL, "NEUTRAL", 500.0)),
        250.0,
    )


def test_style_profile_is_discrete_timing_only_and_json_safe() -> None:
    profile = MotionStyleProfile("FIRM", 0.9, 1.0, 0.95)
    payload = profile.to_dict()
    assert payload["variant"] == "FIRM"
    assert "intensity" not in payload
    assert "pose_excursion_scale" not in payload
    with pytest.raises(FrozenInstanceError):
        profile.move_duration_scale = 0.5  # type: ignore[misc]


def test_styled_plan_preserves_macro_geometry_and_requires_exact_scaled_durations() -> None:
    source = _plan()
    profile = MotionStyleProfile("FIRM", 0.9, 1.0, 0.95)
    styled_steps = tuple(
        MotionStep(step.kind, step.target_pose_name, step.duration_ms * profile.duration_scale_for(step.kind))
        for step in source.motion_steps
    )
    styled = StyledMotionPlan(1, source, profile, styled_steps, 300.0)
    assert [item.target_pose_name for item in styled.motion_steps] == [item.target_pose_name for item in source.motion_steps]
    assert styled.source_motion_plan is source
    assert styled.to_dict()["source_motion_plan"]["macro_name"] == "COUNTER_JAB"
    bad_steps = (MotionStep(MotionStepKind.MOVE_POSE, "AIM", 270.0), *styled_steps[1:])
    with pytest.raises(ValueError, match="pose names"):
        StyledMotionPlan(2, source, profile, bad_steps, 300.0)
