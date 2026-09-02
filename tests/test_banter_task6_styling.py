"""Task 6 compact vocabulary and FIST-only style override tests."""

from __future__ import annotations

import pytest

from banter.motion_library import MotionPlanner
from banter.motion_style_contracts import MotionStyleOverride, MotionStyleProfile
from banter.motion_styling import MotionStyleConfig, MotionStyler
from banter.personality_contracts import BEHAVIOR_VARIANT_VOCABULARY, BehaviorEvent, PersonaConfig


def _behavior(identifier: int, name: str, variant: str) -> BehaviorEvent:
    return BehaviorEvent(identifier, "task6-style", identifier, name, variant, (identifier,), (), float(identifier * 100), 1_000.0)


def test_default_config_has_three_global_variants_but_only_fist_style_overrides() -> None:
    config = MotionStyleConfig()
    assert BEHAVIOR_VARIANT_VOCABULARY == frozenset({"DEFAULT", "FIRM", "FORCEFUL"})
    assert {item.key for item in config.overrides} == {("FIST_COUNTER", "FIRM"), ("FIST_COUNTER", "FORCEFUL")}


def test_default_style_does_not_change_ordinary_macro_timing_or_pose_names() -> None:
    planner = MotionPlanner(clock_ms=lambda: 2_000.0)
    styler = MotionStyler(clock_ms=lambda: 3_000.0)
    source = planner.plan(_behavior(1, "THUMBS_UP_ACK", "DEFAULT"))
    styled = styler.style(source)
    assert styled.style.variant == "DEFAULT"
    assert styled.motion_steps == source.motion_steps
    assert styled.source_motion_plan is source


def test_fist_variants_keep_one_macro_but_have_ordered_timing_styles() -> None:
    planner = MotionPlanner(clock_ms=lambda: 2_000.0)
    styler = MotionStyler(clock_ms=lambda: 3_000.0)
    styled = {
        variant: styler.style(planner.plan(_behavior(index, "FIST_COUNTER", variant)))
        for index, variant in enumerate(("DEFAULT", "FIRM", "FORCEFUL"), start=1)
    }
    assert {item.source_motion_plan.macro_name for item in styled.values()} == {"COUNTER_JAB"}
    assert styled["FORCEFUL"].style.move_duration_scale < styled["FIRM"].style.move_duration_scale < 1.0
    assert styled["FORCEFUL"].style.hold_duration_scale > styled["FIRM"].style.hold_duration_scale
    assert [item.target_pose_name for item in styled["DEFAULT"].motion_steps] == [item.target_pose_name for item in styled["FORCEFUL"].motion_steps]


def test_missing_or_extra_nondefault_override_is_constructor_error() -> None:
    default = MotionStyleConfig()
    with pytest.raises(ValueError, match="vocabulary|coverage"):
        MotionStyler(MotionStyleConfig(overrides=default.overrides[:-1]))
    extra = default.overrides + (MotionStyleOverride("THUMBS_UP_ACK", MotionStyleProfile("FIRM", 0.9, 1.0, 0.95)),)
    with pytest.raises(ValueError, match="coverage"):
        MotionStyler(MotionStyleConfig(overrides=extra))


def test_nondefault_variant_is_rejected_for_an_ordinary_behavior() -> None:
    planner = MotionPlanner(clock_ms=lambda: 2_000.0)
    with pytest.raises(ValueError, match="not configured"):
        planner.plan(_behavior(1, "THUMBS_UP_ACK", "FIRM"))
