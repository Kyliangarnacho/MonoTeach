"""Task 5 declarative Macro coverage and discrete-variant tests."""

from __future__ import annotations

import pytest

from banter.motion_library import MotionLibraryConfig, MotionPlanner
from banter.personality_contracts import BehaviorEvent, PersonaConfig


def _behavior(behavior_id: int, name: str, variant: str) -> BehaviorEvent:
    return BehaviorEvent(behavior_id, "task5-library", behavior_id, name, variant, (behavior_id,), (), float(behavior_id * 100), 1_000.0)


def test_default_library_maps_every_task4_behavior_once_and_preserves_variant() -> None:
    planner = MotionPlanner(clock_ms=lambda: 2_000.0)
    expected = {rule.behavior_name for rule in PersonaConfig().response_rules}
    assert {macro.behavior_name for macro in planner.config.macros} == expected
    plans = tuple(
        planner.plan(_behavior(index, rule.behavior_name, rule.base_variant))
        for index, rule in enumerate(planner.persona_config.response_rules, start=1)
    )
    assert len({plan.macro_name for plan in plans}) == len(plans)
    assert all(plan.source_behavior.variant == rule.base_variant for plan, rule in zip(plans, planner.persona_config.response_rules))
    assert all(plan.motion_steps[-1].target_pose_name == "NEUTRAL" for plan in plans)


def test_variant_is_validated_against_task4_rule_without_changing_macro_identity() -> None:
    planner = MotionPlanner(clock_ms=lambda: 2_000.0)
    playful = planner.plan(_behavior(1, "FIST_COUNTER", "DEFAULT"))
    forceful = planner.plan(_behavior(2, "FIST_COUNTER", "FORCEFUL"))
    assert playful.macro_name == forceful.macro_name == "COUNTER_JAB"
    with pytest.raises(ValueError, match="not configured"):
        planner.plan(_behavior(3, "THUMBS_UP_ACK", "FIRM"))


def test_missing_or_extra_behavior_macro_is_a_constructor_error() -> None:
    default = MotionLibraryConfig()
    with pytest.raises(ValueError, match="coverage"):
        MotionPlanner(MotionLibraryConfig(default.macros[:-1]))
