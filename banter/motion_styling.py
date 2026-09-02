"""Task 6 discrete variant -> StyledMotionPlan expansion; no robot command."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable

from .motion_contracts import MotionPlan, MotionStep
from .motion_style_contracts import MotionStyleOverride, MotionStyleProfile, StyledMotionPlan
from .personality_contracts import BEHAVIOR_VARIANT_VOCABULARY, PersonaConfig


def _monotonic_ms() -> float:
    return time.monotonic() * 1_000.0


@dataclass(frozen=True)
class MotionStyleConfig:
    """One identity default plus explicit overrides for exceptional behaviors."""

    default_profile: MotionStyleProfile = MotionStyleProfile("DEFAULT")
    overrides: tuple[MotionStyleOverride, ...] = (
        MotionStyleOverride("FIST_COUNTER", MotionStyleProfile("FIRM", 0.90, 1.00, 0.95)),
        MotionStyleOverride("FIST_COUNTER", MotionStyleProfile("FORCEFUL", 0.78, 1.20, 0.90)),
    )

    def __post_init__(self) -> None:
        if not isinstance(self.default_profile, MotionStyleProfile) or self.default_profile.variant != "DEFAULT":
            raise ValueError("default_profile must be the DEFAULT MotionStyleProfile.")
        if not isinstance(self.overrides, tuple) or not all(isinstance(item, MotionStyleOverride) for item in self.overrides):
            raise TypeError("overrides must be a tuple of MotionStyleOverride values.")
        if len({item.key for item in self.overrides}) != len(self.overrides):
            raise ValueError("Motion style overrides must not repeat a behavior/variant key.")
        if {self.default_profile.variant, *(item.profile.variant for item in self.overrides)} != BEHAVIOR_VARIANT_VOCABULARY:
            raise ValueError("MotionStyleConfig must expose exactly the global three-variant vocabulary.")

    def profile_for(self, behavior_name: str, variant: str) -> MotionStyleProfile | None:
        if variant == "DEFAULT":
            return self.default_profile
        return next((item.profile for item in self.overrides if item.key == (behavior_name, variant)), None)


class MotionStyler:
    """Apply one known discrete style without changing a behavior's Macro identity."""

    def __init__(
        self,
        config: MotionStyleConfig = MotionStyleConfig(),
        persona_config: PersonaConfig = PersonaConfig(),
        *,
        clock_ms: Callable[[], float] = _monotonic_ms,
    ) -> None:
        if not isinstance(config, MotionStyleConfig) or not isinstance(persona_config, PersonaConfig):
            raise TypeError("config and persona_config must be MotionStyleConfig and PersonaConfig values.")
        if not callable(clock_ms):
            raise TypeError("clock_ms must be callable.")
        expected = {
            (rule.behavior_name, variant)
            for rule in persona_config.response_rules
            for variant in (rule.base_variant, *(item.variant for item in rule.variants))
        }
        actual_overrides = {item.key for item in config.overrides}
        expected_overrides = {item for item in expected if item[1] != "DEFAULT"}
        if actual_overrides != expected_overrides:
            missing, extra = sorted(expected_overrides - actual_overrides), sorted(actual_overrides - expected_overrides)
            raise ValueError(f"Motion style override coverage must match non-DEFAULT Task 4 variants; missing={missing}, extra={extra}.")
        if {variant for _, variant in expected} != BEHAVIOR_VARIANT_VOCABULARY:
            raise ValueError("PersonaConfig must use exactly DEFAULT, FIRM, and FORCEFUL variants.")
        self.config = config
        self.persona_config = persona_config
        self._clock_ms = clock_ms
        self._next_styled_plan_id = 1

    def style(self, motion_plan: MotionPlan) -> StyledMotionPlan:
        if not isinstance(motion_plan, MotionPlan):
            raise TypeError("motion_plan must be a MotionPlan.")
        behavior = motion_plan.source_behavior
        rule = next((item for item in self.persona_config.response_rules if item.behavior_name == behavior.behavior_name), None)
        if rule is None:
            raise ValueError(f"No PersonaConfig rule produces behavior {behavior.behavior_name!r}.")
        allowed = {rule.base_variant, *(item.variant for item in rule.variants)}
        if behavior.variant not in allowed:
            raise ValueError(f"Behavior variant {behavior.variant!r} is not configured for {behavior.behavior_name!r}.")
        profile = self.config.profile_for(behavior.behavior_name, behavior.variant)
        if profile is None:
            raise ValueError(f"No MotionStyleProfile maps {behavior.behavior_name}:{behavior.variant}.")
        steps = tuple(
            MotionStep(step.kind, step.target_pose_name, step.duration_ms * profile.duration_scale_for(step.kind))
            for step in motion_plan.motion_steps
        )
        available = float(self._clock_ms())
        if not math.isfinite(available) or available <= 0.0:
            raise ValueError("Motion styler clock must return a finite positive timestamp in milliseconds.")
        styled = StyledMotionPlan(self._next_styled_plan_id, motion_plan, profile, steps, available)
        self._next_styled_plan_id += 1
        return styled
