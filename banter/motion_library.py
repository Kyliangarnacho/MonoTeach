"""Task 5 declarative BehaviorEvent -> MotionPlan mapping; no robot commands."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable

from .motion_contracts import MotionMacro, MotionPlan, MotionStep, MotionStepKind
from .personality_contracts import BehaviorEvent, PersonaConfig


def _monotonic_ms() -> float:
    return time.monotonic() * 1_000.0


def _move(pose: str, duration_ms: float) -> MotionStep:
    return MotionStep(MotionStepKind.MOVE_POSE, pose, duration_ms)


def _hold(pose: str, duration_ms: float) -> MotionStep:
    return MotionStep(MotionStepKind.HOLD, pose, duration_ms)


def _return(duration_ms: float) -> MotionStep:
    return MotionStep(MotionStepKind.RETURN_NEUTRAL, "NEUTRAL", duration_ms)


@dataclass(frozen=True)
class MotionLibraryConfig:
    """A complete per-behavior macro table, intentionally with no fallback macro."""

    macros: tuple[MotionMacro, ...] = (
        MotionMacro("POINT_BACK", "POINT_JAB", (_move("AIM", 300.0), _move("JAB_EXTEND", 280.0), _hold("JAB_EXTEND", 250.0), _return(450.0))),
        MotionMacro("VICTORY_SALUTE", "SALUTE_SWEEP", (_move("SALUTE_RAISE", 350.0), _move("SALUTE_OUT", 300.0), _hold("SALUTE_OUT", 350.0), _return(500.0))),
        MotionMacro("OPEN_HAND_WAVE", "SIDE_WAVE", (_move("WAVE_LEFT", 330.0), _move("WAVE_RIGHT", 330.0), _move("WAVE_LEFT", 330.0), _return(500.0))),
        MotionMacro("THUMBS_UP_ACK", "UPWARD_ACK", (_move("ACK_UP", 350.0), _hold("ACK_UP", 300.0), _return(450.0))),
        MotionMacro("FIST_COUNTER", "COUNTER_JAB", (_move("RECOIL", 300.0), _move("JAB_EXTEND", 250.0), _hold("JAB_EXTEND", 250.0), _return(500.0))),
        MotionMacro("OFFENDED_RETORT", "RECOIL_DISMISS", (_move("RECOIL", 300.0), _move("DISMISS_LEFT", 260.0), _move("DISMISS_RIGHT", 260.0), _return(500.0))),
        MotionMacro("POWER_POSE_REPLY", "POWER_POSE", (_move("POWER_HIGH", 450.0), _hold("POWER_HIGH", 600.0), _return(550.0))),
        MotionMacro("TRIUMPH_FLOURISH", "TRIUMPH_ARC", (_move("FLOURISH_LOW", 300.0), _move("FLOURISH_HIGH", 420.0), _hold("FLOURISH_HIGH", 450.0), _return(550.0))),
        MotionMacro("PROUD_APPROVAL", "PROUD_DOUBLE_NOD", (_move("PROUD_IN", 260.0), _move("PROUD_UP", 240.0), _move("PROUD_IN", 240.0), _move("PROUD_UP", 240.0), _return(500.0))),
        MotionMacro("HYPE_BOUNCE", "HYPE_DOUBLE_BOUNCE", (_move("HYPE_DOWN", 220.0), _move("HYPE_UP", 220.0), _move("HYPE_DOWN", 220.0), _move("HYPE_UP", 220.0), _return(500.0))),
    )

    def __post_init__(self) -> None:
        if not isinstance(self.macros, tuple) or not self.macros or not all(isinstance(macro, MotionMacro) for macro in self.macros):
            raise ValueError("macros must be a non-empty tuple of MotionMacro values.")
        if len({macro.behavior_name for macro in self.macros}) != len(self.macros):
            raise ValueError("macros must not repeat a behavior_name.")
        if len({macro.macro_name for macro in self.macros}) != len(self.macros):
            raise ValueError("macros must not repeat a macro_name.")

    def macro_for(self, behavior_name: str) -> MotionMacro | None:
        return next((macro for macro in self.macros if macro.behavior_name == behavior_name), None)


class MotionPlanner:
    """Expand known semantic behavior to a named macro without choosing robot geometry."""

    def __init__(
        self,
        config: MotionLibraryConfig = MotionLibraryConfig(),
        persona_config: PersonaConfig = PersonaConfig(),
        *,
        clock_ms: Callable[[], float] = _monotonic_ms,
    ) -> None:
        if not isinstance(config, MotionLibraryConfig) or not isinstance(persona_config, PersonaConfig):
            raise TypeError("config and persona_config must be MotionLibraryConfig and PersonaConfig values.")
        if not callable(clock_ms):
            raise TypeError("clock_ms must be callable.")
        behavior_names = {rule.behavior_name for rule in persona_config.response_rules}
        macro_names = {macro.behavior_name for macro in config.macros}
        if behavior_names != macro_names:
            missing, extra = sorted(behavior_names - macro_names), sorted(macro_names - behavior_names)
            raise ValueError(f"Motion macro coverage must exactly match Persona behavior names; missing={missing}, extra={extra}.")
        self.config = config
        self.persona_config = persona_config
        self._clock_ms = clock_ms
        self._next_plan_id = 1

    def plan(self, behavior: BehaviorEvent) -> MotionPlan:
        """Build one deterministic semantic plan after validating its discrete variant."""
        if not isinstance(behavior, BehaviorEvent):
            raise TypeError("behavior must be a BehaviorEvent.")
        rule = next((item for item in self.persona_config.response_rules if item.behavior_name == behavior.behavior_name), None)
        if rule is None:
            raise ValueError(f"No PersonaConfig rule produces behavior {behavior.behavior_name!r}.")
        allowed_variants = {rule.base_variant, *(item.variant for item in rule.variants)}
        if behavior.variant not in allowed_variants:
            raise ValueError(f"Behavior variant {behavior.variant!r} is not configured for {behavior.behavior_name!r}.")
        macro = self.config.macro_for(behavior.behavior_name)
        if macro is None:  # Constructor coverage prevents this unless a custom config was mutated externally.
            raise ValueError(f"No MotionMacro maps behavior {behavior.behavior_name!r}.")
        available = float(self._clock_ms())
        if not math.isfinite(available) or available <= 0.0:
            raise ValueError("Motion planner clock must return a finite positive timestamp in milliseconds.")
        plan = MotionPlan(self._next_plan_id, behavior, macro.macro_name, macro.steps, available)
        self._next_plan_id += 1
        return plan
