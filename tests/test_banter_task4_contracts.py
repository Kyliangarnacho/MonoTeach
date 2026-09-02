"""Contract coverage for Task 4's discrete declarative behavior vocabulary."""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from banter.contracts import GestureKind
from banter.personality_behavior import empty_persona_state
from banter.personality_contracts import (
    AffectDelta,
    BEHAVIOR_VARIANT_VOCABULARY,
    BehaviorAdmission,
    BehaviorEvent,
    BehaviorRule,
    BehaviorVariantRule,
    PersonaConfig,
    Task4Update,
)
from banter import demo_personality_behavior


def test_default_rules_have_per_stimulus_behavior_names_and_no_intensity_field() -> None:
    config = PersonaConfig()
    names = [rule.behavior_name for rule in config.response_rules]
    assert len(names) == len(set(names))
    assert config.rule_for(GestureKind.FIST) is not None
    assert config.rule_for(GestureKind.MIDDLE_FINGER) is not None
    assert BEHAVIOR_VARIANT_VOCABULARY == frozenset({"DEFAULT", "FIRM", "FORCEFUL"})
    event = BehaviorEvent(1, "session", 1, "FIST_COUNTER", "FORCEFUL", (1,), (), 100.0, 101.0)
    assert not hasattr(event, "intensity")
    assert event.to_dict()["variant"] == "FORCEFUL"
    with pytest.raises(FrozenInstanceError):
        empty_persona_state("session").energy = 0.9  # type: ignore[misc]


def test_rules_and_admission_reject_ambiguous_or_inconsistent_state() -> None:
    with pytest.raises(ValueError, match="UNKNOWN"):
        BehaviorRule(GestureKind.UNKNOWN, "BAD", "DEFAULT")
    with pytest.raises(ValueError, match="must not repeat"):
        BehaviorRule(GestureKind.FIST, "FIST_COUNTER", "DEFAULT", variants=(BehaviorVariantRule("FIRM"), BehaviorVariantRule("FIRM")))
    persona = empty_persona_state("session")
    with pytest.raises(ValueError, match="requires a BehaviorEvent"):
        Task4Update(persona, None, BehaviorAdmission.EMITTED)
    with pytest.raises(ValueError, match="requires EMITTED"):
        Task4Update(persona, BehaviorEvent(1, "session", 1, "FIST_COUNTER", "DEFAULT", (1,), (), 100.0, 101.0), BehaviorAdmission.NO_CANDIDATE)
    assert AffectDelta(annoyance=0.2).annoyance == 0.2


def test_demo_cleanup_continues_after_missing_named_window(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[str] = []

    class Stream:
        def release(self) -> None:
            calls.append("release")

    class Perceiver:
        def close(self) -> None:
            calls.append("close")

    def missing_window(_: str) -> None:
        calls.append("destroy_window")
        raise RuntimeError("window already closed")

    monkeypatch.setattr(demo_personality_behavior.cv2, "destroyWindow", missing_window)
    monkeypatch.setattr(demo_personality_behavior.cv2, "destroyAllWindows", lambda: calls.append("destroy_all"))
    monkeypatch.setattr(demo_personality_behavior.cv2, "waitKey", lambda _: calls.append("wait_key"))
    demo_personality_behavior.cleanup_demo_resources(Perceiver(), Stream(), "missing")
    assert calls == ["release", "destroy_window", "destroy_all", "wait_key", "close"]
