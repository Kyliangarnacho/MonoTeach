"""Task 4 response-table, variant, busy, and conflict tests."""

from __future__ import annotations

from banter.contracts import GestureEvent, GestureKind, HandKey
from banter.gesture_grammar import GestureGrammarConfig
from banter.grammar_contracts import GesturePhraseEvent, GrammarUpdate, PhraseForm
from banter.interaction_memory import InteractionMemory
from banter.personality_behavior import PersonaBehaviorEngine, select_behavior
from banter.personality_contracts import AffectDelta, BehaviorAdmission, BehaviorRule, PersonaConfig, PersonaProfile


def _token(event_id: int, gesture: GestureKind, hand: HandKey, t_ms: float) -> GestureEvent:
    return GestureEvent(event_id, gesture, hand, event_id, max(0.0, t_ms - 50.0), t_ms, t_ms + 5.0, min(50.0, t_ms), 0.9, 1)


def _no_grammar() -> GestureGrammarConfig:
    return GestureGrammarConfig(chords=(), sequences=())


def test_first_fist_stays_default_even_when_persona_is_already_annoyed() -> None:
    forceful_config = PersonaConfig(profile=PersonaProfile(baseline_annoyance=0.70))
    memory = InteractionMemory("fist", clock_ms=lambda: 10_000.0)
    engine = PersonaBehaviorEngine("fist", forceful_config, _no_grammar(), clock_ms=lambda: 20_000.0)
    result = engine.update(memory.update(GrammarUpdate((_token(1, GestureKind.FIST, HandKey.LEFT, 100.0),), ())))
    assert result.behavior is not None
    assert result.behavior.behavior_name == "FIST_COUNTER"
    assert result.behavior.variant == "DEFAULT"


def test_same_hand_repeated_fist_escalates_only_after_release_and_fresh_events() -> None:
    memory = InteractionMemory("fist-streak", clock_ms=lambda: 10_000.0)
    engine = PersonaBehaviorEngine("fist-streak", grammar_config=_no_grammar(), clock_ms=lambda: 20_000.0)
    updates = []
    for event_id, t_ms in enumerate((100.0, 1_100.0, 2_100.0), start=1):
        updates.append(engine.update(memory.update(GrammarUpdate((_token(event_id, GestureKind.FIST, HandKey.LEFT, t_ms),), ()))))
    assert [item.behavior.variant for item in updates if item.behavior is not None] == ["DEFAULT", "FIRM", "FORCEFUL"]


def test_busy_input_is_ignored_without_erasing_memory_or_persona_appraisal() -> None:
    memory = InteractionMemory("busy", clock_ms=lambda: 10_000.0)
    engine = PersonaBehaviorEngine("busy", grammar_config=_no_grammar(), clock_ms=lambda: 20_000.0)
    first = engine.update(memory.update(GrammarUpdate((_token(1, GestureKind.POINT_ONE, HandKey.LEFT, 100.0),), ())))
    second = engine.update(memory.update(GrammarUpdate((_token(2, GestureKind.MIDDLE_FINGER, HandKey.RIGHT, 200.0),), ())))
    assert first.behavior is not None and first.admission is BehaviorAdmission.EMITTED
    assert second.behavior is None and second.admission is BehaviorAdmission.IGNORED_BUSY
    assert memory.snapshot.token_total(HandKey.RIGHT, GestureKind.MIDDLE_FINGER) == 1
    assert second.persona.annoyance > first.persona.annoyance


def test_token_and_phrase_share_one_rule_type_and_explicit_priority_decides_conflict() -> None:
    base = PersonaConfig()
    rules = (
        BehaviorRule(GestureKind.POINT_ONE, "POINT_PRIORITY", "DEFAULT", AffectDelta(), priority=100),
        BehaviorRule(base.rule_for(GestureKind.FIST).stimulus, "FIST_COUNTER", "DEFAULT", priority=10),  # type: ignore[union-attr]
        BehaviorRule(base.rule_for(GestureKind.VICTORY_TWO).stimulus, "VICTORY_SALUTE", "DEFAULT", priority=10),  # type: ignore[union-attr]
        BehaviorRule(base.rule_for(GestureKind.OPEN_PALM_FIVE).stimulus, "OPEN_HAND_WAVE", "DEFAULT", priority=10),  # type: ignore[union-attr]
        BehaviorRule(base.rule_for(GestureKind.THUMBS_UP).stimulus, "THUMBS_UP_ACK", "DEFAULT", priority=10),  # type: ignore[union-attr]
        BehaviorRule(base.rule_for(GestureKind.MIDDLE_FINGER).stimulus, "OFFENDED_RETORT", "DEFAULT", priority=10),  # type: ignore[union-attr]
        BehaviorRule(base.rule_for(base.response_rules[6].stimulus).stimulus, "POWER_POSE_REPLY", "DEFAULT", priority=1),  # type: ignore[union-attr]
    )
    config = PersonaConfig(response_rules=rules)
    memory = InteractionMemory("priority", clock_ms=lambda: 10_000.0)
    left = _token(1, GestureKind.FIST, HandKey.LEFT, 100.0)
    right = _token(2, GestureKind.VICTORY_TWO, HandKey.RIGHT, 120.0)
    point = _token(3, GestureKind.POINT_ONE, HandKey.LEFT, 130.0)
    phrase = GesturePhraseEvent(1, "POWER_VICTORY", PhraseForm.CHORD, (left, right), 125.0)
    current = memory.update(GrammarUpdate((left, right, point), (phrase,)))
    persona = PersonaBehaviorEngine("priority", config, _no_grammar(), clock_ms=lambda: 20_000.0)
    draft = persona.update(current).persona
    selected = select_behavior(draft, current, memory.reset("priority"), config, behavior_id=1)
    assert selected.behavior_name == "POINT_PRIORITY"
