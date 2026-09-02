"""Synthetic Task 2 → Task 3 → Task 4 delayed-response integration tests."""

from __future__ import annotations

from banter.contracts import GestureEvent, GestureKind, HandKey
from banter.gesture_grammar import GestureGrammarConfig, GestureGrammarEngine
from banter.grammar_contracts import GrammarUpdate
from banter.interaction_memory import InteractionMemory
from banter.personality_behavior import PersonaBehaviorEngine
from banter.personality_contracts import BehaviorAdmission


def _token(event_id: int, gesture: GestureKind, hand: HandKey, t_ms: float) -> GestureEvent:
    return GestureEvent(event_id, gesture, hand, event_id, max(0.0, t_ms - 50.0), t_ms, t_ms + 5.0, min(50.0, t_ms), 0.9, 1)


def _pipeline(session: str):
    grammar_config = GestureGrammarConfig()
    return (
        GestureGrammarEngine(grammar_config, clock_ms=lambda: 10_000.0),
        InteractionMemory(session, clock_ms=lambda: 20_000.0),
        PersonaBehaviorEngine(session, grammar_config=grammar_config, clock_ms=lambda: 30_000.0),
    )


def test_sequence_wake_buffers_prefix_and_emits_only_completed_phrase_behavior() -> None:
    grammar, memory, engine = _pipeline("sequence")
    open_update = engine.update(memory.update(grammar.update((_token(1, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 100.0),))))
    fist_update = engine.update(memory.update(grammar.update((_token(2, GestureKind.FIST, HandKey.LEFT, 500.0),))))
    victory_update = engine.update(memory.update(grammar.update((_token(3, GestureKind.VICTORY_TWO, HandKey.LEFT, 900.0),))))
    assert open_update.behavior is None and open_update.admission is BehaviorAdmission.DEFERRED_PREFIX
    assert fist_update.behavior is None and fist_update.admission is BehaviorAdmission.DEFERRED_PREFIX
    assert victory_update.behavior is not None
    assert victory_update.behavior.behavior_name == "TRIUMPH_FLOURISH"
    assert victory_update.behavior.source_token_event_ids == (1, 2, 3)


def test_sequence_timeout_releases_wake_token_once() -> None:
    grammar, memory, engine = _pipeline("sequence-timeout")
    deferred = engine.update(memory.update(grammar.update((_token(1, GestureKind.OPEN_PALM_FIVE, HandKey.LEFT, 100.0),))))
    released = engine.advance_time(2_601.0)
    assert deferred.behavior is None and deferred.admission is BehaviorAdmission.DEFERRED_PREFIX
    assert released.behavior is not None and released.behavior.behavior_name == "OPEN_HAND_WAVE"
    assert engine.advance_time(2_602.0) is released


def test_split_frame_chord_defers_first_token_then_emits_one_phrase_and_task3_accepts_right_first() -> None:
    grammar, memory, engine = _pipeline("chord")
    first = engine.update(memory.update(grammar.update((_token(39, GestureKind.VICTORY_TWO, HandKey.RIGHT, 100.0),))))
    completed = engine.update(memory.update(grammar.update((_token(40, GestureKind.FIST, HandKey.LEFT, 120.0),))))
    assert first.behavior is None and first.admission is BehaviorAdmission.DEFERRED_PREFIX
    assert completed.behavior is not None and completed.behavior.behavior_name == "POWER_POSE_REPLY"
    assert completed.behavior.source_token_event_ids == (39, 40)
    assert memory.snapshot.total_phrase_count == 1


def test_chord_timeout_releases_standalone_once() -> None:
    grammar, memory, engine = _pipeline("chord-timeout")
    deferred = engine.update(memory.update(grammar.update((_token(1, GestureKind.VICTORY_TWO, HandKey.RIGHT, 100.0),))))
    released = engine.advance_time(551.0)
    assert deferred.admission is BehaviorAdmission.DEFERRED_PREFIX
    assert released.behavior is not None and released.behavior.behavior_name == "VICTORY_SALUTE"
